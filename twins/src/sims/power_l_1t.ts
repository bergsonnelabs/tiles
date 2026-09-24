// Digital twin for Power.L.1T — TI BQ25150 single-cell Li-Ion linear charger +
// LDO + ADC.
//
// Pad map (Power-L-1T-b.json): GND (1, switched), LP (2), SW (3), I²C CLK/DAT
// (4/5), BATT- (6), BATT+ (7), SUPPLY+ (8, 3.15–5.5 V charge input), SUPPLY- (9),
// V+ (10, 1.8 V LDO out; the BQ25150 LDO is rated 100 mA, datasheet §7.3 ILDO).
//
// THE INTERESTING GROUND: the downstream GND (pad 1) is NOT a hard tie — it
// reaches BATT- (pad 6) only through an on-board SI8806 MOSFET whose gate follows
// the LDO through 220 kΩ. When the LDO is off (SW→GND, ship mode, no power) the
// MOSFET opens and the downstream system's ground floats: a true soft off. The
// SI8806's VGS(th) is 0.4–1.0 V, so an LDO set below 1.0 V can't be counted on
// to close it either.
//
// NO BATTERY THERMISTOR: TS is strapped to a fixed 5 kΩ (the datasheet's "TS
// not used" connection), so the TS pin sits at 5 kΩ × 80 µA = 400 mV, inside the
// normal band, whatever the cell's temperature. There is no temperature stimulus.
//
// ONE state, two readers. The firmware's calls (worker) and the power layer (main
// thread) read the same fields — the names the driver's setters use. The power
// layer takes VIN / battery / LP / SW from the WIRING (`ctx.padVoltage`) when it
// is given, like Power.L.1N; the `vin_present` / `battery_present` / `lp_mode` /
// `sw_grounded` stimuli only stand in when the twin runs on its own (and for the
// firmware's reads, which have no wiring to look at).
//
// Conversions and register semantics follow tile_power_l_1t.c + the BQ25150
// datasheet (SLUSD04B): VUVLO 3.4 V rising, VOVP 5.5 V, VSLP 130 mV, VLOWV 3.0 V,
// register reset values (VBATREG 4.2 V, ITERM 10 %, ILIM 100 mA, LDO 1.8 V), the
// driver's init (ICHG 80 mA, IPRECHG 18.75 mA, VBATREG 4.2 V, BUVLO 2.6 V), the
// quiescent currents in §7.5, and the TS thresholds (VHOT 0.185 / VWARM 0.265 /
// VCOOL 0.514 / VCOLD 0.585 V).
import type { PowerCtx, PowerRail, TileSim } from '../tileSim';

const DEVICE_ID = 0x20;

// datasheet §7.5
const VUVLO_MV = 3400; // IN active threshold, VIN rising
const VOVP_MV = 5500; // input over-voltage threshold
const VSLP_MV = 130; // sleep exit: VIN must exceed VBAT by this
const VLOWV_MV = 3000; // pre-charge → fast-charge threshold (VLOWV_SEL default)
const BATT_DETECT_MV = 2000; // below this there's no cell to charge (modeling choice)
const LP_VIH_MV = 1350; // /LP input-high threshold (900 kΩ pull-down → unwired = low)
const SW_VIL_MV = 450; // SW read as tied to GND below this
const IIN_QUIESCENT_UA = 780; // VIN supply current, charge disabled
const IBAT_SHIP_UA = 0.01; // 10 nA ship mode
const IBAT_LP_UA = 0.46; // low-power mode, LDO disabled
const IBAT_LP_LDO_UA = 1.7; // low-power mode, LDO enabled
const IBAT_ACTIVE_UA = 18; // active battery mode, LDO disabled
const IBAT_ACTIVE_LDO_UA = 21; // active battery mode, LDO enabled
// LS/LDO output current, datasheet §7.3 ILDO max 100 mA. (The tile JSON's pad 10
// note says 10 mA and its application note 150 mA; the part's rating is used.)
const LDO_LIMIT_UA = 100_000;
// SI8806 ground switch: gate = V+ via 220 kΩ; VGS(th) max 1.0 V (SI8806DB §Specs).
const GND_SWITCH_VTH_MV = 1000;
// TS pin: fixed 5 kΩ strap × 80 µA ITS_BIAS (datasheet §7.5, pin table "TS").
const TS_STRAP_MV = 400;

// TS thresholds, mV (datasheet VHOT/VWARM/VCOOL/VCOLD, default registers).
const TS_HOT_MV = 185;
const TS_COLD_MV = 585;

// charge_state codes
const CS_IDLE = 0;
const CS_PRE_CHARGE = 1;
const CS_FAST_CHARGE = 2;
const CS_DONE = 3;

// power_l_1t_ldo_mode_t / power_l_1t_pmid_mode_t
const LDO_MODE_LOAD_SWITCH = 1;
const PMID_AUTO = 0;
const PMID_BAT_ONLY = 1;

interface State {
  // ── stimuli (the world around the tile) ──
  vbat_mv: number; // cell voltage
  battery_present: number; // a cell is attached
  vin_mv: number; // adapter voltage on SUPPLY+ when plugged in
  vin_present: number; // adapter plugged in
  lp_mode: number; // LP pad held low (900 kΩ pull-down) → low-power mode on battery
  sw_grounded: number; // SW pad tied to GND → LDO output off
  adcin_mv: number; // ADCIN pin (not routed to a pad on this tile)

  // ── chip status (derived each tick) ──
  charge_state: number; // CS_* — idle / pre-charge / fast-charge / done
  shipped: number; // 1 while the chip is latched in ship mode

  // ── charge config (driver setters; values as the chip quantizes them) ──
  charging_enabled: number; // ICCTRL2.CHARGER_DISABLE inverted
  charge_current_ma: number; // ICHG
  charge_voltage_mv: number; // VBATREG
  pre_charge_cma: number; // IPRECHG, centi-mA (1.25 mA steps)
  termination_pct: number; // ITERM, % of ICHG; 0 = termination disabled
  input_current_limit_ma: number; // ILIM
  batt_uvlo_mv: number; // BUVLO
  safety_timer: number; // power_l_1t_safety_timer_t
  pmid_mode: number; // power_l_1t_pmid_mode_t

  // ── LDO ──
  ldo_voltage_mv: number;
  ldo_mode: number; // 0 LDO, 1 load switch
  ldo_enabled: number;

  // ── TS ──
  ts_enabled: number;
  ts_cold_code: number;
  ts_cool_code: number;
  ts_warm_code: number;
  ts_hot_code: number;

  // ── ADC comparators ──
  adc_comp1_ch: number;
  adc_comp2_ch: number;
  adc_comp3_ch: number;

  ship_mode: number; // EN_SHIP_MODE armed (enters ship when VIN is removed)
}

const num = (args: number[], i: number, dflt: number) =>
  args.length > i && Number.isFinite(args[i]) ? Math.trunc(args[i]!) : dflt;
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

// TS pin voltage. Fixed: the tile has a 5 kΩ strap, not a thermistor.
const tsMv = (): number => TS_STRAP_MV;

// The inputs the chip sees. With a solved system (`ctx`) the wiring decides; a
// wired pad with nothing driving it reads 0. Without one, the stimuli stand in.
interface Inputs {
  vinV: number;
  battV: number;
  lpLow: boolean;
  swGrounded: boolean;
}
function resolveInputs(s: State, ctx?: PowerCtx): Inputs {
  const pv = ctx?.padVoltage;
  const wired = ctx != null;
  const vinPad = pv?.['8'];
  const battPad = pv?.['7'];
  const lpPad = pv?.['2'];
  const swPad = pv?.['3'];
  return {
    vinV: vinPad != null ? vinPad : wired ? 0 : s.vin_present ? s.vin_mv : 0,
    battV: battPad != null ? battPad : wired ? 0 : s.battery_present ? s.vbat_mv : 0,
    // LP has a 900 kΩ pull-down: an unwired LP pad in a system is LOW.
    lpLow: lpPad != null ? lpPad < LP_VIH_MV : wired ? true : s.lp_mode === 1,
    swGrounded: swPad != null ? swPad < SW_VIL_MV : wired ? false : s.sw_grounded === 1,
  };
}

// VIN is a usable input (the chip leaves battery-only operation).
const vinValid = (i: Inputs) => i.vinV >= VUVLO_MV && i.vinV <= VOVP_MV;
// STAT0.VIN_PGOOD: valid AND above VBAT + VSLP.
const vinPgood = (i: Inputs) => vinValid(i) && i.vinV > i.battV + VSLP_MV;
const batteryPresent = (i: Inputs) => i.battV >= BATT_DETECT_MV;

// Ship mode: EN_SHIP_MODE arms it; the chip enters when VIN goes away and stays
// latched until VIN returns (datasheet §8.4.1).
const inShip = (s: State, i: Inputs) => !vinValid(i) && (s.shipped === 1 || s.ship_mode === 1);

// BATFET conducting: not shipped, cell above BUVLO.
const batteryOn = (s: State, i: Inputs) => !inShip(s, i) && i.battV >= s.batt_uvlo_mv;
const poweredUp = (s: State, i: Inputs) => vinValid(i) || batteryOn(s, i);
// Low-power mode (LP low, on battery): I²C and the ADC are off.
const lowPower = (i: Inputs) => i.lpLow && !vinValid(i);
// The part answers on I²C.
const i2cAlive = (s: State, i: Inputs) => poweredUp(s, i) && !lowPower(i);

const tsFault = (s: State) => {
  if (!s.ts_enabled) return false;
  const ts = tsMv();
  return ts <= TS_HOT_MV || ts >= TS_COLD_MV;
};

// What the chip's charger is actually doing.
function chargeState(s: State, i: Inputs): number {
  if (
    !vinPgood(i) ||
    !batteryPresent(i) ||
    !s.charging_enabled ||
    s.pmid_mode !== PMID_AUTO || // BAT_ONLY stops charging; FLOAT / PULLDOWN cut PMID
    tsFault(s)
  )
    return CS_IDLE;
  if (i.battV >= s.charge_voltage_mv) return s.termination_pct > 0 ? CS_DONE : CS_IDLE;
  return i.battV < VLOWV_MV ? CS_PRE_CHARGE : CS_FAST_CHARGE;
}

// Current flowing into the cell, mA — the programmed level, capped by ILIM.
function chargeMa(s: State, i: Inputs): number {
  const cs = chargeState(s, i);
  const target =
    cs === CS_PRE_CHARGE ? s.pre_charge_cma / 100 : cs === CS_FAST_CHARGE ? s.charge_current_ma : 0;
  return Math.min(target, s.input_current_limit_ma);
}

function pmidMv(s: State, i: Inputs): number {
  if (s.pmid_mode === PMID_AUTO) return vinValid(i) ? i.vinV : batteryOn(s, i) ? i.battV : 0;
  if (s.pmid_mode === PMID_BAT_ONLY) return batteryOn(s, i) ? i.battV : 0;
  return 0; // floating / pulled down
}

// The LDO's input (VINLS) is tied to PMID on the tile (schematic). The SI8806
// ground switch follows the LDO output.
const ldoOn = (s: State, i: Inputs) =>
  s.ldo_enabled === 1 && !i.swGrounded && poweredUp(s, i) && pmidMv(s, i) > 0;
const ldoOutMv = (s: State, i: Inputs) =>
  !ldoOn(s, i)
    ? 0
    : s.ldo_mode === LDO_MODE_LOAD_SWITCH
      ? pmidMv(s, i)
      : Math.min(s.ldo_voltage_mv, pmidMv(s, i));

// Driver: percent = (VBAT − 3000) · 100 / 1200, integer, clamped 0…100.
function percentOf(mv: number): number {
  if (mv <= 3000) return 0;
  if (mv >= 4200) return 100;
  return Math.floor(((mv - 3000) * 100) / 1200);
}

// The driver's view: STAT0 / STAT1 bits.
function stat0(s: State, i: Inputs): number {
  const cs = chargeState(s, i);
  let b = 0;
  if (vinPgood(i)) b |= 0x01;
  if (cs === CS_DONE) b |= 0x20;
  if (cs === CS_FAST_CHARGE && i.battV >= s.charge_voltage_mv - 100) b |= 0x40; // CV taper
  return b;
}
function stat1(s: State, i: Inputs): number {
  const ts = tsMv();
  let b = 0;
  if (i.vinV > VOVP_MV) b |= 0x80;
  if (batteryPresent(i) && i.battV < s.batt_uvlo_mv) b |= 0x10;
  if (s.ts_enabled) {
    if (ts >= TS_COLD_MV) b |= 0x08;
    else if (ts >= 514) b |= 0x04;
    if (ts <= TS_HOT_MV) b |= 0x01;
    else if (ts <= 265) b |= 0x02;
  }
  return b;
}
// tile_power_l_1t_get_charge_status().charging — the driver derives it from
// STAT0/STAT1 (PGOOD, not done, no UVLO / OVP / TS cold / TS hot) plus the
// software gates: ICCTRL2.CHARGER_DISABLE clear and PMID_MODE = AUTO.
function driverCharging(s: State, i: Inputs): boolean {
  const s0 = stat0(s, i);
  const s1 = stat1(s, i);
  return (
    (s0 & 0x01) !== 0 &&
    (s0 & 0x20) === 0 &&
    (s1 & 0x99) === 0 &&
    s.charging_enabled === 1 &&
    s.pmid_mode === PMID_AUTO
  );
}

// Charge-current register quantization (driver set_charge_current_ma).
function quantizeIchg(ma: number): number {
  const m = Math.max(0, Math.min(500, ma));
  if (m > 318) return Math.floor((Math.min(255, Math.floor((m * 2) / 5)) * 5) / 2);
  return Math.floor((Math.min(255, Math.floor((m * 4) / 5)) * 5) / 4);
}

const sim: TileSim<State> = {
  tile: 'Power.L.1T',

  // After tile_power_l_1t_init(): ICHG 80 mA, IPRECHG 18.75 mA, VBATREG 4.2 V,
  // BUVLO 2.6 V, TS on, 6 h timer, watchdog off, ship mode cleared; everything
  // else at its register reset (ITERM 10 %, ILIM 100 mA, LDO on at 1.8 V).
  defaultState: {
    vbat_mv: 3800,
    battery_present: 1,
    vin_mv: 5000,
    vin_present: 0,
    lp_mode: 0,
    sw_grounded: 0,
    adcin_mv: 0,

    charge_state: CS_IDLE,
    shipped: 0,

    charging_enabled: 1,
    charge_current_ma: 80,
    charge_voltage_mv: 4200,
    pre_charge_cma: 1875,
    termination_pct: 10,
    input_current_limit_ma: 100,
    batt_uvlo_mv: 2600,
    safety_timer: 1,
    pmid_mode: PMID_AUTO,

    ldo_voltage_mv: 1800,
    ldo_mode: 0,
    ldo_enabled: 1,

    ts_enabled: 1,
    ts_cold_code: 0,
    ts_cool_code: 0,
    ts_warm_code: 0,
    ts_hot_code: 0,

    adc_comp1_ch: 0,
    adc_comp2_ch: 0,
    adc_comp3_ch: 0,

    ship_mode: 0,
  },

  // Physical stimuli. In a wired system the SUPPLY+ / BATT+ / LP / SW pads come
  // from the wiring; these stand in for a tile on its own.
  controls: [
    {
      type: 'slider',
      field: 'vbat_mv',
      label: 'Battery voltage',
      min: 2400,
      max: 4500,
      step: 10,
      unit: 'mV',
      description: 'Cell voltage — get_vbat_mv reads it; get_percent maps 3.0–4.2 V to 0–100 %.',
    },
    { type: 'toggle', field: 'battery_present', label: 'Battery attached' },
    { type: 'toggle', field: 'vin_present', label: 'VIN plugged in' },
    {
      type: 'slider',
      field: 'vin_mv',
      label: 'VIN voltage',
      min: 3000,
      max: 6000,
      step: 50,
      unit: 'mV',
      description: 'Adapter voltage on SUPPLY+ (pad 8). Valid 3.4–5.5 V; above 5.5 V is OVP.',
    },
    {
      type: 'toggle',
      field: 'lp_mode',
      label: 'LP pad low',
      description: 'On battery, a low LP pad puts the chip in low-power mode (I²C and ADC off).',
    },
    { type: 'toggle', field: 'sw_grounded', label: 'SW tied to GND (LDO off)' },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_power_l_1t_find: () => ({ scalar: 1 }),
    // init (tile_power_l_1t.c): the charge settings it writes, and the ship
    // request it clears. ILIM, ITERM, the LDO and charger-enable are untouched.
    tile_power_l_1t_init: () => ({
      nextState: {
        charge_current_ma: 80,
        pre_charge_cma: 1875,
        charge_voltage_mv: 4200,
        batt_uvlo_mv: 2600,
        ts_enabled: 1,
        safety_timer: 1,
        ship_mode: 0,
      },
    }),

    // ── charge config ──
    tile_power_l_1t_set_charge_current_ma: ({ args }) => ({
      nextState: { charge_current_ma: quantizeIchg(num(args, 0, 80)) },
    }),
    tile_power_l_1t_set_charge_voltage_mv: ({ args }) => {
      const mv = clamp(num(args, 0, 4200), 3600, 4600);
      return { nextState: { charge_voltage_mv: 3600 + Math.floor((mv - 3600) / 10) * 10 } };
    },
    tile_power_l_1t_set_pre_charge_ma: ({ state, args }) => {
      // IPRECHG step follows ICHARGE_RANGE (set when ICHG > 318 mA).
      const range1 = state.charge_current_ma > 318;
      const ma = Math.max(0, num(args, 0, 18) & 0xff);
      const code = range1
        ? Math.min(31, Math.floor((Math.min(ma, 77) * 2) / 5))
        : Math.min(31, Math.floor((Math.min(ma, 38) * 4) / 5));
      return { nextState: { pre_charge_cma: code * (range1 ? 250 : 125) } };
    },
    tile_power_l_1t_set_termination_percent: ({ args }) => ({
      nextState: { termination_pct: Math.min(31, Math.max(0, num(args, 0, 10) & 0xff)) },
    }),
    tile_power_l_1t_set_input_current_limit_ma: ({ args }) => {
      // Largest ILIMCTRL level ≤ the request; 50 mA floor.
      const ma = num(args, 0, 100);
      const levels = [600, 500, 400, 300, 200, 150, 100];
      return { nextState: { input_current_limit_ma: levels.find((l) => ma >= l) ?? 50 } };
    },
    tile_power_l_1t_charger_enable: ({ args }) => ({
      nextState: { charging_enabled: num(args, 0, 1) ? 1 : 0 },
    }),
    tile_power_l_1t_set_battery_uvlo_mv: ({ args }) => {
      const mv = num(args, 0, 3000);
      const levels = [3000, 2800, 2600, 2400];
      return { nextState: { batt_uvlo_mv: levels.find((l) => mv >= l) ?? 2200 } };
    },
    tile_power_l_1t_set_safety_timer: ({ args }) => ({
      nextState: { safety_timer: num(args, 0, 1) & 0x03 },
    }),
    tile_power_l_1t_set_pmid_mode: ({ args }) => ({
      nextState: { pmid_mode: num(args, 0, 0) & 0x03 },
    }),

    // ── ADC reads (0 when the chip isn't answering: ship, LP, unpowered) ──
    tile_power_l_1t_get_vbat_mv: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) ? i.battV : 0 };
    },
    tile_power_l_1t_get_vin_mv: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) ? Math.min(i.vinV, 6000) : 0 };
    },
    tile_power_l_1t_get_pmid_mv: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) ? pmidMv(state, i) : 0 };
    },
    tile_power_l_1t_get_charge_current_ma: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) ? Math.floor(chargeMa(state, i)) : 0 };
    },
    tile_power_l_1t_get_input_current_ma: ({ state }) => {
      // IIN is only valid with VIN above VUVLO: charge + the chip's own ~0.78 mA.
      const i = resolveInputs(state);
      return {
        scalar:
          i2cAlive(state, i) && vinValid(i)
            ? Math.floor(chargeMa(state, i) + IIN_QUIESCENT_UA / 1000)
            : 0,
      };
    },
    tile_power_l_1t_get_ts_mv: ({ state }) => {
      const i = resolveInputs(state);
      // TS isn't measured in low-power mode, like the rest of the ADC.
      return { scalar: i2cAlive(state, i) ? tsMv() : 0 };
    },
    tile_power_l_1t_get_adcin_mv: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) ? clamp(state.adcin_mv, 0, 1200) : 0 };
    },
    tile_power_l_1t_get_percent: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) ? percentOf(i.battV) : 0 };
    },

    // ── TS ──
    tile_power_l_1t_set_ts_cold: ({ args }) => ({
      nextState: { ts_cold_code: num(args, 0, 0) & 0xff },
    }),
    tile_power_l_1t_set_ts_cool: ({ args }) => ({
      nextState: { ts_cool_code: num(args, 0, 0) & 0xff },
    }),
    tile_power_l_1t_set_ts_warm: ({ args }) => ({
      nextState: { ts_warm_code: num(args, 0, 0) & 0xff },
    }),
    tile_power_l_1t_set_ts_hot: ({ args }) => ({
      nextState: { ts_hot_code: num(args, 0, 0) & 0xff },
    }),
    tile_power_l_1t_set_ts_enabled: ({ args }) => ({
      nextState: { ts_enabled: num(args, 0, 1) ? 1 : 0 },
    }),

    // ── LDO ──
    tile_power_l_1t_set_ldo_voltage_mv: ({ args }) => {
      const mv = clamp(num(args, 0, 1800), 600, 3700);
      return { nextState: { ldo_voltage_mv: 600 + Math.floor((mv - 600) / 100) * 100 } };
    },
    tile_power_l_1t_set_ldo_mode: ({ args }) => ({
      nextState: { ldo_mode: num(args, 0, 0) === LDO_MODE_LOAD_SWITCH ? 1 : 0 },
    }),
    tile_power_l_1t_set_ldo_enabled: ({ args }) => ({
      nextState: { ldo_enabled: num(args, 0, 1) ? 1 : 0 },
    }),

    // ── status ──
    // Fills a power_l_1t_status_t the program can't read back through the DSL.
    tile_power_l_1t_get_charge_status: () => ({}),
    tile_power_l_1t_read_status: ({ state, args }) => {
      const i = resolveInputs(state);
      if (!i2cAlive(state, i)) return { scalar: 0 };
      const r1 = state.charge_current_ma > 318;
      const regs: Record<number, number> = {
        0x00: stat0(state, i),
        0x01: stat1(state, i),
        0x12: Math.floor((state.charge_voltage_mv - 3600) / 10) & 0x7f,
        0x13: Math.floor((state.charge_current_ma * (r1 ? 2 : 4)) / 5) & 0xff,
        0x14: (r1 ? 0x80 : 0) | (Math.floor(state.pre_charge_cma / (r1 ? 250 : 125)) & 0x1f),
        0x15: state.termination_pct === 0 ? 0x01 : (state.termination_pct & 0x1f) << 1,
        0x19: [50, 100, 150, 200, 300, 400, 500, 600].indexOf(state.input_current_limit_ma) & 0x07,
        0x1d:
          (state.ldo_enabled ? 0x80 : 0) |
          ((Math.floor((state.ldo_voltage_mv - 600) / 100) & 0x1f) << 2) |
          (state.ldo_mode ? 0x02 : 0),
        0x35: (state.ship_mode ? 0x80 : 0) | 0x10,
        0x6f: DEVICE_ID,
      };
      return { scalar: regs[num(args, 0, 0) & 0xff] ?? 0 };
    },
    tile_power_l_1t_write_reg: () => ({}),
    tile_power_l_1t_is_charging: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) && driverCharging(state, i) ? 1 : 0 };
    },
    tile_power_l_1t_is_charge_done: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) && chargeState(state, i) === CS_DONE ? 1 : 0 };
    },
    tile_power_l_1t_is_battery_low: ({ state, args }) => {
      const i = resolveInputs(state);
      const pct = i2cAlive(state, i) ? percentOf(i.battV) : 0;
      return { scalar: pct < (num(args, 0, 0) & 0xff) ? 1 : 0 };
    },
    tile_power_l_1t_is_powered: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) && vinPgood(i) ? 1 : 0 };
    },
    // Charge evolves over minutes; the state is constant within one call, so
    // "done now?" is the answer the poll loop would reach.
    tile_power_l_1t_wait_for_charge_done: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) && chargeState(state, i) === CS_DONE ? 1 : 0 };
    },

    // ── power management ──
    // Arms EN_SHIP_MODE; the chip enters ship mode once VIN is gone.
    tile_power_l_1t_enter_ship_mode: () => ({ nextState: { ship_mode: 1 } }),

    // ── ADC comparators ──
    tile_power_l_1t_set_adc_comparator: ({ args }) => {
      const comp = num(args, 0, 1);
      const ch = num(args, 1, 0) & 0x07;
      if (comp === 1) return { nextState: { adc_comp1_ch: ch } };
      if (comp === 2) return { nextState: { adc_comp2_ch: ch } };
      if (comp === 3) return { nextState: { adc_comp3_ch: ch } };
      return {};
    },
    // Threshold crossings aren't modeled: no alarm flags (FLAG2[6:4] = 0).
    tile_power_l_1t_get_adc_comparators: () => ({ scalar: 0 }),
  },

  provenance: {
    tile_power_l_1t_find: 'canonical', // fixed 0x6B
    tile_power_l_1t_init: 'canonical', // defaultState = post-init registers
    tile_power_l_1t_set_charge_current_ma: 'canonical', // 1.25 / 2.5 mA steps, 500 mA cap
    tile_power_l_1t_set_charge_voltage_mv: 'canonical', // 3.6 V + 10 mV·code
    tile_power_l_1t_set_pre_charge_ma: 'canonical', // IPRECHG steps follow ICHARGE_RANGE
    tile_power_l_1t_set_termination_percent: 'canonical',
    tile_power_l_1t_set_input_current_limit_ma: 'canonical', // ILIMCTRL table, floor
    tile_power_l_1t_charger_enable: 'canonical',
    tile_power_l_1t_set_battery_uvlo_mv: 'canonical', // BUVLO table
    tile_power_l_1t_set_safety_timer: 'inferred', // stored; timer expiry not modeled
    tile_power_l_1t_set_pmid_mode: 'canonical',
    tile_power_l_1t_get_vbat_mv: 'canonical',
    tile_power_l_1t_get_vin_mv: 'canonical',
    tile_power_l_1t_get_pmid_mv: 'inferred', // PMID ≈ VIN / VBAT (FET drop ignored)
    tile_power_l_1t_get_charge_current_ma: 'inferred', // CC step, no CV taper
    tile_power_l_1t_get_input_current_ma: 'inferred', // no system load in the worker
    tile_power_l_1t_get_ts_mv: 'canonical', // 5 kΩ strap × 80 µA bias = 400 mV
    tile_power_l_1t_get_adcin_mv: 'hallucinated', // ADCIN isn't routed on this tile
    tile_power_l_1t_get_percent: 'canonical', // driver's linear 3.0–4.2 V
    tile_power_l_1t_set_ts_cold: 'inferred', // codes stored; thresholds stay default
    tile_power_l_1t_set_ts_cool: 'inferred',
    tile_power_l_1t_set_ts_warm: 'inferred',
    tile_power_l_1t_set_ts_hot: 'inferred',
    tile_power_l_1t_set_ts_enabled: 'canonical',
    tile_power_l_1t_set_ldo_voltage_mv: 'canonical', // 600 mV + 100 mV·code
    tile_power_l_1t_set_ldo_mode: 'canonical', // VINLS tied to PMID (schematic)
    tile_power_l_1t_set_ldo_enabled: 'canonical',
    tile_power_l_1t_get_charge_status: 'inferred', // struct not readable from the DSL
    tile_power_l_1t_read_status: 'inferred', // STAT0/1, config regs, DEVICE_ID; rest 0
    tile_power_l_1t_write_reg: 'hallucinated', // raw writes not modeled
    tile_power_l_1t_is_charging: 'canonical', // the driver's STAT0/STAT1 derivation
    tile_power_l_1t_is_charge_done: 'canonical',
    tile_power_l_1t_is_battery_low: 'canonical',
    tile_power_l_1t_is_powered: 'canonical', // VIN_PGOOD
    tile_power_l_1t_wait_for_charge_done: 'inferred',
    tile_power_l_1t_enter_ship_mode: 'canonical', // arms; enters on VIN removal
    tile_power_l_1t_set_adc_comparator: 'inferred',
    tile_power_l_1t_get_adc_comparators: 'inferred',
    power: 'inferred', // quiescents + LDO rating canonical; charge/LDO currents modeled
  },

  // Each tick: latch/unlatch ship mode and publish the charge state, from the
  // stimuli (the worker has no wiring to read).
  deriveState(state) {
    const i = resolveInputs(state);
    const update: Partial<State> = {};
    if (vinValid(i)) {
      // VIN wakes a latched chip; a still-armed bit waits for VIN to go.
      if (state.shipped) {
        update.shipped = 0;
        update.ship_mode = 0;
      }
    } else if (state.ship_mode && !state.shipped) {
      update.shipped = 1;
    }
    const cs = chargeState({ ...state, ...update }, i);
    if (cs !== state.charge_state) update.charge_state = cs;
    return update;
  },

  // V+ (pad 10) and the switched ground (pad 1, the SI8806 follows the LDO).
  padOutputs(state) {
    const i = resolveInputs(state);
    const on = ldoOn(state, i) && ldoOutMv(state, i) >= GND_SWITCH_VTH_MV ? 1 : 0;
    return { '10': on, '1': on };
  },

  // Electrical. SUPPLY+ (8) and BATT+ (7) are always declared so the solver
  // tracks their pads; LP (2) and SW (3) too, so the chip can read their level.
  // On VIN, SUPPLY's current is left undeclared (the solver makes it what V+
  // delivers) and the chip's own quiescent + charge current rides on draw_ua; on
  // battery the same holds for BATT.
  power(state, ctx) {
    const i = resolveInputs(state, ctx);
    const onVin = vinValid(i);
    const shipped = inShip(state, i);
    const chargeUa = Math.round(chargeMa(state, i) * 1000);
    // Below the SI8806's VGS(th) the downstream ground (pad 1) isn't closed, so
    // nothing on V+ has a return path: treat the output as off.
    const ldoEn = ldoOn(state, i) && ldoOutMv(state, i) >= GND_SWITCH_VTH_MV;
    const vout = ldoEn ? ldoOutMv(state, i) : 0;
    const rails: PowerRail[] = [];

    rails.push({
      name: 'SUPPLY',
      role: 'supply',
      v_mv: i.vinV,
      ...(onVin ? {} : { i_ua: 0 }),
      pads: ['8'],
      limit_ua: state.input_current_limit_ma * 1000,
      note: onVin
        ? chargeUa > 0
          ? 'input — charging + system'
          : 'input'
        : i.vinV > VOVP_MV
          ? 'input over-voltage (OVP)'
          : 'no input',
    });
    rails.push(
      onVin
        ? {
            name: 'BATT',
            role: 'supply',
            v_mv: i.battV,
            i_ua: 0,
            pads: ['7'],
            note: chargeUa > 0 ? 'battery receiving charge' : 'battery idle',
          }
        : shipped || !batteryOn(state, i)
          ? {
              name: 'BATT',
              role: 'supply',
              v_mv: i.battV,
              i_ua: shipped && batteryPresent(i) ? IBAT_SHIP_UA : 0,
              pads: ['7'],
              note: shipped ? 'ship mode — battery disconnected' : 'battery below UVLO',
            }
          : {
              name: 'BATT',
              role: 'supply',
              v_mv: i.battV,
              pads: ['7'],
              note: 'battery input (discharging)',
            },
    );
    rails.push({ name: 'LP', role: 'supply', v_mv: 0, i_ua: 0, pads: ['2'], note: 'LP input' });
    rails.push({ name: 'SW', role: 'supply', v_mv: 0, i_ua: 0, pads: ['3'], note: 'SW input' });
    rails.push({
      name: 'V+',
      role: 'output',
      v_mv: vout,
      pads: ['10'],
      limit_ua: LDO_LIMIT_UA,
      note: ldoEn
        ? state.ldo_mode === LDO_MODE_LOAD_SWITCH
          ? 'load switch (ground via pad 1 switch)'
          : 'LDO (ground via pad 1 switch)'
        : 'LDO off — pad 1 ground also open',
    });

    let draw = 0;
    if (onVin) draw = IIN_QUIESCENT_UA + chargeUa;
    else if (batteryOn(state, i))
      draw = lowPower(i)
        ? state.ldo_enabled
          ? IBAT_LP_LDO_UA
          : IBAT_LP_UA
        : state.ldo_enabled
          ? IBAT_ACTIVE_LDO_UA
          : IBAT_ACTIVE_UA;
    else if (shipped && batteryPresent(i)) draw = IBAT_SHIP_UA;
    return { draw_ua: draw, rails };
  },
};

export default sim;
