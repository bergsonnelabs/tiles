// Digital twin for Power.L.1N — Nordic nPM1300 PMIC (battery charger + PMIC).
//
// A power *transform* tile: a USB charger input (CHG, pad 6) and a rechargeable
// battery (BATT, pad 7) feed three outputs — a normally-on 3.3 V buck (pad 9), a
// normally-on 1.8 V buck (pad 10), and the VSYS passthrough (pad 8, ≈ the higher
// of CHG/BATT). The two bucks come up at fixed voltages set by the tile's VSET
// pulldowns (VSET1 47 kΩ → 1.8 V, VSET2 330 kΩ → 3.3 V); firmware can override
// them. Three on-board indicator LEDs (LED0 red, LED1 orange, LED2 green) show
// error / charging / host status.
//
// Topology, pad map, voltages and charge ranges are datasheet- and
// tile-JSON-accurate (canonical). The current split between the CHG and BATT
// supplies — and back-filling the battery draw from downstream loads — is a
// solver/modeling choice (the actual downstream load lives on other tiles), so
// the power layer is marked "inferred".
import type { PowerCtx, TileSim } from '../tileSim';

const QUIESCENT_UA = 10; // nPM1300 system quiescent (bucks in PFM, light load)
const UVLO_MV = 2300; // below this on every input the PMIC can't hold its rails up
// A buck only steps DOWN: once VSYS falls within ~this of a buck's setpoint the
// regulator drops out (≈100 % duty) and the rail follows VSYS minus the pass-FET
// IR drop. The exact dropout is load-dependent; this fixed value is an inferred
// approximation — enough to model the rail sagging as the battery drains.
const BUCK_DROPOUT_MV = 100;
// Each buck is rated 200 mA (the tile definition's `max_current` for 3V3 and 1V8).
const BUCK_LIMIT_UA = 200_000;
// Buck efficiency (nPM1300 PS v1.1): 3.3 V ~94 % from 100 µA to 100 mA (Fig. 3,
// AUTO, VSYS 3.8 V; EFFBUCK 93 % typ at 200 mA); 1.8 V ~88 % in PWM (Fig. 24).
// Load-independent here: the light-load fall-off in PWM is not modelled.
const BUCK_EFF_3V3 = 0.94;
const BUCK_EFF_1V8 = 0.88;

interface State {
  // ── inputs (exercised via controls) ──
  usb_connected: number; // CHG present (USB plugged in)
  usb_mv: number; // CHG input voltage
  batt_mv: number; // battery terminal voltage
  battery_present: number; // a battery is attached
  fault: number; // charger fault (NTC/die/safety-timer)
  die_temp_c: number; // PMIC die temperature

  // ── charger config ──
  charging_enabled: number;
  charge_current_ma: number; // CC level (32-800)
  vbus_ilim_ma: number; // VBUS input current limit (100-1500); the chip boots at 100, init() sets 500
  term_mv: number; // termination (full) voltage

  // ── buck regulators (1 = VOUT1 1.8 V, 2 = VOUT2 3.3 V) ──
  buck1_en: number;
  buck2_en: number;
  buck1_mv: number;
  buck2_mv: number;

  // ── indicator LEDs (mode: 0 ERROR, 1 CHARGING, 2 HOST, 3 unused) ──
  led0_mode: number;
  led1_mode: number;
  led2_mode: number;
  led0_host: number; // host-commanded on/off (effective in HOST mode)
  led1_host: number;
  led2_host: number;

  [field: string]: number;
}

// charge-status bit masks (BCHGCHARGESTATUS)
const CHG_BATTERYDETECTED = 1 << 0;
const CHG_COMPLETED = 1 << 1;
const CHG_TRICKLE = 1 << 2;
const CHG_CC = 1 << 3;
const CHG_CV = 1 << 4;

// LED mode codes (LEDDRVxMODESEL)
const LED_ERROR = 0;
const LED_CHARGING = 1;
const LED_HOST = 2;

const pick = (args: number[], i: number, cur: number) =>
  args.length > i && Number.isFinite(args[i]) ? args[i] : cur;

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

// The PMIC's two inputs, resolved from the actual WIRING when available. CHG (pad
// 6) and BATT (pad 7) read their net voltage from the electrical ctx — so an
// external USB tile drives "is USB present" and a wired battery (behind a switch)
// drives "is a battery there", instead of internal toggles. Each input falls back
// to its control only when that pad isn't wired (a standalone PMIC). USB/battery
// "present" means the pad actually carries voltage — a switched-off battery or an
// unpowered USB reads absent, so the PMIC won't pretend to charge a phantom cell.
interface Inputs {
  chgV: number;
  battV: number;
  usbPresent: boolean;
  batteryPresent: boolean;
  vinMv: number;
  inputOk: boolean;
}
function resolveInputs(s: State, ctx?: PowerCtx): Inputs {
  const chgPad = ctx?.padVoltage?.['6'];
  const battPad = ctx?.padVoltage?.['7'];
  // Inside a solved system (`ctx` given) the WIRING is the truth: a pad with no
  // voltage on it has nothing connected — no USB, no battery. The internal toggles
  // only stand in when the twin is asked on its own, with no system around it.
  // (Falling back to them in a system invented a battery nobody had wired, and a
  // 100 mA charge into it.)
  const wired = ctx != null;
  const chgV = chgPad != null ? chgPad : wired ? 0 : s.usb_connected === 1 ? s.usb_mv : 0;
  const battV = battPad != null ? battPad : wired ? 0 : s.batt_mv;
  const usbPresent = chgV > UVLO_MV;
  const batteryPresent =
    battPad != null ? battV > UVLO_MV : wired ? false : s.battery_present === 1;
  const vinMv = Math.max(chgV, battV);
  return { chgV, battV, usbPresent, batteryPresent, vinMv, inputOk: vinMv > UVLO_MV };
}

// Is the charger actively pushing current into the battery? Needs USB power in,
// a real battery to charge, the charger enabled, no fault, and the cell below its
// termination voltage.
function charging(s: State, inp: Inputs): boolean {
  return (
    inp.usbPresent &&
    inp.batteryPresent &&
    s.charging_enabled === 1 &&
    s.fault !== 1 &&
    inp.battV < s.term_mv
  );
}

// Battery full (termination reached) while charging from USB.
function complete(s: State, inp: Inputs): boolean {
  return inp.usbPresent && inp.batteryPresent && inp.battV >= s.term_mv;
}

// Build the BCHGCHARGESTATUS byte from the modeled state.
function chargeStatus(s: State, inp: Inputs): number {
  let b = 0;
  // BATTERYDETECTED is a charger-domain bit: it reads 0 while charging is
  // disabled, cell fitted or not (tile_power_l_1n.h:303-310, bench-measured).
  if (inp.batteryPresent && s.charging_enabled === 1) b |= CHG_BATTERYDETECTED;
  if (complete(s, inp)) b |= CHG_COMPLETED;
  if (charging(s, inp)) {
    if (inp.battV < 3000)
      b |= CHG_TRICKLE; // pre-charge a depleted cell
    else if (inp.battV >= s.term_mv - 50)
      b |= CHG_CV; // constant-voltage taper
    else b |= CHG_CC; // bulk constant-current
  }
  return b;
}

// A 10-bit ADC reading as the driver converts it: raw = mV·1023/FS, then back
// with integer math (tile_power_l_1n.c:234-248). VBAT FS 5.0 V, VSYS 6.375 V.
function adcMv(mv: number, fullScaleMv: number): number {
  const raw = clamp(Math.round((mv * 1023) / fullScaleMv), 0, 1023);
  return Math.floor((raw * fullScaleMv) / 1023);
}

// BCHGVTERM quantization (vterm_code, tile_power_l_1n.c:62): 3.50-3.65 V and
// 4.00-4.45 V in 50 mV steps, rounded down; the 3.65-4.00 V gap reads 3.65 V.
function quantizeTermMv(mv: number): number {
  const v = clamp(mv, 3500, 4450);
  if (v < 4000) return 3500 + Math.floor((Math.min(v, 3650) - 3500) / 50) * 50;
  return 4000 + Math.floor((v - 4000) / 50) * 50;
}

// VSYS ≈ the higher input, gated on at least one input being live.
function vsys(inp: Inputs): number {
  return inp.inputOk ? clamp(inp.vinMv, 0, 5000) : 0;
}

// Resolve an indicator LED to on/off given its mode + the auto signals.
function ledOn(mode: number, host: number, isChg: boolean, isFault: boolean): number {
  if (mode === LED_HOST) return host ? 1 : 0;
  if (mode === LED_CHARGING) return isChg ? 1 : 0;
  if (mode === LED_ERROR) return isFault ? 1 : 0;
  return 0; // unused
}

const sim: TileSim<State> = {
  tile: 'Power.L.1N',

  defaultState: {
    usb_connected: 0,
    usb_mv: 5000,
    batt_mv: 3800,
    battery_present: 1,
    fault: 0,
    die_temp_c: 25,

    charging_enabled: 1,
    charge_current_ma: 100,
    // Like the charge settings beside it, this is the state AFTER the driver's init():
    // the chip powers up at 100 mA and init() raises it to 500 before it charges.
    vbus_ilim_ma: 500,
    term_mv: 4200,

    buck1_en: 1,
    buck2_en: 1,
    buck1_mv: 1800,
    buck2_mv: 3300,

    led0_mode: LED_ERROR,
    led1_mode: LED_CHARGING,
    led2_mode: LED_HOST,
    led0_host: 0,
    led1_host: 0,
    led2_host: 0,
  },

  // USB presence (CHG) and the battery now come from the WIRING — drive them with
  // the USB tile's "VBUS power" toggle and the battery part, not PMIC controls.
  // What's left are the PMIC's own config knobs.
  controls: [
    { type: 'toggle', field: 'charging_enabled', label: 'Charging enabled' },
    {
      type: 'slider',
      field: 'charge_current_ma',
      label: 'Charge current',
      min: 32,
      max: 800,
      step: 2,
      unit: 'mA',
    },
    { type: 'toggle', field: 'fault', label: 'Charger fault' },
    {
      type: 'slider',
      field: 'die_temp_c',
      label: 'Die temperature',
      min: -20,
      max: 125,
      step: 1,
      unit: '°C',
    },
    { type: 'toggle', field: 'led2_host', label: 'LED2 green (host)' },
  ],

  // The one thing a person makes HAPPEN to a charger: a fault (a shorted cell, a
  // safety timer). Its settings are the program's, not the simulator's.
  stimuli: [
    {
      id: 'fault',
      label: 'Charger fault',
      controls: [{ kind: 'toggle', id: 'fault', label: 'charger fault', fields: ['fault'] }],
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    // 1 if the PMIC ACKs at 0x6B (tile_power_l_1n.c:75)
    tile_power_l_1n_find: () => ({ scalar: 1 }),
    // init (tile_power_l_1n.c:82-144): LED modes, then 500 mA input limit,
    // 100 mA charge, 4.20 V termination, charging on.
    tile_power_l_1n_init: () => ({
      nextState: {
        led0_mode: LED_ERROR,
        led1_mode: LED_CHARGING,
        led2_mode: LED_HOST,
        vbus_ilim_ma: 500,
        charge_current_ma: 100,
        term_mv: 4200,
        charging_enabled: 1,
      },
    }),

    // ── charger ──
    tile_power_l_1n_charger_enable: ({ args }) => ({
      nextState: { charging_enabled: pick(args, 0, 1) ? 1 : 0 },
    }),
    // 100 mA steps, rounded DOWN, 100-1500 (VBUSINILIM0 code = mA / 100).
    tile_power_l_1n_set_vbus_limit_ma: ({ state, args }) => ({
      nextState: {
        vbus_ilim_ma: Math.floor(clamp(pick(args, 0, state.vbus_ilim_ma), 100, 1500) / 100) * 100,
      },
    }),
    tile_power_l_1n_set_charge_current_ma: ({ state, args }) => ({
      // 2 mA steps, rounded down (ISETMSB/LSB idx = mA / 2)
      nextState: {
        charge_current_ma:
          Math.floor(clamp(pick(args, 0, state.charge_current_ma), 32, 800) / 2) * 2,
      },
    }),
    tile_power_l_1n_set_term_mv: ({ state, args }) => ({
      nextState: { term_mv: quantizeTermMv(pick(args, 0, state.term_mv)) },
    }),
    tile_power_l_1n_get_charge_status: ({ state }) => ({
      scalar: chargeStatus(state, resolveInputs(state)),
    }),
    tile_power_l_1n_get_charge_error: ({ state }) => ({ scalar: state.fault ? 0x01 : 0 }),
    tile_power_l_1n_is_charging: ({ state }) => ({
      scalar: charging(state, resolveInputs(state)) ? 1 : 0,
    }),
    tile_power_l_1n_is_charge_complete: ({ state }) => ({
      scalar: complete(state, resolveInputs(state)) ? 1 : 0,
    }),
    tile_power_l_1n_battery_present: ({ state }) => ({
      scalar: resolveInputs(state).batteryPresent ? 1 : 0,
    }),

    // ── measurements ──
    tile_power_l_1n_get_vbat_mv: ({ state }) => ({
      scalar: adcMv(resolveInputs(state).battV, 5000),
    }),
    tile_power_l_1n_get_vsys_mv: ({ state }) => ({
      scalar: adcMv(vsys(resolveInputs(state)), 6375),
    }),
    tile_power_l_1n_get_die_temp_c: ({ state }) => ({ scalar: Math.trunc(state.die_temp_c) }),

    // ── buck regulators ──
    tile_power_l_1n_buck_enable: ({ args }) => {
      const buck = pick(args, 0, 1);
      const on = pick(args, 1, 1) ? 1 : 0;
      if (buck === 2) return { nextState: { buck2_en: on } };
      if (buck === 1) return { nextState: { buck1_en: on } };
      return { nextState: {} };
    },
    tile_power_l_1n_buck_set_mv: ({ args }) => {
      const buck = pick(args, 0, 1);
      // 1.0-3.3 V in 100 mV steps, rounded down (NORMVOUT code = (mv-1000)/100)
      const mv = 1000 + Math.floor((clamp(pick(args, 1, 1800), 1000, 3300) - 1000) / 100) * 100;
      if (buck === 2) return { nextState: { buck2_mv: mv } };
      if (buck === 1) return { nextState: { buck1_mv: mv } };
      return { nextState: {} };
    },

    // ── indicator LEDs ──
    tile_power_l_1n_led_set_mode: ({ args }) => {
      const led = pick(args, 0, 0);
      const mode = pick(args, 1, LED_HOST);
      if (led === 0) return { nextState: { led0_mode: mode } };
      if (led === 1) return { nextState: { led1_mode: mode } };
      if (led === 2) return { nextState: { led2_mode: mode } };
      return { nextState: {} };
    },
    tile_power_l_1n_led_set: ({ args }) => {
      const led = pick(args, 0, 0);
      const on = pick(args, 1, 0) ? 1 : 0;
      if (led === 0) return { nextState: { led0_host: on } };
      if (led === 1) return { nextState: { led1_host: on } };
      if (led === 2) return { nextState: { led2_host: on } };
      return { nextState: {} };
    },

    // ── misc ──
    tile_power_l_1n_get_reset_cause: () => ({ scalar: 0 }),
  },

  provenance: {
    // canonical — datasheet- / tile-JSON-accurate value or bit layout
    tile_power_l_1n_find: 'canonical', // I2C 0x6B
    tile_power_l_1n_get_charge_status: 'canonical', // BCHGCHARGESTATUS bit layout
    tile_power_l_1n_is_charging: 'canonical',
    tile_power_l_1n_is_charge_complete: 'canonical',
    tile_power_l_1n_battery_present: 'canonical',
    tile_power_l_1n_get_vbat_mv: 'canonical', // VBAT mV (FS 5.0 V)
    tile_power_l_1n_get_vsys_mv: 'canonical', // VSYS = higher of inputs
    tile_power_l_1n_get_die_temp_c: 'canonical', // die-temp transfer
    tile_power_l_1n_set_charge_current_ma: 'canonical', // 32-800 mA range
    tile_power_l_1n_set_vbus_limit_ma: 'canonical', // VBUSINILIM0: mA/100, boots at 100 mA
    tile_power_l_1n_set_term_mv: 'canonical', // 3.5-4.45 V range
    tile_power_l_1n_buck_set_mv: 'canonical', // 1.0-3.3 V range
    tile_power_l_1n_led_set_mode: 'canonical', // LEDDRVxMODESEL codes
    // inferred — behavior follows obviously from the driver but isn't a datasheet number
    tile_power_l_1n_charger_enable: 'inferred',
    tile_power_l_1n_buck_enable: 'inferred',
    tile_power_l_1n_led_set: 'inferred',
    tile_power_l_1n_get_charge_error: 'inferred', // fault → nonzero code (reason bits not modeled)
    // hallucinated — not really modeled
    tile_power_l_1n_get_reset_cause: 'hallucinated', // always reports 0
    power: 'inferred', // voltages/topology canonical; current split is a modeling choice
  },

  // On-board status LEDs (LED0 red / error, LED1 orange / charging, LED2 green /
  // host; nPM1300 balls A1-A3) — declared as indicators so the canvas renders
  // them generically.
  indicators(state, ctx) {
    const isChg = charging(state, resolveInputs(state, ctx));
    const isFault = state.fault === 1;
    return [
      {
        id: 'led0',
        label: 'Error (red)',
        color: '#ef4444',
        level: ledOn(state.led0_mode, state.led0_host, isChg, isFault),
      },
      {
        id: 'led1',
        label: 'Charging (orange)',
        color: '#f97316',
        level: ledOn(state.led1_mode, state.led1_host, isChg, isFault),
      },
      {
        id: 'led2',
        label: 'Host (green)',
        color: '#22c55e',
        level: ledOn(state.led2_mode, state.led2_host, isChg, isFault),
      },
    ];
  },

  // Electrical layer. Inputs CHG (pad 6) / BATT (pad 7); outputs VSYS (pad 8),
  // 3V3 buck (pad 9), 1V8 buck (pad 10).
  //
  // Inputs are read from the wired net (PowerCtx) so the upstream circuit — a
  // battery behind a switch, a USB-C jack — actually drives the PMIC. An input
  // that's wired but undriven reads 0 (the supply is cut); an input that ISN'T
  // wired falls back to its own control (a standalone PMIC modelling its own
  // cell / USB). With no input above UVLO the bucks and VSYS collapse — exactly
  // what makes flipping the supply switch power the Core down.
  //
  // On USB the charge current + system quiescent are drawn from CHG and the
  // battery receives charge; on battery the BATT supply omits i_ua so the solver
  // back-fills it from the downstream loads on the output nets.
  power(state, ctx) {
    const inp = resolveInputs(state, ctx);
    const { chgV, battV, usbPresent, inputOk, vinMv } = inp;
    const isChg = charging(state, inp);
    const chargeUa = isChg ? state.charge_current_ma * 1000 : 0;
    const ownUa = inputOk ? chargeUa + QUIESCENT_UA : 0;
    const rails = [] as NonNullable<ReturnType<NonNullable<TileSim<State>['power']>>['rails']>;

    // CHG (pad 6) and BATT (pad 7) are ALWAYS declared — even when absent — so the
    // solver always tracks their pads and feeds back their wired voltage on the
    // next pass. (Conditionally emitting CHG hid pad 6 from the ctx probe, so the
    // PMIC never noticed a wired USB.) When VBUS is present it sources the system +
    // charge current and the battery receives charge; otherwise the battery
    // sources (no i_ua → solver back-fills it from the output loads).
    // With VBUS present its current is left UNDECLARED: the solver then makes it
    // what the outputs actually deliver (power-conserved), plus this chip's own
    // draw — quiescent and any charge current — which rides along via `draw_ua`.
    // (Declaring it as just `ownUa` hid the whole system load from the USB source.)
    rails!.push({
      name: 'CHG',
      role: 'supply',
      v_mv: chgV,
      ...(usbPresent ? {} : { i_ua: 0 }),
      pads: ['6'],
      // the VBUS input limiter (500 mA once the driver's init() has run)
      limit_ua: state.vbus_ilim_ma * 1000,
      note: usbPresent ? (isChg ? 'USB input — charging + system' : 'USB input') : 'USB absent',
    });
    rails!.push(
      usbPresent
        ? {
            name: 'BATT',
            role: 'supply',
            v_mv: battV,
            i_ua: 0,
            pads: ['7'],
            note: isChg ? 'battery receiving charge' : 'battery idle',
          }
        : {
            name: 'BATT',
            role: 'supply',
            v_mv: battV,
            pads: ['7'],
            note: 'battery input (discharging)',
          },
    );

    // A buck can't boost: its rail holds the setpoint only while VSYS is at least
    // a dropout above it; once VSYS sags toward the setpoint the rail follows
    // VSYS down (minus the pass-FET drop). So a draining battery pulls the 3V3 /
    // 1V8 rails below their nominal — they don't stay magically pegged.
    const buckOut = (en: number, setMv: number) =>
      inputOk && en ? Math.min(setMv, Math.max(0, vinMv - BUCK_DROPOUT_MV)) : 0;
    // VSYS passes the input's current straight through (the power path is a
    // switch, not a converter), and both bucks run from it: the solver then
    // charges USB / the battery for VSYS loads 1:1 and for each buck's load at
    // its efficiency (datasheet EFFBUCK / Fig. 3 / Fig. 24).
    rails!.push({
      name: 'VSYS',
      role: 'output',
      v_mv: inputOk ? vinMv : 0,
      pads: ['8'],
      conversion: 'linear',
      note: 'system (≈ higher of CHG/BATT)',
    });
    rails!.push({
      name: '3V3',
      role: 'output',
      v_mv: buckOut(state.buck2_en, state.buck2_mv),
      pads: ['9'],
      limit_ua: BUCK_LIMIT_UA,
      from: 'VSYS',
      conversion: 'switching',
      efficiency: BUCK_EFF_3V3,
      note: 'buck2 (3.3 V; follows VSYS down in dropout)',
    });
    rails!.push({
      name: '1V8',
      role: 'output',
      v_mv: buckOut(state.buck1_en, state.buck1_mv),
      pads: ['10'],
      limit_ua: BUCK_LIMIT_UA,
      from: 'VSYS',
      conversion: 'switching',
      efficiency: BUCK_EFF_1V8,
      note: 'buck1 (1.8 V; follows VSYS down in dropout)',
    });

    return { draw_ua: ownUa, rails };
  },
};

export default sim;
