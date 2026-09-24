// src/sims/power_l_1t.ts
var DEVICE_ID = 32;
var VUVLO_MV = 3400;
var VOVP_MV = 5500;
var VSLP_MV = 130;
var VLOWV_MV = 3e3;
var BATT_DETECT_MV = 2e3;
var LP_VIH_MV = 1350;
var SW_VIL_MV = 450;
var IIN_QUIESCENT_UA = 780;
var IBAT_SHIP_UA = 0.01;
var IBAT_LP_UA = 0.46;
var IBAT_LP_LDO_UA = 1.7;
var IBAT_ACTIVE_UA = 18;
var IBAT_ACTIVE_LDO_UA = 21;
var LDO_LIMIT_UA = 1e4;
var TS_HOT_MV = 185;
var TS_COLD_MV = 585;
var CS_IDLE = 0;
var CS_PRE_CHARGE = 1;
var CS_FAST_CHARGE = 2;
var CS_DONE = 3;
var LDO_MODE_LOAD_SWITCH = 1;
var PMID_AUTO = 0;
var PMID_BAT_ONLY = 1;
var num = (args, i, dflt) => args.length > i && Number.isFinite(args[i]) ? Math.trunc(args[i]) : dflt;
var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
var TS_CURVE = [
  [0, TS_COLD_MV],
  [10, 514],
  [45, 265],
  [60, TS_HOT_MV]
];
function tsMv(tc) {
  const c = TS_CURVE;
  let i = 0;
  while (i < c.length - 2 && tc > c[i + 1][0]) i++;
  const [t0, v0] = c[i];
  const [t1, v1] = c[i + 1];
  return clamp(Math.round(v0 + (tc - t0) * (v1 - v0) / (t1 - t0)), 0, 1200);
}
function resolveInputs(s, ctx) {
  const pv = ctx?.padVoltage;
  const wired = ctx != null;
  const vinPad = pv?.["8"];
  const battPad = pv?.["7"];
  const lpPad = pv?.["2"];
  const swPad = pv?.["3"];
  return {
    vinV: vinPad != null ? vinPad : wired ? 0 : s.vin_present ? s.vin_mv : 0,
    battV: battPad != null ? battPad : wired ? 0 : s.battery_present ? s.vbat_mv : 0,
    // LP has a 900 kΩ pull-down: an unwired LP pad in a system is LOW.
    lpLow: lpPad != null ? lpPad < LP_VIH_MV : wired ? true : s.lp_mode === 1,
    swGrounded: swPad != null ? swPad < SW_VIL_MV : wired ? false : s.sw_grounded === 1
  };
}
var vinValid = (i) => i.vinV >= VUVLO_MV && i.vinV <= VOVP_MV;
var vinPgood = (i) => vinValid(i) && i.vinV > i.battV + VSLP_MV;
var batteryPresent = (i) => i.battV >= BATT_DETECT_MV;
var inShip = (s, i) => !vinValid(i) && (s.shipped === 1 || s.ship_mode === 1);
var batteryOn = (s, i) => !inShip(s, i) && i.battV >= s.batt_uvlo_mv;
var poweredUp = (s, i) => vinValid(i) || batteryOn(s, i);
var lowPower = (i) => i.lpLow && !vinValid(i);
var i2cAlive = (s, i) => poweredUp(s, i) && !lowPower(i);
var tsFault = (s) => {
  if (!s.ts_enabled) return false;
  const ts = tsMv(s.temp_c);
  return ts <= TS_HOT_MV || ts >= TS_COLD_MV;
};
function chargeState(s, i) {
  if (!vinPgood(i) || !batteryPresent(i) || !s.charging_enabled || tsFault(s)) return CS_IDLE;
  if (i.battV >= s.charge_voltage_mv) return s.termination_pct > 0 ? CS_DONE : CS_IDLE;
  return i.battV < VLOWV_MV ? CS_PRE_CHARGE : CS_FAST_CHARGE;
}
function chargeMa(s, i) {
  const cs = chargeState(s, i);
  const target = cs === CS_PRE_CHARGE ? s.pre_charge_cma / 100 : cs === CS_FAST_CHARGE ? s.charge_current_ma : 0;
  return Math.min(target, s.input_current_limit_ma);
}
function pmidMv(s, i) {
  if (s.pmid_mode === PMID_AUTO) return vinValid(i) ? i.vinV : batteryOn(s, i) ? i.battV : 0;
  if (s.pmid_mode === PMID_BAT_ONLY) return batteryOn(s, i) ? i.battV : 0;
  return 0;
}
var ldoOn = (s, i) => s.ldo_enabled === 1 && !i.swGrounded && poweredUp(s, i) && pmidMv(s, i) > 0;
var ldoOutMv = (s, i) => !ldoOn(s, i) ? 0 : s.ldo_mode === LDO_MODE_LOAD_SWITCH ? pmidMv(s, i) : Math.min(s.ldo_voltage_mv, pmidMv(s, i));
function percentOf(mv) {
  if (mv <= 3e3) return 0;
  if (mv >= 4200) return 100;
  return Math.floor((mv - 3e3) * 100 / 1200);
}
function stat0(s, i) {
  const cs = chargeState(s, i);
  let b = 0;
  if (vinPgood(i)) b |= 1;
  if (cs === CS_DONE) b |= 32;
  if (cs === CS_FAST_CHARGE && i.battV >= s.charge_voltage_mv - 100) b |= 64;
  return b;
}
function stat1(s, i) {
  const ts = tsMv(s.temp_c);
  let b = 0;
  if (i.vinV > VOVP_MV) b |= 128;
  if (batteryPresent(i) && i.battV < s.batt_uvlo_mv) b |= 16;
  if (s.ts_enabled) {
    if (ts >= TS_COLD_MV) b |= 8;
    else if (ts >= 514) b |= 4;
    if (ts <= TS_HOT_MV) b |= 1;
    else if (ts <= 265) b |= 2;
  }
  return b;
}
function driverCharging(s, i) {
  const s0 = stat0(s, i);
  const s1 = stat1(s, i);
  return (s0 & 1) !== 0 && (s0 & 32) === 0 && (s1 & 153) === 0;
}
function quantizeIchg(ma) {
  const m = Math.max(0, Math.min(500, ma));
  if (m > 318) return Math.floor(Math.min(255, Math.floor(m * 2 / 5)) * 5 / 2);
  return Math.floor(Math.min(255, Math.floor(m * 4 / 5)) * 5 / 4);
}
var sim = {
  tile: "Power.L.1T",
  // After tile_power_l_1t_init(): ICHG 80 mA, IPRECHG 18.75 mA, BUVLO 2.6 V,
  // watchdog off, ship mode cleared; everything else at its register reset
  // (VBATREG 4.2 V, ITERM 10 %, ILIM 100 mA, LDO on at 1.8 V, TS on, 6 h timer).
  defaultState: {
    vbat_mv: 3800,
    battery_present: 1,
    vin_mv: 5e3,
    vin_present: 0,
    temp_c: 25,
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
    ship_mode: 0
  },
  // Physical stimuli. In a wired system the SUPPLY+ / BATT+ / LP / SW pads come
  // from the wiring; these stand in for a tile on its own.
  controls: [
    {
      type: "slider",
      field: "vbat_mv",
      label: "Battery voltage",
      min: 2400,
      max: 4500,
      step: 10,
      unit: "mV",
      description: "Cell voltage \u2014 get_vbat_mv reads it; get_percent maps 3.0\u20134.2 V to 0\u2013100 %."
    },
    { type: "toggle", field: "battery_present", label: "Battery attached" },
    { type: "toggle", field: "vin_present", label: "VIN plugged in" },
    {
      type: "slider",
      field: "vin_mv",
      label: "VIN voltage",
      min: 3e3,
      max: 6e3,
      step: 50,
      unit: "mV",
      description: "Adapter voltage on SUPPLY+ (pad 8). Valid 3.4\u20135.5 V; above 5.5 V is OVP."
    },
    {
      type: "slider",
      field: "temp_c",
      label: "Battery temperature",
      min: -20,
      max: 70,
      step: 1,
      unit: "\xB0C",
      description: "NTC temperature \u2192 get_ts_mv. With TS on, below 0 \xB0C or above 60 \xB0C pauses charging."
    },
    {
      type: "toggle",
      field: "lp_mode",
      label: "LP pad low",
      description: "On battery, a low LP pad puts the chip in low-power mode (I\xB2C and ADC off)."
    },
    { type: "toggle", field: "sw_grounded", label: "SW tied to GND (LDO off)" }
  ],
  hostCalls: {
    // ── lifecycle ──
    tile_power_l_1t_find: () => ({ scalar: 1 }),
    tile_power_l_1t_init: () => ({ scalar: 0 }),
    // ── charge config ──
    tile_power_l_1t_set_charge_current_ma: ({ args }) => ({
      nextState: { charge_current_ma: quantizeIchg(num(args, 0, 80)) }
    }),
    tile_power_l_1t_set_charge_voltage_mv: ({ args }) => {
      const mv = clamp(num(args, 0, 4200), 3600, 4600);
      return { nextState: { charge_voltage_mv: 3600 + Math.floor((mv - 3600) / 10) * 10 } };
    },
    tile_power_l_1t_set_pre_charge_ma: ({ state, args }) => {
      const range1 = state.charge_current_ma > 318;
      const ma = Math.max(0, num(args, 0, 18) & 255);
      const code = range1 ? Math.min(31, Math.floor(Math.min(ma, 77) * 2 / 5)) : Math.min(31, Math.floor(Math.min(ma, 38) * 4 / 5));
      return { nextState: { pre_charge_cma: code * (range1 ? 250 : 125) } };
    },
    tile_power_l_1t_set_termination_percent: ({ args }) => ({
      nextState: { termination_pct: Math.min(31, Math.max(0, num(args, 0, 10) & 255)) }
    }),
    tile_power_l_1t_set_input_current_limit_ma: ({ args }) => {
      const ma = num(args, 0, 100);
      const levels = [600, 500, 400, 300, 200, 150, 100];
      return { nextState: { input_current_limit_ma: levels.find((l) => ma >= l) ?? 50 } };
    },
    tile_power_l_1t_charger_enable: ({ args }) => ({
      nextState: { charging_enabled: num(args, 0, 1) ? 1 : 0 }
    }),
    tile_power_l_1t_set_battery_uvlo_mv: ({ args }) => {
      const mv = num(args, 0, 3e3);
      const levels = [3e3, 2800, 2600, 2400];
      return { nextState: { batt_uvlo_mv: levels.find((l) => mv >= l) ?? 2200 } };
    },
    tile_power_l_1t_set_safety_timer: ({ args }) => ({
      nextState: { safety_timer: num(args, 0, 1) & 3 }
    }),
    tile_power_l_1t_set_pmid_mode: ({ args }) => ({
      nextState: { pmid_mode: num(args, 0, 0) & 3 }
    }),
    // ── ADC reads (0 when the chip isn't answering: ship, LP, unpowered) ──
    tile_power_l_1t_get_vbat_mv: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) ? i.battV : 0 };
    },
    tile_power_l_1t_get_vin_mv: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) ? Math.min(i.vinV, 6e3) : 0 };
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
      const i = resolveInputs(state);
      return {
        scalar: i2cAlive(state, i) && vinValid(i) ? Math.floor(chargeMa(state, i) + IIN_QUIESCENT_UA / 1e3) : 0
      };
    },
    tile_power_l_1t_get_ts_mv: ({ state }) => {
      const i = resolveInputs(state);
      return { scalar: i2cAlive(state, i) ? tsMv(state.temp_c) : 0 };
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
      nextState: { ts_cold_code: num(args, 0, 0) & 255 }
    }),
    tile_power_l_1t_set_ts_cool: ({ args }) => ({
      nextState: { ts_cool_code: num(args, 0, 0) & 255 }
    }),
    tile_power_l_1t_set_ts_warm: ({ args }) => ({
      nextState: { ts_warm_code: num(args, 0, 0) & 255 }
    }),
    tile_power_l_1t_set_ts_hot: ({ args }) => ({
      nextState: { ts_hot_code: num(args, 0, 0) & 255 }
    }),
    tile_power_l_1t_set_ts_enabled: ({ args }) => ({
      nextState: { ts_enabled: num(args, 0, 1) ? 1 : 0 }
    }),
    // ── LDO ──
    tile_power_l_1t_set_ldo_voltage_mv: ({ args }) => {
      const mv = clamp(num(args, 0, 1800), 600, 3700);
      return { nextState: { ldo_voltage_mv: 600 + Math.floor((mv - 600) / 100) * 100 } };
    },
    tile_power_l_1t_set_ldo_mode: ({ args }) => ({
      nextState: { ldo_mode: num(args, 0, 0) === LDO_MODE_LOAD_SWITCH ? 1 : 0 }
    }),
    tile_power_l_1t_set_ldo_enabled: ({ args }) => ({
      nextState: { ldo_enabled: num(args, 0, 1) ? 1 : 0 }
    }),
    // ── status ──
    // Fills a power_l_1t_status_t the program can't read back through the DSL.
    tile_power_l_1t_get_charge_status: () => ({}),
    tile_power_l_1t_read_status: ({ state, args }) => {
      const i = resolveInputs(state);
      if (!i2cAlive(state, i)) return { scalar: 0 };
      const r1 = state.charge_current_ma > 318;
      const regs = {
        0: stat0(state, i),
        1: stat1(state, i),
        18: Math.floor((state.charge_voltage_mv - 3600) / 10) & 127,
        19: Math.floor(state.charge_current_ma * (r1 ? 2 : 4) / 5) & 255,
        20: (r1 ? 128 : 0) | Math.floor(state.pre_charge_cma / (r1 ? 250 : 125)) & 31,
        21: state.termination_pct === 0 ? 1 : (state.termination_pct & 31) << 1,
        25: [50, 100, 150, 200, 300, 400, 500, 600].indexOf(state.input_current_limit_ma) & 7,
        29: (state.ldo_enabled ? 128 : 0) | (Math.floor((state.ldo_voltage_mv - 600) / 100) & 31) << 2 | (state.ldo_mode ? 2 : 0),
        53: (state.ship_mode ? 128 : 0) | 16,
        111: DEVICE_ID
      };
      return { scalar: regs[num(args, 0, 0) & 255] ?? 0 };
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
      return { scalar: pct < (num(args, 0, 0) & 255) ? 1 : 0 };
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
      const ch = num(args, 1, 0) & 7;
      if (comp === 1) return { nextState: { adc_comp1_ch: ch } };
      if (comp === 2) return { nextState: { adc_comp2_ch: ch } };
      if (comp === 3) return { nextState: { adc_comp3_ch: ch } };
      return {};
    },
    // Threshold crossings aren't modeled: no alarm flags (FLAG2[6:4] = 0).
    tile_power_l_1t_get_adc_comparators: () => ({ scalar: 0 })
  },
  provenance: {
    tile_power_l_1t_find: "canonical",
    // fixed 0x6B
    tile_power_l_1t_init: "canonical",
    // defaultState = post-init registers
    tile_power_l_1t_set_charge_current_ma: "canonical",
    // 1.25 / 2.5 mA steps, 500 mA cap
    tile_power_l_1t_set_charge_voltage_mv: "canonical",
    // 3.6 V + 10 mV·code
    tile_power_l_1t_set_pre_charge_ma: "canonical",
    // IPRECHG steps follow ICHARGE_RANGE
    tile_power_l_1t_set_termination_percent: "canonical",
    tile_power_l_1t_set_input_current_limit_ma: "canonical",
    // ILIMCTRL table, floor
    tile_power_l_1t_charger_enable: "canonical",
    tile_power_l_1t_set_battery_uvlo_mv: "canonical",
    // BUVLO table
    tile_power_l_1t_set_safety_timer: "inferred",
    // stored; timer expiry not modeled
    tile_power_l_1t_set_pmid_mode: "canonical",
    tile_power_l_1t_get_vbat_mv: "canonical",
    tile_power_l_1t_get_vin_mv: "canonical",
    tile_power_l_1t_get_pmid_mv: "inferred",
    // PMID ≈ VIN / VBAT (FET drop ignored)
    tile_power_l_1t_get_charge_current_ma: "inferred",
    // CC step, no CV taper
    tile_power_l_1t_get_input_current_ma: "inferred",
    // no system load in the worker
    tile_power_l_1t_get_ts_mv: "inferred",
    // thresholds canonical; NTC network assumed
    tile_power_l_1t_get_adcin_mv: "hallucinated",
    // ADCIN isn't routed on this tile
    tile_power_l_1t_get_percent: "canonical",
    // driver's linear 3.0–4.2 V
    tile_power_l_1t_set_ts_cold: "inferred",
    // codes stored; thresholds stay default
    tile_power_l_1t_set_ts_cool: "inferred",
    tile_power_l_1t_set_ts_warm: "inferred",
    tile_power_l_1t_set_ts_hot: "inferred",
    tile_power_l_1t_set_ts_enabled: "canonical",
    tile_power_l_1t_set_ldo_voltage_mv: "canonical",
    // 600 mV + 100 mV·code
    tile_power_l_1t_set_ldo_mode: "inferred",
    // load-switch input assumed = PMID
    tile_power_l_1t_set_ldo_enabled: "canonical",
    tile_power_l_1t_get_charge_status: "inferred",
    // struct not readable from the DSL
    tile_power_l_1t_read_status: "inferred",
    // STAT0/1, config regs, DEVICE_ID; rest 0
    tile_power_l_1t_write_reg: "hallucinated",
    // raw writes not modeled
    tile_power_l_1t_is_charging: "canonical",
    // the driver's STAT0/STAT1 derivation
    tile_power_l_1t_is_charge_done: "canonical",
    tile_power_l_1t_is_battery_low: "canonical",
    tile_power_l_1t_is_powered: "canonical",
    // VIN_PGOOD
    tile_power_l_1t_wait_for_charge_done: "inferred",
    tile_power_l_1t_enter_ship_mode: "canonical",
    // arms; enters on VIN removal
    tile_power_l_1t_set_adc_comparator: "inferred",
    tile_power_l_1t_get_adc_comparators: "inferred",
    power: "inferred"
    // quiescents canonical; charge/LDO currents and VINLS=PMID modeled
  },
  // Each tick: latch/unlatch ship mode and publish the charge state, from the
  // stimuli (the worker has no wiring to read).
  deriveState(state) {
    const i = resolveInputs(state);
    const update = {};
    if (vinValid(i)) {
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
    const on = ldoOn(state, resolveInputs(state)) ? 1 : 0;
    return { "10": on, "1": on };
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
    const chargeUa = Math.round(chargeMa(state, i) * 1e3);
    const ldoEn = ldoOn(state, i);
    const vout = ldoOutMv(state, i);
    const rails = [];
    rails.push({
      name: "SUPPLY",
      role: "supply",
      v_mv: i.vinV,
      ...onVin ? {} : { i_ua: 0 },
      pads: ["8"],
      limit_ua: state.input_current_limit_ma * 1e3,
      note: onVin ? chargeUa > 0 ? "input \u2014 charging + system" : "input" : i.vinV > VOVP_MV ? "input over-voltage (OVP)" : "no input"
    });
    rails.push(
      onVin ? {
        name: "BATT",
        role: "supply",
        v_mv: i.battV,
        i_ua: 0,
        pads: ["7"],
        note: chargeUa > 0 ? "battery receiving charge" : "battery idle"
      } : shipped || !batteryOn(state, i) ? {
        name: "BATT",
        role: "supply",
        v_mv: i.battV,
        i_ua: shipped && batteryPresent(i) ? IBAT_SHIP_UA : 0,
        pads: ["7"],
        note: shipped ? "ship mode \u2014 battery disconnected" : "battery below UVLO"
      } : {
        name: "BATT",
        role: "supply",
        v_mv: i.battV,
        pads: ["7"],
        note: "battery input (discharging)"
      }
    );
    rails.push({ name: "LP", role: "supply", v_mv: 0, i_ua: 0, pads: ["2"], note: "LP input" });
    rails.push({ name: "SW", role: "supply", v_mv: 0, i_ua: 0, pads: ["3"], note: "SW input" });
    rails.push({
      name: "V+",
      role: "output",
      v_mv: vout,
      pads: ["10"],
      limit_ua: LDO_LIMIT_UA,
      note: ldoEn ? state.ldo_mode === LDO_MODE_LOAD_SWITCH ? "load switch (ground via pad 1 switch)" : "LDO (ground via pad 1 switch)" : "LDO off \u2014 pad 1 ground also open"
    });
    let draw = 0;
    if (onVin) draw = IIN_QUIESCENT_UA + chargeUa;
    else if (batteryOn(state, i))
      draw = lowPower(i) ? state.ldo_enabled ? IBAT_LP_LDO_UA : IBAT_LP_UA : state.ldo_enabled ? IBAT_ACTIVE_LDO_UA : IBAT_ACTIVE_UA;
    else if (shipped && batteryPresent(i)) draw = IBAT_SHIP_UA;
    return { draw_ua: draw, rails };
  }
};
var power_l_1t_default = sim;
export {
  power_l_1t_default as default
};
