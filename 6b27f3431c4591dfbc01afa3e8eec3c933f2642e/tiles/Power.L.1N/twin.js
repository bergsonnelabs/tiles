// src/sims/power_l_1n.ts
var QUIESCENT_UA = 10;
var UVLO_MV = 2300;
var BUCK_DROPOUT_MV = 100;
var BUCK_LIMIT_UA = 2e5;
var CHG_BATTERYDETECTED = 1 << 0;
var CHG_COMPLETED = 1 << 1;
var CHG_TRICKLE = 1 << 2;
var CHG_CC = 1 << 3;
var CHG_CV = 1 << 4;
var LED_ERROR = 0;
var LED_CHARGING = 1;
var LED_HOST = 2;
var pick = (args, i, cur) => args.length > i && Number.isFinite(args[i]) ? args[i] : cur;
var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
function resolveInputs(s, ctx) {
  const chgPad = ctx?.padVoltage?.["6"];
  const battPad = ctx?.padVoltage?.["7"];
  const wired = ctx != null;
  const chgV = chgPad != null ? chgPad : wired ? 0 : s.usb_connected === 1 ? s.usb_mv : 0;
  const battV = battPad != null ? battPad : wired ? 0 : s.batt_mv;
  const usbPresent = chgV > UVLO_MV;
  const batteryPresent = battPad != null ? battV > UVLO_MV : wired ? false : s.battery_present === 1;
  const vinMv = Math.max(chgV, battV);
  return { chgV, battV, usbPresent, batteryPresent, vinMv, inputOk: vinMv > UVLO_MV };
}
function charging(s, inp) {
  return inp.usbPresent && inp.batteryPresent && s.charging_enabled === 1 && s.fault !== 1 && inp.battV < s.term_mv;
}
function complete(s, inp) {
  return inp.usbPresent && inp.batteryPresent && inp.battV >= s.term_mv;
}
function chargeStatus(s, inp) {
  let b = 0;
  if (inp.batteryPresent && s.charging_enabled === 1) b |= CHG_BATTERYDETECTED;
  if (complete(s, inp)) b |= CHG_COMPLETED;
  if (charging(s, inp)) {
    if (inp.battV < 3e3)
      b |= CHG_TRICKLE;
    else if (inp.battV >= s.term_mv - 50)
      b |= CHG_CV;
    else b |= CHG_CC;
  }
  return b;
}
function adcMv(mv, fullScaleMv) {
  const raw = clamp(Math.round(mv * 1023 / fullScaleMv), 0, 1023);
  return Math.floor(raw * fullScaleMv / 1023);
}
function quantizeTermMv(mv) {
  const v = clamp(mv, 3500, 4450);
  if (v < 4e3) return 3500 + Math.floor((Math.min(v, 3650) - 3500) / 50) * 50;
  return 4e3 + Math.floor((v - 4e3) / 50) * 50;
}
function vsys(inp) {
  return inp.inputOk ? clamp(inp.vinMv, 0, 5e3) : 0;
}
function ledOn(mode, host, isChg, isFault) {
  if (mode === LED_HOST) return host ? 1 : 0;
  if (mode === LED_CHARGING) return isChg ? 1 : 0;
  if (mode === LED_ERROR) return isFault ? 1 : 0;
  return 0;
}
var sim = {
  tile: "Power.L.1N",
  defaultState: {
    usb_connected: 0,
    usb_mv: 5e3,
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
    led0_mode: LED_HOST,
    led1_mode: LED_CHARGING,
    led2_mode: LED_ERROR,
    led0_host: 0,
    led1_host: 0,
    led2_host: 0
  },
  // USB presence (CHG) and the battery now come from the WIRING — drive them with
  // the USB tile's "VBUS power" toggle and the battery part, not PMIC controls.
  // What's left are the PMIC's own config knobs.
  controls: [
    { type: "toggle", field: "charging_enabled", label: "Charging enabled" },
    {
      type: "slider",
      field: "charge_current_ma",
      label: "Charge current",
      min: 32,
      max: 800,
      step: 2,
      unit: "mA"
    },
    { type: "toggle", field: "fault", label: "Charger fault" },
    {
      type: "slider",
      field: "die_temp_c",
      label: "Die temperature",
      min: -20,
      max: 125,
      step: 1,
      unit: "\xB0C"
    },
    { type: "toggle", field: "led0_host", label: "LED0 green (host)" }
  ],
  // The one thing a person makes HAPPEN to a charger: a fault (a shorted cell, a
  // safety timer). Its settings are the program's, not the simulator's.
  stimuli: [
    {
      id: "fault",
      label: "Charger fault",
      controls: [{ kind: "toggle", id: "fault", label: "charger fault", fields: ["fault"] }]
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    // 1 if the PMIC ACKs at 0x6B (tile_power_l_1n.c:75)
    tile_power_l_1n_find: () => ({ scalar: 1 }),
    // init (tile_power_l_1n.c:82-144): LED modes, then 500 mA input limit,
    // 100 mA charge, 4.20 V termination, charging on.
    tile_power_l_1n_init: () => ({
      nextState: {
        led0_mode: LED_HOST,
        led1_mode: LED_CHARGING,
        led2_mode: LED_ERROR,
        vbus_ilim_ma: 500,
        charge_current_ma: 100,
        term_mv: 4200,
        charging_enabled: 1
      }
    }),
    // ── charger ──
    tile_power_l_1n_charger_enable: ({ args }) => ({
      nextState: { charging_enabled: pick(args, 0, 1) ? 1 : 0 }
    }),
    // 100 mA steps, rounded DOWN, 100-1500 (VBUSINILIM0 code = mA / 100).
    tile_power_l_1n_set_vbus_limit_ma: ({ state, args }) => ({
      nextState: {
        vbus_ilim_ma: Math.floor(clamp(pick(args, 0, state.vbus_ilim_ma), 100, 1500) / 100) * 100
      }
    }),
    tile_power_l_1n_set_charge_current_ma: ({ state, args }) => ({
      // 2 mA steps, rounded down (ISETMSB/LSB idx = mA / 2)
      nextState: {
        charge_current_ma: Math.floor(clamp(pick(args, 0, state.charge_current_ma), 32, 800) / 2) * 2
      }
    }),
    tile_power_l_1n_set_term_mv: ({ state, args }) => ({
      nextState: { term_mv: quantizeTermMv(pick(args, 0, state.term_mv)) }
    }),
    tile_power_l_1n_get_charge_status: ({ state }) => ({
      scalar: chargeStatus(state, resolveInputs(state))
    }),
    tile_power_l_1n_get_charge_error: ({ state }) => ({ scalar: state.fault ? 1 : 0 }),
    tile_power_l_1n_is_charging: ({ state }) => ({
      scalar: charging(state, resolveInputs(state)) ? 1 : 0
    }),
    tile_power_l_1n_is_charge_complete: ({ state }) => ({
      scalar: complete(state, resolveInputs(state)) ? 1 : 0
    }),
    tile_power_l_1n_battery_present: ({ state }) => ({
      scalar: resolveInputs(state).batteryPresent ? 1 : 0
    }),
    // ── measurements ──
    tile_power_l_1n_get_vbat_mv: ({ state }) => ({
      scalar: adcMv(resolveInputs(state).battV, 5e3)
    }),
    tile_power_l_1n_get_vsys_mv: ({ state }) => ({
      scalar: adcMv(vsys(resolveInputs(state)), 6375)
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
      const mv = 1e3 + Math.floor((clamp(pick(args, 1, 1800), 1e3, 3300) - 1e3) / 100) * 100;
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
    tile_power_l_1n_get_reset_cause: () => ({ scalar: 0 })
  },
  provenance: {
    // canonical — datasheet- / tile-JSON-accurate value or bit layout
    tile_power_l_1n_find: "canonical",
    // I2C 0x6B
    tile_power_l_1n_get_charge_status: "canonical",
    // BCHGCHARGESTATUS bit layout
    tile_power_l_1n_is_charging: "canonical",
    tile_power_l_1n_is_charge_complete: "canonical",
    tile_power_l_1n_battery_present: "canonical",
    tile_power_l_1n_get_vbat_mv: "canonical",
    // VBAT mV (FS 5.0 V)
    tile_power_l_1n_get_vsys_mv: "canonical",
    // VSYS = higher of inputs
    tile_power_l_1n_get_die_temp_c: "canonical",
    // die-temp transfer
    tile_power_l_1n_set_charge_current_ma: "canonical",
    // 32-800 mA range
    tile_power_l_1n_set_vbus_limit_ma: "canonical",
    // VBUSINILIM0: mA/100, boots at 100 mA
    tile_power_l_1n_set_term_mv: "canonical",
    // 3.5-4.45 V range
    tile_power_l_1n_buck_set_mv: "canonical",
    // 1.0-3.3 V range
    tile_power_l_1n_led_set_mode: "canonical",
    // LEDDRVxMODESEL codes
    // inferred — behavior follows obviously from the driver but isn't a datasheet number
    tile_power_l_1n_charger_enable: "inferred",
    tile_power_l_1n_buck_enable: "inferred",
    tile_power_l_1n_led_set: "inferred",
    tile_power_l_1n_get_charge_error: "inferred",
    // fault → nonzero code (reason bits not modeled)
    // hallucinated — not really modeled
    tile_power_l_1n_get_reset_cause: "hallucinated",
    // always reports 0
    power: "inferred"
    // voltages/topology canonical; current split is a modeling choice
  },
  // On-board status LEDs (LED0 green / host, LED1 yellow / charging, LED2 red /
  // error) — declared as indicators so the canvas renders them generically.
  indicators(state, ctx) {
    const isChg = charging(state, resolveInputs(state, ctx));
    const isFault = state.fault === 1;
    return [
      {
        id: "led0",
        label: "Host (green)",
        color: "#22c55e",
        level: ledOn(state.led0_mode, state.led0_host, isChg, isFault)
      },
      {
        id: "led1",
        label: "Charging (yellow)",
        color: "#eab308",
        level: ledOn(state.led1_mode, state.led1_host, isChg, isFault)
      },
      {
        id: "led2",
        label: "Error (red)",
        color: "#ef4444",
        level: ledOn(state.led2_mode, state.led2_host, isChg, isFault)
      }
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
    const chargeUa = isChg ? state.charge_current_ma * 1e3 : 0;
    const ownUa = inputOk ? chargeUa + QUIESCENT_UA : 0;
    const rails = [];
    rails.push({
      name: "CHG",
      role: "supply",
      v_mv: chgV,
      ...usbPresent ? {} : { i_ua: 0 },
      pads: ["6"],
      // the VBUS input limiter (500 mA once the driver's init() has run)
      limit_ua: state.vbus_ilim_ma * 1e3,
      note: usbPresent ? isChg ? "USB input \u2014 charging + system" : "USB input" : "USB absent"
    });
    rails.push(
      usbPresent ? {
        name: "BATT",
        role: "supply",
        v_mv: battV,
        i_ua: 0,
        pads: ["7"],
        note: isChg ? "battery receiving charge" : "battery idle"
      } : {
        name: "BATT",
        role: "supply",
        v_mv: battV,
        pads: ["7"],
        note: "battery input (discharging)"
      }
    );
    const buckOut = (en, setMv) => inputOk && en ? Math.min(setMv, Math.max(0, vinMv - BUCK_DROPOUT_MV)) : 0;
    rails.push({
      name: "VSYS",
      role: "output",
      v_mv: inputOk ? vinMv : 0,
      pads: ["8"],
      note: "system (\u2248 higher of CHG/BATT)"
    });
    rails.push({
      name: "3V3",
      role: "output",
      v_mv: buckOut(state.buck2_en, state.buck2_mv),
      pads: ["9"],
      limit_ua: BUCK_LIMIT_UA,
      note: "buck2 (3.3 V; follows VSYS down in dropout)"
    });
    rails.push({
      name: "1V8",
      role: "output",
      v_mv: buckOut(state.buck1_en, state.buck1_mv),
      pads: ["10"],
      limit_ua: BUCK_LIMIT_UA,
      note: "buck1 (1.8 V; follows VSYS down in dropout)"
    });
    return { draw_ua: ownUa, rails };
  }
};
var power_l_1n_default = sim;
export {
  power_l_1n_default as default
};
