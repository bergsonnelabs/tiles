// src/sims/core_st_w5.ts
var MODES = ["run", "sleep", "stop0", "stop1", "standby"];
var CLOCKS = [16, 32, 100];
var BLE = ["off", "advertising", "connected", "tx", "rx"];
var RUN_UA = { ldo: { 16: 910, 32: 2290, 100: 6160 }, smps: { 16: 450, 32: 1470, 100: 3350 } };
var SLEEP_UA = { ldo: { 16: 340, 32: 950, 100: 2140 }, smps: { 16: 220, 32: 820, 100: 1500 } };
var FIXED_UA = {
  stop0: { ldo: 49, smps: 11 },
  stop1: { ldo: 22.6, smps: 22.6 },
  standby: { ldo: 0.37, smps: 0.37 }
};
var RADIO_UA = {
  off: { ldo: 0, smps: 0 },
  advertising: { ldo: 800, smps: 500 },
  connected: { ldo: 1200, smps: 700 },
  tx: { ldo: 10510, smps: 5540 },
  rx: { ldo: 7910, smps: 5220 }
};
var RAIL_MV = 3300;
var LED_R_OHM = 40;
var LED_VF_MV = 1900;
var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
var modeOf = (s) => MODES[clamp(s.power_mode, 0, MODES.length - 1)];
var clockOf = (s) => CLOCKS[clamp(s.sysclk, 0, CLOCKS.length - 1)];
var bleOf = (s) => BLE[clamp(s.ble_state, 0, BLE.length - 1)];
var regOf = (s) => s.regulator ? "smps" : "ldo";
var ledUa = (s) => s.led_on && RAIL_MV > LED_VF_MV ? Math.round((RAIL_MV - LED_VF_MV) / LED_R_OHM * 1e3) : 0;
function coreUa(s) {
  const reg = regOf(s);
  const mode = modeOf(s);
  if (mode === "run") return RUN_UA[reg][clockOf(s)];
  if (mode === "sleep") return SLEEP_UA[reg][clockOf(s)];
  return FIXED_UA[mode][reg];
}
function radioUa(s) {
  const mode = modeOf(s);
  if (mode !== "run" && mode !== "sleep") return 0;
  return RADIO_UA[bleOf(s)][regOf(s)];
}
var sim = {
  tile: "Core.ST.W5",
  defaultState: {
    power_mode: 0,
    // run
    sysclk: 1,
    // 32 MHz
    regulator: 1,
    // SMPS (default firmware path)
    led_on: 0,
    ble_state: 0,
    // off
    i2c_pullups: 1
  },
  controls: [
    {
      type: "slider",
      field: "power_mode",
      label: "Mode (0 run\xB71 sleep\xB72 stop0\xB73 stop1\xB74 standby)",
      min: 0,
      max: 4,
      step: 1
    },
    {
      type: "slider",
      field: "sysclk",
      label: "Clock (0:16\xB71:32\xB72:100 MHz)",
      min: 0,
      max: 2,
      step: 1
    },
    { type: "toggle", field: "regulator", label: "SMPS regulator (off = LDO)" },
    {
      type: "slider",
      field: "ble_state",
      label: "BLE (0 off\xB71 adv\xB72 conn\xB73 tx\xB74 rx)",
      min: 0,
      max: 4,
      step: 1
    },
    { type: "toggle", field: "led_on", label: "On-board LED (PB12)" },
    { type: "toggle", field: "i2c_pullups", label: "I2C3 pull-ups populated" }
  ],
  hostCalls: {},
  provenance: {
    power: "inferred"
    // MCU + Tx/Rx currents canonical (DS14127); adv/connected averages + LED estimated
  },
  // Optional 2.2k pull-ups idle I2C3 (pads 4 CLK / 5 DAT) high. LED (PB12) and the
  // antenna (RF matching network) are internal — neither is a numbered pad.
  padOutputs(state) {
    const up = state.i2c_pullups ? 1 : 0;
    return { "I2C3.CLK": up, "I2C3.DAT": up };
  },
  // Supply draw on V+ (pad 14) / GND (pad 1): MCU-mode + BLE radio + LED.
  power(state) {
    const ua = coreUa(state) + radioUa(state) + ledUa(state);
    const ble = bleOf(state);
    return {
      draw_ua: Math.round(ua * 1e3) / 1e3,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: RAIL_MV,
          i_ua: Math.round(ua * 1e3) / 1e3,
          pads: ["14"],
          note: `STM32WBA55 ${modeOf(state)}/${regOf(state)}${ble !== "off" ? " +BLE " + ble : ""}${state.led_on ? " +LED" : ""}`
        }
      ]
    };
  }
};
var core_st_w5_default = sim;
export {
  core_st_w5_default as default
};
