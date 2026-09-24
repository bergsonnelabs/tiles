// src/sims/core_st_l4_1.ts
var MODES = [
  "run",
  "lp_run",
  "sleep",
  "lp_sleep",
  "stop0",
  "stop1",
  "stop2",
  "standby",
  "shutdown"
];
var CLOCKS = [1, 2, 16, 32];
var RUN_UA = { 1: 190, 2: 265, 16: 1550, 32: 3e3 };
var SLEEP_UA = { 1: 100, 2: 150, 16: 425, 32: 725 };
var FIXED_UA = {
  lp_run: 190,
  // LP run @ 2 MHz MSI
  lp_sleep: 52.5,
  // LP sleep @ 2 MHz
  stop0: 115,
  stop1: 4,
  stop2: 0.75,
  standby: 0.12,
  // 120 nA
  shutdown: 0.031
  // 31 nA
};
var RAIL_MV = 3300;
var LED_R_OHM = 400;
var LED_VF_MV = 1900;
var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
var modeOf = (s) => MODES[clamp(s.power_mode, 0, MODES.length - 1)];
var clockOf = (s) => CLOCKS[clamp(s.clock_idx, 0, CLOCKS.length - 1)];
var ledUa = (s) => s.led_on && RAIL_MV > LED_VF_MV ? Math.round((RAIL_MV - LED_VF_MV) / LED_R_OHM * 1e3) : 0;
function coreUa(s) {
  const mode = modeOf(s);
  if (mode === "run") return RUN_UA[clockOf(s)];
  if (mode === "sleep") return SLEEP_UA[clockOf(s)];
  return FIXED_UA[mode];
}
var sim = {
  tile: "Core.ST.L4.1",
  defaultState: {
    power_mode: 0,
    // run
    clock_idx: 1,
    // 2 MHz (MSI default)
    led_on: 0,
    i2c_pullups: 1
  },
  controls: [
    {
      type: "slider",
      field: "power_mode",
      label: "Mode (0 run\xB71 lpRun\xB72 sleep\xB73 lpSleep\xB74-6 stop0/1/2\xB77 standby\xB78 shutdown)",
      min: 0,
      max: 8,
      step: 1
    },
    {
      type: "slider",
      field: "clock_idx",
      label: "Clock (0:1\xB71:2\xB72:16\xB73:32 MHz, run/sleep only)",
      min: 0,
      max: 3,
      step: 1
    },
    { type: "toggle", field: "led_on", label: "On-board LED (PA8)" },
    { type: "toggle", field: "i2c_pullups", label: "I2C pull-ups populated" }
  ],
  // No host calls — a Core has no peripheral driver.
  hostCalls: {},
  provenance: {
    power: "inferred"
    // MCU-mode currents canonical (DS12470); LED current estimated
  },
  // The board's only standing pad behaviour: when the optional 2.2k pull-ups are
  // populated, I2C1 (pads 4 CLK / 5 DAT) idles high. The LED (PA8) is internal,
  // not a numbered pad, so it isn't a pad output.
  padOutputs(state) {
    const up = state.i2c_pullups ? 1 : 0;
    return { "I2C1.CLK": up, "I2C1.DAT": up };
  },
  // Supply draw on V+ (pad 10) / GND (pad 1): MCU-mode current + LED when lit.
  power(state) {
    const ua = coreUa(state) + ledUa(state);
    return {
      draw_ua: Math.round(ua * 1e3) / 1e3,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: RAIL_MV,
          i_ua: Math.round(ua * 1e3) / 1e3,
          pads: ["10"],
          note: `STM32L422 ${modeOf(state)}${state.led_on ? " + LED" : ""}`
        }
      ]
    };
  }
};
var core_st_l4_1_default = sim;
export {
  core_st_l4_1_default as default
};
