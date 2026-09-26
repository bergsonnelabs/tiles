// src/sims/core_st_l4.ts
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
var CLOCKS = [8, 16, 48, 80];
var RUN_UA = { 8: 715, 16: 1550, 48: 4400, 80: 7300 };
var SLEEP_UA = {
  8: 245,
  16: 425,
  48: 1e3,
  80: 1650
};
var FIXED_UA = {
  lp_run: 190,
  // T25: LP run @ 2 MHz MSI
  lp_sleep: 52.5,
  // T31: LP sleep @ 2 MHz
  stop0: 115,
  // T35, RTC disabled
  stop1: 4,
  // T34, RTC disabled
  stop2: 0.79,
  // T33, RTC disabled
  standby: 0.12,
  // T36: 120 nA, no IWDG, RTC disabled
  shutdown: 0.031
  // T37: 31 nA
};
var VDD_NOM_MV = 3300;
var VDD_MIN_MV = 1710;
var VDD_MAX_MV = 3600;
var LED_R_OHM = 40;
var LED_VF_MV = 1900;
var PIN_R_OHM = 55;
var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
var modeOf = (s) => MODES[clamp(s.power_mode, 0, MODES.length - 1)];
var clockOf = (s) => CLOCKS[clamp(s.clock_idx, 0, CLOCKS.length - 1)];
var vddOf = (ctx) => ctx?.padVoltage?.["10"] ?? VDD_NOM_MV;
var ledUa = (s, vdd) => {
  const mode = modeOf(s);
  if (!s.led_on || mode === "standby" || mode === "shutdown" || vdd <= LED_VF_MV) return 0;
  return Math.round((vdd - LED_VF_MV) / (LED_R_OHM + PIN_R_OHM) * 1e3);
};
function coreUa(s) {
  const mode = modeOf(s);
  if (mode === "run") return RUN_UA[clockOf(s)];
  if (mode === "sleep") return SLEEP_UA[clockOf(s)];
  return FIXED_UA[mode];
}
var sim = {
  tile: "Core.ST.L4",
  defaultState: {
    power_mode: 0,
    // run
    clock_idx: 1,
    // 16 MHz (Clock knob default "medium")
    led_on: 0,
    i2c_pullups: 0
    // PA9/PC15 reset to analog (Hi-Z): pull-ups disengaged
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
      label: "Clock (0:8\xB71:16\xB72:48\xB73:80 MHz, run/sleep only)",
      min: 0,
      max: 3,
      step: 1
    },
    { type: "toggle", field: "led_on", label: "On-board LED (PA8)" },
    {
      type: "toggle",
      field: "i2c_pullups",
      label: "2.2k I2C1 pull-ups engaged (PC15/PA9 high)"
    }
  ],
  // No host calls: a Core has no peripheral driver.
  hostCalls: {},
  provenance: {
    power: "inferred"
    // MCU-mode currents canonical (DS12470); LED current estimated (Vf, pin R)
  },
  // With PC15 / PA9 driven high the 2.2k resistors pull I2C1 (pad 4 CLK, pad 5
  // DAT) high. The LED (PA8) is internal, not a numbered pad.
  padOutputs(state) {
    return state.i2c_pullups ? { "4": 1, "5": 1 } : {};
  },
  // Supply draw on V+ (pad 10) / GND (pad 1): MCU-mode current + LED when lit.
  power(state, ctx) {
    const vdd = vddOf(ctx);
    const ua = vdd < VDD_MIN_MV ? 0 : coreUa(state) + ledUa(state, vdd);
    const i = Math.round(ua * 1e3) / 1e3;
    return {
      draw_ua: i,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: vdd,
          i_ua: i,
          pads: ["10"],
          note: vdd < VDD_MIN_MV ? "STM32L422 unpowered (V+ below 1.71 V)" : `STM32L422 ${modeOf(state)}${state.led_on ? " + LED" : ""}${vdd > VDD_MAX_MV ? " (V+ above 3.6 V max)" : ""}`
        }
      ]
    };
  }
};
var core_st_l4_default = sim;
export {
  core_st_l4_default as default
};
