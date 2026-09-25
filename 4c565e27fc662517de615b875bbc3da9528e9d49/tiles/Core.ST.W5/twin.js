// src/sims/core_st_w5.ts
var MODES = ["run", "sleep", "stop0", "stop1", "standby"];
var CLOCKS = [16, 32, 64, 100];
var BLE = ["off", "advertising", "connected", "tx", "rx", "tx_10dbm"];
var RUN_UA = {
  16: 910,
  32: 2290,
  64: 4110,
  100: 6160
};
var SLEEP_UA = {
  16: 340,
  32: 950,
  64: 1510,
  100: 2140
};
var FIXED_UA = {
  stop0: 49,
  // T51, SRAM2 + cache retained
  stop1: 22.6,
  // T52, SRAM1 retained, ULPMEN = 1
  standby: 0.37
  // T54, all peripherals disabled, ULPMEN = 1
};
var RADIO_TOTAL_UA = {
  tx: 10510,
  // Tx 0 dBm
  rx: 7910,
  // Rx 1 Mbps
  tx_10dbm: 21150
  // Tx +10 dBm
};
var RADIO_AVG_UA = {
  off: 0,
  advertising: 800,
  connected: 1200
};
var VDD_NOM_MV = 3300;
var VDD_MIN_MV = 1710;
var VDD_MAX_MV = 3600;
var LED_R_OHM = 40;
var LED_VF_MV = 1900;
var PIN_R_OHM = 55;
var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
var modeOf = (s) => MODES[clamp(s.power_mode, 0, MODES.length - 1)];
var clockOf = (s) => CLOCKS[clamp(s.sysclk, 0, CLOCKS.length - 1)];
var bleOf = (s) => BLE[clamp(s.ble_state, 0, BLE.length - 1)];
var vddOf = (ctx) => ctx?.padVoltage?.["14"] ?? VDD_NOM_MV;
var ledUa = (s, vdd) => s.led_on && modeOf(s) !== "standby" && vdd > LED_VF_MV ? Math.round((vdd - LED_VF_MV) / (LED_R_OHM + PIN_R_OHM) * 1e3) : 0;
function mcuUa(s) {
  const mode = modeOf(s);
  if (mode !== "run" && mode !== "sleep") return FIXED_UA[mode];
  const core = mode === "run" ? RUN_UA[clockOf(s)] : SLEEP_UA[clockOf(s)];
  const ble = bleOf(s);
  if (ble in RADIO_TOTAL_UA) return Math.max(core, RADIO_TOTAL_UA[ble]);
  return core + RADIO_AVG_UA[ble];
}
var sim = {
  tile: "Core.ST.W5",
  defaultState: {
    power_mode: 0,
    // run
    sysclk: 1,
    // 32 MHz (Clock knob default "medium")
    led_on: 0,
    ble_state: 0,
    // off
    i2c_pullups: 0
    // PC14/PC15 reset to analog (Hi-Z): pull-ups disengaged
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
      label: "Clock (0:16\xB71:32\xB72:64\xB73:100 MHz)",
      min: 0,
      max: 3,
      step: 1
    },
    {
      type: "slider",
      field: "ble_state",
      label: "BLE (0 off\xB71 adv\xB72 conn\xB73 tx 0 dBm\xB74 rx\xB75 tx +10 dBm)",
      min: 0,
      max: 5,
      step: 1
    },
    { type: "toggle", field: "led_on", label: "On-board LED (PB12)" },
    {
      type: "toggle",
      field: "i2c_pullups",
      label: "2.2k I2C3 pull-ups engaged (PC15/PC14 high)"
    }
  ],
  hostCalls: {},
  provenance: {
    power: "inferred"
    // MCU + Tx/Rx currents canonical (DS14127, LDO); adv/connected averages + LED estimated
  },
  // With PC15 / PC14 driven high the 2.2k resistors pull I2C3 (pad 4 CLK, pad 5
  // DAT) high. LED (PB12) and the antenna are internal, not numbered pads.
  padOutputs(state) {
    return state.i2c_pullups ? { "4": 1, "5": 1 } : {};
  },
  // Supply draw on V+ (pad 14) / GND (pad 1): MCU (+ radio) + LED.
  power(state, ctx) {
    const vdd = vddOf(ctx);
    const on = vdd >= VDD_MIN_MV;
    const ua = on ? mcuUa(state) + ledUa(state, vdd) : 0;
    const i = Math.round(ua * 1e3) / 1e3;
    const ble = bleOf(state);
    return {
      draw_ua: i,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: vdd,
          i_ua: i,
          pads: ["14"],
          note: on ? `STM32WBA55 ${modeOf(state)}${ble !== "off" ? " +BLE " + ble : ""}${state.led_on ? " +LED" : ""}${vdd > VDD_MAX_MV ? " (V+ above 3.6 V max)" : ""}` : "STM32WBA55 unpowered (V+ below 1.71 V)"
        }
      ]
    };
  }
};
var core_st_w5_default = sim;
export {
  core_st_w5_default as default
};
