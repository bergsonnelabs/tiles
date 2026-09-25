// Digital twin for Core.ST.W5 (formerly Core.W): the BLE MCU host board itself.
//
// A Core, not a peripheral: no tile driver, no host calls. The twin models the
// board standards: STM32WBA55 power across run/sleep/stop/standby, the BLE radio
// as a power state (the on-board Johanson 2450AT chip antenna is RF-only, no pad),
// the on-board status LED, and the two software-switched I2C3 pull-ups.
//
// Regulator: LDO only. The tile schematic ties VDDSMPS and VLXSMPS to ground with
// no SMPS inductor (DS14127 §3.12.1 caution for SMPS parts used in an LDO application),
// so the SMPS figures in the datasheet do not apply to this board.
//
// Power numbers: STM32WBA5xxx datasheet (DS14127 Rev 7, TYP @ 3.3 V / 25 °C),
// LDO rows of Tables 45/49/51/52/54 and BLE radio Table 37. Pad map per
// Core-ST-W5-b.json (V+ pad 14, GND pad 1, I2C3 CLK/DAT pads 4/5). Board parts
// per the tile schematic: red LED on PB12 through R2 = 40 Ω; R3 = 2.2k from PC15
// to pad 4 (I2C3 SCL) and R4 = 2.2k from PC14 to pad 5 (I2C3 SDA).
import type { PowerCtx, TileSim } from '../tileSim';

// power_mode slider index → mode.
const MODES = ['run', 'sleep', 'stop0', 'stop1', 'standby'] as const;
// sysclk slider index → MHz: the tile's Clock knob (definition config.clock:
// low 16 MHz HSI16, medium 32 MHz HSE [default], high 64 MHz PLL, max 100 MHz PLL).
const CLOCKS = [16, 32, 64, 100] as const;
// ble_state slider index → activity.
const BLE = ['off', 'advertising', 'connected', 'tx', 'rx', 'tx_10dbm'] as const;

// MCU current (µA) by clock on the LDO (T45 Run, T49 Sleep). 64 MHz has no row:
// interpolated linearly between the 32 and 100 MHz points (both Range 1 + HSE).
const RUN_UA: Record<number, number> = {
  16: 910,
  32: 2290,
  64: 4110,
  100: 6160,
};
const SLEEP_UA: Record<number, number> = {
  16: 340,
  32: 950,
  64: 1510,
  100: 2140,
};
// Fixed low-power modes (µA), LDO, 3.3 V.
const FIXED_UA: Record<string, number> = {
  stop0: 49, // T51, SRAM2 + cache retained
  stop1: 22.6, // T52, SRAM1 retained, ULPMEN = 1
  standby: 0.37, // T54, all peripherals disabled, ULPMEN = 1
};
// Radio. Table 37 figures are the WHOLE device during Tx/Rx ("including 2.4 GHz
// RADIO subsystem and digital processing"), LDO: they replace the core figure
// rather than add to it. Advertising / connected are duty-cycled averages added
// on top of the core (estimated: the datasheet gives only the active peaks, and
// the real average depends on the interval the program picks).
const RADIO_TOTAL_UA: Record<string, number> = {
  tx: 10510, // Tx 0 dBm
  rx: 7910, // Rx 1 Mbps
  tx_10dbm: 21150, // Tx +10 dBm
};
const RADIO_AVG_UA: Record<string, number> = {
  off: 0,
  advertising: 800,
  connected: 1200,
};

const VDD_NOM_MV = 3300; // V+ when the pad isn't wired to a known rail
const VDD_MIN_MV = 1710; // DS14127 Table 31 (definition power[].min says 1.8 V)
const VDD_MAX_MV = 3600; // Table 31 operating max
const LED_R_OHM = 40; // R2 (schematic)
const LED_VF_MV = 1900; // red LED forward drop (assumed; LED part not in the BOM)
// GPIO output resistance: VOH = VDD - 0.4 V at 8 mA .. VDD - 1.3 V at 20 mA
// (DS14127 Table 77), ~55 Ω. With R2 at only 40 Ω the pin sets most of the current.
const PIN_R_OHM = 55;

interface State {
  power_mode: number;
  sysclk: number;
  led_on: number; // on-board red LED (PB12, active-high)
  ble_state: number;
  i2c_pullups: number; // PC15/PC14 driven high: 2.2k pull-ups on pads 4/5 engaged
  [field: string]: number;
}

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const modeOf = (s: State) => MODES[clamp(s.power_mode, 0, MODES.length - 1)];
const clockOf = (s: State) => CLOCKS[clamp(s.sysclk, 0, CLOCKS.length - 1)];
const bleOf = (s: State) => BLE[clamp(s.ble_state, 0, BLE.length - 1)];
// V+ from the wiring (pad 14) when it's on a resolved net, else nominal.
const vddOf = (ctx?: PowerCtx) => ctx?.padVoltage?.['14'] ?? VDD_NOM_MV;

// LED current when lit: (VDD - Vf) across R2 plus the pin's output resistance.
const ledUa = (s: State, vdd: number) =>
  s.led_on && modeOf(s) !== 'standby' && vdd > LED_VF_MV
    ? Math.round(((vdd - LED_VF_MV) / (LED_R_OHM + PIN_R_OHM)) * 1000)
    : 0;

// MCU + radio (µA). The radio only runs while the MCU is awake (run/sleep).
function mcuUa(s: State): number {
  const mode = modeOf(s);
  if (mode !== 'run' && mode !== 'sleep') return FIXED_UA[mode];
  const core = mode === 'run' ? RUN_UA[clockOf(s)] : SLEEP_UA[clockOf(s)];
  const ble = bleOf(s);
  if (ble in RADIO_TOTAL_UA) return Math.max(core, RADIO_TOTAL_UA[ble]);
  return core + RADIO_AVG_UA[ble];
}

const sim: TileSim<State> = {
  tile: 'Core.ST.W5',

  defaultState: {
    power_mode: 0, // run
    sysclk: 1, // 32 MHz (Clock knob default "medium")
    led_on: 0,
    ble_state: 0, // off
    i2c_pullups: 0, // PC14/PC15 reset to analog (Hi-Z): pull-ups disengaged
  },

  controls: [
    {
      type: 'slider',
      field: 'power_mode',
      label: 'Mode (0 run·1 sleep·2 stop0·3 stop1·4 standby)',
      min: 0,
      max: 4,
      step: 1,
    },
    {
      type: 'slider',
      field: 'sysclk',
      label: 'Clock (0:16·1:32·2:64·3:100 MHz)',
      min: 0,
      max: 3,
      step: 1,
    },
    {
      type: 'slider',
      field: 'ble_state',
      label: 'BLE (0 off·1 adv·2 conn·3 tx 0 dBm·4 rx·5 tx +10 dBm)',
      min: 0,
      max: 5,
      step: 1,
    },
    { type: 'toggle', field: 'led_on', label: 'On-board LED (PB12)' },
    {
      type: 'toggle',
      field: 'i2c_pullups',
      label: '2.2k I2C3 pull-ups engaged (PC15/PC14 high)',
    },
  ],

  hostCalls: {},

  provenance: {
    power: 'inferred', // MCU + Tx/Rx currents canonical (DS14127, LDO); adv/connected averages + LED estimated
  },

  // With PC15 / PC14 driven high the 2.2k resistors pull I2C3 (pad 4 CLK, pad 5
  // DAT) high. LED (PB12) and the antenna are internal, not numbered pads.
  padOutputs(state): Record<string, number> {
    return state.i2c_pullups ? { '4': 1, '5': 1 } : {};
  },

  // Supply draw on V+ (pad 14) / GND (pad 1): MCU (+ radio) + LED.
  power(state, ctx) {
    const vdd = vddOf(ctx);
    const on = vdd >= VDD_MIN_MV;
    const ua = on ? mcuUa(state) + ledUa(state, vdd) : 0;
    const i = Math.round(ua * 1000) / 1000;
    const ble = bleOf(state);
    return {
      draw_ua: i,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: vdd,
          i_ua: i,
          pads: ['14'],
          note: on
            ? `STM32WBA55 ${modeOf(state)}${ble !== 'off' ? ' +BLE ' + ble : ''}${state.led_on ? ' +LED' : ''}${vdd > VDD_MAX_MV ? ' (V+ above 3.6 V max)' : ''}`
            : 'STM32WBA55 unpowered (V+ below 1.71 V)',
        },
      ],
    };
  },
};

export default sim;
