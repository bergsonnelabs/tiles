// Digital twin for Core.ST.L4 (formerly Core.ST.L4.1, and before that Core.U.1):
// the MCU host board itself. Not the Core.ST.L4.2, a different board.
//
// A Core, not a peripheral: no tile driver and no host calls. The twin models the
// board-level standards: STM32L422 power across run / low-power / stop / standby
// modes, the on-board status LED, and the two software-switched I2C1 pull-ups.
//
// MCU: STM32L422TB (Cortex-M4). Power numbers from its datasheet (DS12470 Rev 6,
// TYP @ 3.0 V / 25 °C, all peripherals disabled). Pad map per Core-ST-L4-b.json
// (V+ pad 10, GND pad 1, I2C1 CLK/DAT pads 4/5). Board parts per the tile
// schematic: red LED on PA8 through R1 = 40 Ω; R3 = 2.2k from PC15 to pad 4
// (I2C1 SCL) and R4 = 2.2k from PA9 to pad 5 (I2C1 SDA). PC15 sits behind the
// backup-domain power switch and can source only 3 mA (DS12470 §6.3.14); a
// 2.2k pull-up held low at 3.3 V asks 1.5 mA of it, inside that.
import type { PowerCtx, TileSim } from '../tileSim';

// power_mode slider index → mode (datasheet operating modes).
const MODES = [
  'run',
  'lp_run',
  'sleep',
  'lp_sleep',
  'stop0',
  'stop1',
  'stop2',
  'standby',
  'shutdown',
] as const;
// clock_idx slider index → MHz: the tile's Clock knob (definition config.clock:
// low 8 MHz MSI, medium 16 MHz MSI [default], high 48 MHz MSI, max 80 MHz PLL).
const CLOCKS = [8, 16, 48, 80] as const;

// Run / Sleep current (µA) by clock. Coregen never lowers VOS on the L4, so the
// core stays in voltage Range 1 (reset default): DS12470 Table 25 (Run) and
// Table 31 (Sleep), Range 1 rows. 8 MHz has no Range 1 row; the Range 2 figure
// is used, so the real draw there is slightly higher.
const RUN_UA: Record<number, number> = { 8: 715, 16: 1550, 48: 4400, 80: 7300 };
const SLEEP_UA: Record<number, number> = {
  8: 245,
  16: 425,
  48: 1000,
  80: 1650,
};
// Fixed-current modes (µA), clock-independent, 3 V typ.
const FIXED_UA: Record<string, number> = {
  lp_run: 190, // T25: LP run @ 2 MHz MSI
  lp_sleep: 52.5, // T31: LP sleep @ 2 MHz
  stop0: 115, // T35, RTC disabled
  stop1: 4.0, // T34, RTC disabled
  stop2: 0.79, // T33, RTC disabled
  standby: 0.12, // T36: 120 nA, no IWDG, RTC disabled
  shutdown: 0.031, // T37: 31 nA
};

const VDD_NOM_MV = 3300; // V+ when the pad isn't wired to a known rail
const VDD_MIN_MV = 1710; // DS12470 general operating conditions / definition power[].min
const VDD_MAX_MV = 3600; // operating max (4.0 V absolute max)
const LED_R_OHM = 40; // R1 (schematic)
const LED_VF_MV = 1900; // red LED forward drop (assumed; LED part not in the BOM)
// GPIO output resistance: VOH = VDD - 0.4 V at 8 mA .. VDD - 1.3 V at 20 mA
// (DS12470 Table 60), ~55 Ω. With R1 at only 40 Ω the pin,
// not the resistor, sets most of the LED current.
const PIN_R_OHM = 55;

interface State {
  power_mode: number; // index into MODES
  clock_idx: number; // index into CLOCKS (run/sleep only)
  led_on: number; // on-board red LED (PA8, active-high)
  i2c_pullups: number; // PC15/PA9 driven high: 2.2k pull-ups on pads 4/5 engaged
  [field: string]: number;
}

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const modeOf = (s: State) => MODES[clamp(s.power_mode, 0, MODES.length - 1)];
const clockOf = (s: State) => CLOCKS[clamp(s.clock_idx, 0, CLOCKS.length - 1)];
// V+ from the wiring (pad 10) when it's on a resolved net, else nominal.
const vddOf = (ctx?: PowerCtx) => ctx?.padVoltage?.['10'] ?? VDD_NOM_MV;

// LED current when lit: (VDD - Vf) across R1 plus the pin's output resistance.
// The LED only lights while the MCU runs (PA8 is not held through standby/shutdown).
const ledUa = (s: State, vdd: number) => {
  const mode = modeOf(s);
  if (!s.led_on || mode === 'standby' || mode === 'shutdown' || vdd <= LED_VF_MV) return 0;
  return Math.round(((vdd - LED_VF_MV) / (LED_R_OHM + PIN_R_OHM)) * 1000);
};

// MCU supply current (µA) for the selected mode.
function coreUa(s: State): number {
  const mode = modeOf(s);
  if (mode === 'run') return RUN_UA[clockOf(s)];
  if (mode === 'sleep') return SLEEP_UA[clockOf(s)];
  return FIXED_UA[mode];
}

const sim: TileSim<State> = {
  tile: 'Core.ST.L4',

  defaultState: {
    power_mode: 0, // run
    clock_idx: 1, // 16 MHz (Clock knob default "medium")
    led_on: 0,
    i2c_pullups: 0, // PA9/PC15 reset to analog (Hi-Z): pull-ups disengaged
  },

  controls: [
    {
      type: 'slider',
      field: 'power_mode',
      label: 'Mode (0 run·1 lpRun·2 sleep·3 lpSleep·4-6 stop0/1/2·7 standby·8 shutdown)',
      min: 0,
      max: 8,
      step: 1,
    },
    {
      type: 'slider',
      field: 'clock_idx',
      label: 'Clock (0:8·1:16·2:48·3:80 MHz, run/sleep only)',
      min: 0,
      max: 3,
      step: 1,
    },
    { type: 'toggle', field: 'led_on', label: 'On-board LED (PA8)' },
    {
      type: 'toggle',
      field: 'i2c_pullups',
      label: '2.2k I2C1 pull-ups engaged (PC15/PA9 high)',
    },
  ],

  // No host calls: a Core has no peripheral driver.
  hostCalls: {},

  provenance: {
    power: 'inferred', // MCU-mode currents canonical (DS12470); LED current estimated (Vf, pin R)
  },

  // With PC15 / PA9 driven high the 2.2k resistors pull I2C1 (pad 4 CLK, pad 5
  // DAT) high. The LED (PA8) is internal, not a numbered pad.
  padOutputs(state): Record<string, number> {
    return state.i2c_pullups ? { '4': 1, '5': 1 } : {};
  },

  // Supply draw on V+ (pad 10) / GND (pad 1): MCU-mode current + LED when lit.
  power(state, ctx) {
    const vdd = vddOf(ctx);
    const ua = vdd < VDD_MIN_MV ? 0 : coreUa(state) + ledUa(state, vdd);
    const i = Math.round(ua * 1000) / 1000;
    return {
      draw_ua: i,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: vdd,
          i_ua: i,
          pads: ['10'],
          note:
            vdd < VDD_MIN_MV
              ? 'STM32L422 unpowered (V+ below 1.71 V)'
              : `STM32L422 ${modeOf(state)}${state.led_on ? ' + LED' : ''}${vdd > VDD_MAX_MV ? ' (V+ above 3.6 V max)' : ''}`,
        },
      ],
    };
  },
};

export default sim;
