// Digital twin for Core.ST.L4.1 (formerly Core.U.1) — the MCU host board itself.
//
// A Core, not a peripheral: no tile driver and no host calls. The twin models the
// board-level standards — STM32L422 power across run / low-power / stop / standby
// modes, the on-board status LED, and the optional I2C pull-ups.
//
// MCU: STM32L422TB (Cortex-M4). Power numbers from its datasheet (DS12470 Rev 6,
// TYP @ 3.0 V / 25 °C). Pad map per Core-ST-L4-1-b.json (V+ pad 10, GND pad 1,
// I2C1 CLK/DAT pads 4/5).
import type { TileSim } from '../tileSim';

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
// clock_idx slider index → MHz (matches the tile's MSI/HSI16/PLL clock options).
const CLOCKS = [1, 2, 16, 32] as const;

// Run / Sleep current (µA) by clock (DS12470 T25 / T31, 3 V typ).
const RUN_UA: Record<number, number> = { 1: 190, 2: 265, 16: 1550, 32: 3000 };
const SLEEP_UA: Record<number, number> = { 1: 100, 2: 150, 16: 425, 32: 725 };
// Fixed-current modes (µA) — clock-independent (T31/T33/T34/T35/T36/T37).
const FIXED_UA: Record<string, number> = {
  lp_run: 190, // LP run @ 2 MHz MSI
  lp_sleep: 52.5, // LP sleep @ 2 MHz
  stop0: 115,
  stop1: 4.0,
  stop2: 0.75,
  standby: 0.12, // 120 nA
  shutdown: 0.031, // 31 nA
};

const RAIL_MV = 3300; // V+ operating typical
const LED_R_OHM = 400; // R1 (schematic)
const LED_VF_MV = 1900; // red LED forward drop (assumed)

interface State {
  power_mode: number; // index into MODES
  clock_idx: number; // index into CLOCKS (run/sleep only)
  led_on: number; // on-board red LED (PA8, active-high)
  i2c_pullups: number; // optional 2.2k pull-ups on pads 4/5 populated
  [field: string]: number;
}

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const modeOf = (s: State) => MODES[clamp(s.power_mode, 0, MODES.length - 1)];
const clockOf = (s: State) => CLOCKS[clamp(s.clock_idx, 0, CLOCKS.length - 1)];

// LED current when lit (Ohm's law on R1; off below the rail's Vf).
const ledUa = (s: State) =>
  s.led_on && RAIL_MV > LED_VF_MV ? Math.round(((RAIL_MV - LED_VF_MV) / LED_R_OHM) * 1000) : 0;

// MCU supply current (µA) for the selected mode.
function coreUa(s: State): number {
  const mode = modeOf(s);
  if (mode === 'run') return RUN_UA[clockOf(s)];
  if (mode === 'sleep') return SLEEP_UA[clockOf(s)];
  return FIXED_UA[mode];
}

const sim: TileSim<State> = {
  tile: 'Core.ST.L4.1',

  defaultState: {
    power_mode: 0, // run
    clock_idx: 1, // 2 MHz (MSI default)
    led_on: 0,
    i2c_pullups: 1,
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
      label: 'Clock (0:1·1:2·2:16·3:32 MHz, run/sleep only)',
      min: 0,
      max: 3,
      step: 1,
    },
    { type: 'toggle', field: 'led_on', label: 'On-board LED (PA8)' },
    { type: 'toggle', field: 'i2c_pullups', label: 'I2C pull-ups populated' },
  ],

  // No host calls — a Core has no peripheral driver.
  hostCalls: {},

  provenance: {
    power: 'inferred', // MCU-mode currents canonical (DS12470); LED current estimated
  },

  // The board's only standing pad behaviour: when the optional 2.2k pull-ups are
  // populated, I2C1 (pads 4 CLK / 5 DAT) idles high. The LED (PA8) is internal,
  // not a numbered pad, so it isn't a pad output.
  padOutputs(state) {
    const up = state.i2c_pullups ? 1 : 0;
    return { 'I2C1.CLK': up, 'I2C1.DAT': up };
  },

  // Supply draw on V+ (pad 10) / GND (pad 1): MCU-mode current + LED when lit.
  power(state) {
    const ua = coreUa(state) + ledUa(state);
    return {
      draw_ua: Math.round(ua * 1000) / 1000,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: RAIL_MV,
          i_ua: Math.round(ua * 1000) / 1000,
          pads: ['10'],
          note: `STM32L422 ${modeOf(state)}${state.led_on ? ' + LED' : ''}`,
        },
      ],
    };
  },
};

export default sim;
