// Digital twin for Core.ST.W5 (formerly Core.W) — the BLE MCU host board itself.
//
// A Core, not a peripheral: no tile driver, no host calls. The twin models the
// board standards — STM32WBA55 power across run/sleep/stop/standby (LDO vs SMPS
// regulator path), the BLE radio as a power state (the on-board Johanson 2450AT
// chip antenna is RF-only, no pad), the on-board status LED, and the optional
// I2C pull-ups.
//
// Power numbers: STM32WBA5xxx datasheet (DS14127 Rev 7, TYP @ 3.3 V / 25 °C),
// MCU Tables 45/46/49/51/52/54 and BLE radio Table 37. Pad map per Core-ST-W5-b.json
// (V+ pad 14, GND pad 1, I2C3 CLK/DAT pads 4/5).
import type { TileSim } from '../tileSim';

// power_mode slider index → mode.
const MODES = ['run', 'sleep', 'stop0', 'stop1', 'standby'] as const;
// sysclk slider index → MHz.
const CLOCKS = [16, 32, 100] as const;
// ble_state slider index → activity.
const BLE = ['off', 'advertising', 'connected', 'tx', 'rx'] as const;

// MCU core current (µA) by clock, per regulator path (LDO / SMPS).
const RUN_UA = { ldo: { 16: 910, 32: 2290, 100: 6160 }, smps: { 16: 450, 32: 1470, 100: 3350 } };
const SLEEP_UA = { ldo: { 16: 340, 32: 950, 100: 2140 }, smps: { 16: 220, 32: 820, 100: 1500 } };
// Fixed low-power modes (µA) — SMPS path largely matches in deep stop/standby.
const FIXED_UA: Record<string, { ldo: number; smps: number }> = {
  stop0: { ldo: 49, smps: 11 },
  stop1: { ldo: 22.6, smps: 22.6 },
  standby: { ldo: 0.37, smps: 0.37 },
};
// BLE radio adder (µA) on top of core current (Table 37, 0 dBm Tx / 1 Mbps Rx).
// advertising/connected are duty-cycled averages (estimated — datasheet gives peaks only).
const RADIO_UA: Record<string, { ldo: number; smps: number }> = {
  off: { ldo: 0, smps: 0 },
  advertising: { ldo: 800, smps: 500 },
  connected: { ldo: 1200, smps: 700 },
  tx: { ldo: 10510, smps: 5540 },
  rx: { ldo: 7910, smps: 5220 },
};

const RAIL_MV = 3300;
const LED_R_OHM = 40; // R2 (schematic) — atypically low, see provenance
const LED_VF_MV = 1900;

interface State {
  power_mode: number;
  sysclk: number;
  regulator: number; // 0 = LDO, 1 = SMPS
  led_on: number; // on-board red LED (PB12, active-high)
  ble_state: number;
  i2c_pullups: number; // optional 2.2k pull-ups on I2C3 (pads 4/5)
  [field: string]: number;
}

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const modeOf = (s: State) => MODES[clamp(s.power_mode, 0, MODES.length - 1)];
const clockOf = (s: State) => CLOCKS[clamp(s.sysclk, 0, CLOCKS.length - 1)];
const bleOf = (s: State) => BLE[clamp(s.ble_state, 0, BLE.length - 1)];
const regOf = (s: State): 'ldo' | 'smps' => (s.regulator ? 'smps' : 'ldo');

const ledUa = (s: State) =>
  s.led_on && RAIL_MV > LED_VF_MV ? Math.round(((RAIL_MV - LED_VF_MV) / LED_R_OHM) * 1000) : 0;

function coreUa(s: State): number {
  const reg = regOf(s);
  const mode = modeOf(s);
  if (mode === 'run') return RUN_UA[reg][clockOf(s)];
  if (mode === 'sleep') return SLEEP_UA[reg][clockOf(s)];
  return FIXED_UA[mode][reg];
}
// Radio only runs when the MCU is awake (run/sleep); deep-sleep modes drop it.
function radioUa(s: State): number {
  const mode = modeOf(s);
  if (mode !== 'run' && mode !== 'sleep') return 0;
  return RADIO_UA[bleOf(s)][regOf(s)];
}

const sim: TileSim<State> = {
  tile: 'Core.ST.W5',

  defaultState: {
    power_mode: 0, // run
    sysclk: 1, // 32 MHz
    regulator: 1, // SMPS (default firmware path)
    led_on: 0,
    ble_state: 0, // off
    i2c_pullups: 1,
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
      label: 'Clock (0:16·1:32·2:100 MHz)',
      min: 0,
      max: 2,
      step: 1,
    },
    { type: 'toggle', field: 'regulator', label: 'SMPS regulator (off = LDO)' },
    {
      type: 'slider',
      field: 'ble_state',
      label: 'BLE (0 off·1 adv·2 conn·3 tx·4 rx)',
      min: 0,
      max: 4,
      step: 1,
    },
    { type: 'toggle', field: 'led_on', label: 'On-board LED (PB12)' },
    { type: 'toggle', field: 'i2c_pullups', label: 'I2C3 pull-ups populated' },
  ],

  hostCalls: {},

  provenance: {
    power: 'inferred', // MCU + Tx/Rx currents canonical (DS14127); adv/connected averages + LED estimated
  },

  // Optional 2.2k pull-ups idle I2C3 (pads 4 CLK / 5 DAT) high. LED (PB12) and the
  // antenna (RF matching network) are internal — neither is a numbered pad.
  padOutputs(state) {
    const up = state.i2c_pullups ? 1 : 0;
    return { 'I2C3.CLK': up, 'I2C3.DAT': up };
  },

  // Supply draw on V+ (pad 14) / GND (pad 1): MCU-mode + BLE radio + LED.
  power(state) {
    const ua = coreUa(state) + radioUa(state) + ledUa(state);
    const ble = bleOf(state);
    return {
      draw_ua: Math.round(ua * 1000) / 1000,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: RAIL_MV,
          i_ua: Math.round(ua * 1000) / 1000,
          pads: ['14'],
          note: `STM32WBA55 ${modeOf(state)}/${regOf(state)}${ble !== 'off' ? ' +BLE ' + ble : ''}${state.led_on ? ' +LED' : ''}`,
        },
      ],
    };
  },
};

export default sim;
