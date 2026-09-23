// Digital twin for Sense.ACP — ams-OSRAM TMD3725 ALS / RGB colour / proximity
// sensor. The physical world (ambient clear light, per-channel red/green/blue,
// and a proximity return) is driven by the controls; host calls return the
// counts firmware reads. ALS counts scale with the configured gain and
// integration time; the proximity count scales with proximity gain and LED
// drive — both datasheet-directional (Figures 13 & 15).
//
// Power note: the proximity IR LED is fed from an on-tile TPS61099 boost off
// the 1.8 V input, so proximity draw is modelled as boosted average LED current
// on top of the sensor's quiescent. Supply figures are estimates (the TMD3725
// datasheet publishes optical, not supply-current, tables) — marked inferred.
import type { TileSim } from '../tileSim';

interface State {
  // ── physical world (base counts at the reference gain/integration) ──
  clear: number; // Clear/ambient channel, 0..65535
  red: number; // Red channel, 0..65535
  green: number; // Green channel, 0..65535
  blue: number; // Blue channel, 0..65535
  proximity: number; // Proximity return, 0..255 (near object → high)

  // ── configuration (set by the driver's setters) ──
  als_gain: number; // AGAIN code 0..3 → 1/4/16/64x
  atime: number; // ATIME register; integration = (atime+1)·2.78 ms
  prox_gain: number; // PGAIN code 0..3 → 1/2/4/8x
  prox_drive_ma: number; // IR LED drive current, mA

  // ── operation ──
  als_on: number; // ALS engine enabled
  prox_on: number; // proximity engine enabled

  [field: string]: number;
}

// STATUS register bits (mirrors TMD3725_ST_* in the driver header).
const ST_PSAT = 0x40;
const ST_ASAT = 0x80;

// Reference operating point the base control values are expressed at.
const ALS_GAIN_MUL = [1, 4, 16, 64]; // AGAIN code → multiplier
const ALS_GAIN_REF = 16; // controls are counts "as seen at 16x"
const ATIME_REF = 0x2f; // ...and the default ~133 ms integration
const PROX_GAIN_MUL = [1, 2, 4, 8]; // PGAIN code → multiplier
const PROX_GAIN_REF = 4; // controls are counts "as seen at 4x"
const PROX_DRIVE_REF = 12; // ...and 12 mA LED drive

const clamp = (v: number, lo: number, hi: number) => (v < lo ? lo : v > hi ? hi : v);
const pick = (args: number[], i: number, cur: number) =>
  args.length > i && Number.isFinite(args[i]) ? args[i] : cur;

// Full-scale ALS count: 1024 per integration step, capped at 16 bits
// (driver header tile_sense_acp.h:232-233) — 49152 at the default ATIME 0x2F.
const alsMax = (s: State) => Math.min(65535, 1024 * ((s.atime & 0xff) + 1));

// ALS count for a channel: base scaled by gain (relative to 16x) and by
// integration time (linear in ATIME), saturating at the ATIME full scale.
function alsCount(base: number, s: State): number {
  if (!s.als_on) return 0;
  const gain = ALS_GAIN_MUL[s.als_gain & 3] / ALS_GAIN_REF;
  const time = (s.atime + 1) / (ATIME_REF + 1);
  return clamp(Math.round(base * gain * time), 0, alsMax(s));
}

// Proximity count: base scaled by proximity gain (relative to 4x) and LED
// drive (relative to 12 mA), saturating at the 8-bit range.
function proxCount(s: State): number {
  if (!s.prox_on) return 0;
  const gain = PROX_GAIN_MUL[s.prox_gain & 3] / PROX_GAIN_REF;
  const drive = s.prox_drive_ma / PROX_DRIVE_REF;
  return clamp(Math.round(s.proximity * gain * drive), 0, 255);
}

function statusByte(s: State): number {
  let status = 0;
  const max = alsMax(s);
  const alsSat =
    s.als_on === 1 &&
    (alsCount(s.clear, s) >= max ||
      alsCount(s.red, s) >= max ||
      alsCount(s.green, s) >= max ||
      alsCount(s.blue, s) >= max);
  if (alsSat) status |= ST_ASAT;
  if (s.prox_on === 1 && proxCount(s) >= 255) status |= ST_PSAT;
  return status;
}

// PLDRIVE quantises to 6 mA steps: i_LED = 6·(PLDRIVE+1), clamped 6..192.
function quantiseDriveMa(ma: number): number {
  const steps = clamp(Math.floor(clamp(ma, 6, 192) / 6), 1, 32);
  return steps * 6;
}

const sim: TileSim<State> = {
  tile: 'Sense.ACP',

  defaultState: {
    // White-ish ambient: R/G/B ratios near the datasheet's 2700 K white-LED
    // figures (R/C ~60%, G/C ~35%, B/C ~22%).
    clear: 8000,
    red: 4800,
    green: 2800,
    blue: 1760,
    proximity: 30,

    als_gain: 2, // 16x
    atime: 0x2f, // ~133 ms
    prox_gain: 2, // 4x
    prox_drive_ma: 12,

    als_on: 1,
    prox_on: 1,
  },

  controls: [
    { type: 'slider', field: 'clear', label: 'Ambient (clear)', min: 0, max: 65535, step: 100 },
    { type: 'slider', field: 'red', label: 'Red', min: 0, max: 65535, step: 100 },
    { type: 'slider', field: 'green', label: 'Green', min: 0, max: 65535, step: 100 },
    { type: 'slider', field: 'blue', label: 'Blue', min: 0, max: 65535, step: 100 },
    {
      type: 'slider',
      field: 'proximity',
      label: 'Proximity (near→high)',
      min: 0,
      max: 255,
      step: 1,
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    // 1 when the ID register matches (tile_sense_acp.c:82), not the address
    tile_sense_acp_find: () => ({ scalar: 1 }),
    tile_sense_acp_init: () => ({
      nextState: {
        als_gain: 2,
        atime: 0x2f,
        prox_gain: 2,
        prox_drive_ma: 12,
        als_on: 1,
        prox_on: 1,
      },
    }),
    tile_sense_acp_sleep: () => ({ nextState: { als_on: 0, prox_on: 0 } }),
    tile_sense_acp_wake: () => ({ nextState: { als_on: 1, prox_on: 1 } }),

    // ── configuration ──
    tile_sense_acp_set_als_gain: ({ state, args }) => ({
      nextState: { als_gain: pick(args, 0, state.als_gain) & 3 },
    }),
    tile_sense_acp_set_integration_time: ({ state, args }) => ({
      nextState: { atime: pick(args, 0, state.atime) & 0xff },
    }),
    tile_sense_acp_set_prox_gain: ({ state, args }) => ({
      nextState: { prox_gain: pick(args, 0, state.prox_gain) & 3 },
    }),
    tile_sense_acp_set_prox_drive_ma: ({ state, args }) => ({
      nextState: { prox_drive_ma: quantiseDriveMa(pick(args, 0, state.prox_drive_ma)) },
    }),

    // ── ambient light / colour ──
    tile_sense_acp_get_clear: ({ state }) => ({ scalar: alsCount(state.clear, state) }),
    tile_sense_acp_get_red: ({ state }) => ({ scalar: alsCount(state.red, state) }),
    tile_sense_acp_get_green: ({ state }) => ({ scalar: alsCount(state.green, state) }),
    tile_sense_acp_get_blue: ({ state }) => ({ scalar: alsCount(state.blue, state) }),

    // ── proximity ──
    tile_sense_acp_get_proximity: ({ state }) => ({ scalar: proxCount(state) }),

    // ── status ──
    tile_sense_acp_get_status: ({ state }) => ({ scalar: statusByte(state) }),
  },

  provenance: {
    tile_sense_acp_find: 'canonical', // I2C 0x39, ID reg 0xE4
    tile_sense_acp_set_als_gain: 'canonical', // CFG1 AGAIN
    tile_sense_acp_set_integration_time: 'canonical', // ATIME
    tile_sense_acp_set_prox_gain: 'canonical', // PCFG1 PGAIN
    tile_sense_acp_set_prox_drive_ma: 'canonical', // PCFG1 PLDRIVE, 6 mA steps
    // getters return modelled counts (gain/integration scaling is
    // datasheet-directional, but the base light is a control, not a
    // photodiode model)
    tile_sense_acp_get_clear: 'inferred',
    tile_sense_acp_get_red: 'inferred',
    tile_sense_acp_get_green: 'inferred',
    tile_sense_acp_get_blue: 'inferred',
    tile_sense_acp_get_proximity: 'inferred',
    tile_sense_acp_get_status: 'inferred', // modelled saturation flags
    power: 'inferred', // estimated supply + boosted LED average
    // (init/sleep/wake default to "inferred")
  },

  // Supply on the 1.8 V input (pad 10). Sleep is near-zero; when running,
  // the ALS front-end plus the proximity analog path draw quiescent current,
  // and the proximity IR LED adds a boosted average on top (TPS61099 stepping
  // 1.8 V up to the LED rail). All figures are estimates → provenance inferred.
  power(state) {
    const running = state.als_on === 1 || state.prox_on === 1;
    if (!running) {
      return {
        draw_ua: 3,
        rails: [{ name: '1V8', role: 'supply', v_mv: 1800, i_ua: 3, pads: ['10'], note: 'sleep' }],
      };
    }
    const oscillator = 10; // PON oscillator/ADC housekeeping
    const alsPart = state.als_on ? 240 : 0; // ALS/colour front-end
    const proxSensor = state.prox_on ? 90 : 0; // proximity analog path
    // Boosted average LED input: ~4.5% pulse duty, boost ratio ~2.5, ~85%
    // efficiency → ~130 µA of 1.8 V input per mA of programmed LED drive.
    const proxLed = state.prox_on ? Math.round(state.prox_drive_ma * 130) : 0;
    const draw = oscillator + alsPart + proxSensor + proxLed;
    return {
      draw_ua: draw,
      rails: [
        {
          name: '1V8',
          role: 'supply',
          v_mv: 1800,
          i_ua: draw,
          pads: ['10'],
          note: state.prox_on ? 'active (incl. boosted prox LED avg)' : 'ALS active',
        },
      ],
    };
  },
};

export default sim;
