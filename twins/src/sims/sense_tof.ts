// Digital twin for Sense.TOF — ams-OSRAM TMF8806 direct time-of-flight ranging
// sensor (driver tile_sense_tof.{h,c}; TMF8806 DS001097 v4 + HostDriverCommunication
// AN001069 v2).
//
// The physical world (object distance, return strength, die temperature, an
// injected error status) is driven by the controls; host calls return the result
// record exactly as the firmware reads it (distance in mm, status, reliability
// 0..63, temperature, the 8-bit result counter). The program's calls (start /
// stop / sleep / set_period / set_kilo_iters / set_distance_mode) land in the same
// fields `power()` reads, so the supply current follows the program.
//
// Pads (Sense-TOF-a.json): INT is pad 9 (output), V+ pad 10, GND pad 1. Pad 3 is
// EN, an INPUT pulled up on-board (sensor enabled by default; ground it to
// disable) — not driven by the twin, and holding it low is not modeled.
import type { TileSim } from '../tileSim';

interface State {
  // ── physical world (controls) ──
  /** Object distance from the sensor, mm. */
  distance_mm: number;
  /** Return strength / confidence, 0..63 (RESULT_INFO[5:0]). */
  reliability: number;
  /** Die temperature, °C (register 0x33, signed 8-bit). */
  temperature_c: number;
  /** Fault injection: 0 = valid result (status 0x00), 1 = error status 0x10 with
   * distance 0 (datasheet: 0x10+ = error). */
  fault_inject: number;

  // ── operation (what the driver's calls set) ──
  /** Continuous ranging active: `start` sets, `stop` / `sleep` / `reset` clear. */
  measuring: number;
  /** PON cleared by `sleep` — standby until `wake`. */
  sleeping: number;
  /** RESULT_NUMBER register: bumped once per completed measurement, wraps 256. */
  result_number: number;
  /** `t` of the last modeled measurement (deriveState's clock). */
  last_result_t: number;

  // ── ranging configuration (the driver's cfg) ──
  /** sense_tof_distance_mode_t: 0 short (~200 mm), 1 2500 mm, 2 5000 mm. */
  distance_mode: number;
  /** Repetition period code: 0 single-shot, 1-253 ms, 0xFE 1 s, 0xFF 2 s. */
  period_ms: number;
  /** Iterations per ranging, in thousands (SNR / power trade-off). */
  kilo_iters: number;
  /** Object-detection threshold, 0..63 (cmd_data3[5:0]; 0 → 6 internally). */
  threshold: number;

  // ── threshold-interrupt window (App0 cmd 0x08 / 0x09) ──
  /** 0 = INT on every measurement; n = n consecutive in-window results. */
  threshold_persistence: number;
  threshold_low_mm: number; // inclusive; low > high → no object can be reported
  threshold_high_mm: number; // inclusive
  /** Consecutive in-window measurements so far (compared against persistence). */
  in_band_streak: number;

  // ── calibration / algorithm-state blobs ──
  /** Driver's calib_valid: set by factory_calibrate / set_calibration. */
  calib_valid: number;
  /** 14- and 11-byte blobs as base64 (state holds only scalars/strings). */
  calib_data_b64: string;
  state_data_b64: string;
}

/** SENSE_TOF_PRESENCE_RELIABILITY_MIN — the presence helpers' cutoff. */
const PRESENCE_RELIABILITY_MIN = 32;
/** The chip substitutes 6 for a detection threshold of 0 (datasheet §6.9). */
const DEFAULT_DETECTION_THRESHOLD = 6;
/** App0 revision (APPREV_MAJOR/MINOR/PATCH) — not the driver version. */
const APP_VERSION = [1, 2, 0];
/** Serial from cmd 0x47: a recognizable non-zero pattern, since the flat variant
 * signals an error with all zeros. */
const DUMMY_SERIAL = [0xab, 0xcd, 0xef, 0x42];

// ── blobs ──
function bytesToB64(bytes: readonly number[]): string {
  let bin = '';
  for (const b of bytes) bin += String.fromCharCode(b & 0xff);
  return typeof btoa === 'function' ? btoa(bin) : '';
}
function b64ToBytes(s: string, len: number): number[] {
  const bin = s && typeof atob === 'function' ? atob(s) : '';
  const out: number[] = [];
  for (let i = 0; i < len; i++) out.push(i < bin.length ? bin.charCodeAt(i) & 0xff : 0);
  return out;
}

// ── the result record ──

/** The furthest the active distance mode reaches — mirrors
 * `tile_sense_tof_max_range_mm` (200 / 2500 / 5000 mm). */
export function maxRangeMm(distanceMode: number): number {
  return distanceMode === 0 ? 200 : distanceMode === 2 ? 5000 : 2500;
}

/** What the SENSOR reports as distance: 0 means "no object" — beyond the mode's
 * reach, a return weaker than the detection threshold, or an error status. This
 * is what get_result exposes, unchanged. */
function rawDistance(s: State): number {
  if (s.fault_inject) return 0;
  if (s.distance_mm > maxRangeMm(s.distance_mode)) return 0;
  if (s.reliability < (s.threshold || DEFAULT_DETECTION_THRESHOLD)) return 0;
  return s.distance_mm;
}
/** No object → reliability 0 too. */
function rawReliability(s: State): number {
  return rawDistance(s) > 0 ? s.reliability & 0x3f : 0;
}
function status(s: State): number {
  return s.fault_inject ? 0x10 : 0x00;
}
/** `tile_sense_tof_get_distance_mm`: 0 SATURATES to the mode's max range so
 * `distance < threshold` stays quiet on an empty room. That is the ONLY
 * transformation — no reliability gate (tile_sense_tof.c get_distance_mm). */
function saturatedDistance(s: State): number {
  return rawDistance(s) || maxRangeMm(s.distance_mode);
}
/** The presence question (is_object_within): demands confidence ≥ 32 and a real
 * target. Uses the raw distance — the saturated value is never 0. */
function objectWithin(s: State, mm: number): boolean {
  const d = rawDistance(s);
  return rawReliability(s) >= PRESENCE_RELIABILITY_MIN && d > 0 && d <= mm;
}
/** In the threshold-interrupt window (HostDriverComm §8.12): needs an object. */
function inWindow(s: State): boolean {
  if (s.threshold_low_mm > s.threshold_high_mm) return false;
  const d = rawDistance(s);
  return d > 0 && d >= s.threshold_low_mm && d <= s.threshold_high_mm;
}
/** One single-shot measurement (measure_single and the helpers built on it):
 * a fresh result number. */
const nextResult = (s: State) => (s.result_number + 1) & 0xff;
const arg = (args: number[], i: number, fallback: number) =>
  args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;

// ── Supply current (TMF8806 DS001097 v4, "Current consumption", 2.8 V, 23 °C) ──
//
// Ranging is a duty cycle (datasheet §6.8): each period the chip ranges for
// kIters iterations — one per VCSEL clock — then WAITs out the rest of the
// period. If ranging outlasts the period the chip ranges back-to-back.
//
//   I_avg = (t_rng · I_rng + (P − t_rng) · I_wait) / P,  t_rng = kIters·1000 / f_vcsel
//
// Checked against the datasheet's I_RANGING_AVG rows:
//   2.5 m, 30 ms, 900k → 25.8 mA  (table 25.8)    2.5 m, 30 ms, 450k → 13.1 (13.5)
//   5 m,   30 ms, 450k → 21.5 mA  (table 20.4)    5 m,   60 ms, 900k → 21.6 (22.9)
// (the table's "33 ms" / "66 ms" are the 0x1E / 0x3C period codes: with 30 ms the
// 2.5 m row reproduces I_ACTIVE_RANGING's 32.5 mA, with 33 ms it would need 35.6).
// Not modeled: the datasheet's ultra-low-power mode (46 µA at 1 s) — the driver
// doesn't use it, so between rangings the chip sits in WAIT at 140 µA.
const I_RANGING_UA = 32500; // I_ACTIVE_RANGING, vcselClkDiv2=0 (short + 2.5 m)
// 5 m mode runs the VCSEL at half clock (vcselClkDiv2=1). The table gives no
// ranging current for it; 27 mA is the value that fits both 5 m rows (±6 %).
const I_RANGING_DIV2_UA = 27000;
const I_WAIT_UA = 140; // I_WAIT: CPU off, oscillator on, timer wakeup
const I_STANDBY_UA = 85; // I_STANDBY: PON=0 (sleep)
const VCSEL_HZ = 37.6e6; // vcselClkDiv2=0; 18.8 MHz with div2

/** Repetition period in ms for a cmd_data2 code (0 = single shot → no period). */
export function periodMs(code: number): number {
  if (code === 0xfe) return 1000;
  if (code === 0xff) return 2000;
  return code;
}

/** How long one ranging takes, ms: one iteration per VCSEL clock. */
export function rangingMs(distanceMode: number, kiloIters: number): number {
  const hz = distanceMode === 2 ? VCSEL_HZ / 2 : VCSEL_HZ;
  return (kiloIters * 1000 * 1000) / hz;
}

/** Average supply current, µA. */
export function averageCurrentUa(s: State): number {
  if (s.sleeping === 1) return I_STANDBY_UA;
  const period = periodMs(s.period_ms);
  // Stopped, or single-shot (each measure_single is a one-off the program paces):
  // awake and waiting.
  if (s.measuring !== 1 || period === 0) return I_WAIT_UA;
  const iRng = s.distance_mode === 2 ? I_RANGING_DIV2_UA : I_RANGING_UA;
  const tRng = rangingMs(s.distance_mode, s.kilo_iters);
  if (tRng >= period) return iRng; // back-to-back: the period is ignored
  return Math.round((tRng * iRng + (period - tRng) * I_WAIT_UA) / period);
}

function currentNote(s: State): string {
  if (s.sleeping === 1) return 'standby (sleep)';
  const period = periodMs(s.period_ms);
  if (s.measuring !== 1 || period === 0) return 'idle, waiting for a command';
  const tRng = rangingMs(s.distance_mode, s.kilo_iters);
  if (tRng >= period) return `ranging back-to-back (${tRng.toFixed(0)} ms each)`;
  return `ranging ${tRng.toFixed(1)} ms every ${period} ms`;
}

const sim: TileSim<State> = {
  tile: 'Sense.TOF',

  // State AFTER init: App0 booted, result interrupt enabled, cfg defaults
  // (tof_apply_defaults: 2500 mm mode, 0x1E period, 900k iterations, threshold 6).
  // Init does not start ranging — the program calls start().
  defaultState: {
    distance_mm: 500,
    reliability: 50,
    temperature_c: 25,
    fault_inject: 0,

    measuring: 0,
    sleeping: 0,
    result_number: 0,
    last_result_t: 0,

    distance_mode: 1,
    period_ms: 0x1e,
    kilo_iters: 900,
    threshold: 6,

    threshold_persistence: 0,
    threshold_low_mm: 0,
    threshold_high_mm: 0,
    in_band_streak: 0,

    calib_valid: 0,
    calib_data_b64: '',
    state_data_b64: '',
  },

  controls: [
    {
      type: 'slider',
      field: 'distance_mm',
      label: 'Object distance',
      min: 0,
      max: 5000,
      step: 10,
      unit: 'mm',
      description: "Beyond the distance mode's reach the sensor reports no object.",
    },
    {
      type: 'slider',
      field: 'reliability',
      label: 'Return strength',
      min: 0,
      max: 63,
      step: 1,
      description:
        'Reliability 0–63. Below the detection threshold (6) no object is reported; the presence helpers need 32.',
    },
    {
      type: 'slider',
      field: 'temperature_c',
      label: 'Die temperature',
      min: -40,
      max: 85,
      step: 1,
      unit: '°C',
    },
    {
      type: 'toggle',
      field: 'fault_inject',
      label: 'Error status',
      description: 'Report status 0x10 (an error) with distance 0.',
    },
  ],

  // Where the object is. The slider spans the distance range the program chose,
  // so all of it is somewhere the sensor sees.
  stimuli: [
    {
      id: 'distance',
      label: 'Object distance',
      controls: [
        {
          kind: 'slider',
          id: 'distance_mm',
          label: 'distance',
          field: 'distance_mm',
          min: 0,
          max: 2500,
          step: 10,
          unit: 'mm',
          maxBy: { field: 'distance_mode', values: { '0': 200, '1': 2500, '2': 5000 } },
        },
      ],
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_sense_tof_find: () => ({ scalar: 1 }),
    tile_sense_tof_init: () => ({ scalar: 0 }),
    // stop, then PON=0.
    tile_sense_tof_sleep: () => ({ nextState: { measuring: 0, sleeping: 1, in_band_streak: 0 } }),
    // Re-runs the boot sequence; ranging stays stopped until start().
    tile_sense_tof_wake: () => ({ nextState: { sleeping: 0 } }),
    // CPU reset + memzero of the driver's state: cfg, measuring and calibration
    // all go to 0; the chip's App0 config (threshold window) is lost too.
    tile_sense_tof_reset: () => ({
      nextState: {
        measuring: 0,
        sleeping: 0,
        result_number: 0,
        distance_mode: 0,
        period_ms: 0,
        kilo_iters: 0,
        threshold: 0,
        threshold_persistence: 0,
        threshold_low_mm: 0,
        threshold_high_mm: 0,
        in_band_streak: 0,
        calib_valid: 0,
        calib_data_b64: '',
        state_data_b64: '',
      },
    }),
    tile_sense_tof_start: () => ({ nextState: { measuring: 1, sleeping: 0 } }),
    tile_sense_tof_stop: () => ({ nextState: { measuring: 0, in_band_streak: 0 } }),

    // ── results ──
    tile_sense_tof_get_distance_mm: ({ state }) => ({ scalar: saturatedDistance(state) }),
    tile_sense_tof_max_range_mm: ({ state }) => ({ scalar: maxRangeMm(state.distance_mode) }),
    // int32[5]: [distance_mm, status, reliability, temperature, result_number] —
    // the latest result, raw (no saturation). Reading does not measure.
    tile_sense_tof_get_result_flat: ({ state }) => ({
      array: [
        rawDistance(state),
        status(state),
        rawReliability(state),
        Math.trunc(state.temperature_c),
        state.result_number,
      ],
    }),
    // One single-shot measurement; returns 1 once the result interrupt fires
    // (an error STATUS is still a completed measurement). The real call blocks
    // ~30-200 ms; the twin answers at once.
    tile_sense_tof_measure_single_flat: ({ state }) => {
      const seq = nextResult(state);
      return {
        scalar: 1,
        outScalars: {
          mm: rawDistance(state),
          status: status(state),
          reliability: rawReliability(state),
          temp_c: Math.trunc(state.temperature_c),
          seq,
        },
        nextState: { result_number: seq },
      };
    },
    // INT_STATUS result bit: set by every completed measurement, object or not.
    tile_sense_tof_result_ready: ({ state }) => ({
      scalar: state.measuring === 1 && state.sleeping !== 1 ? 1 : 0,
    }),

    // ── presence helpers (single-shot, reliability ≥ 32) ──
    tile_sense_tof_is_object_within: ({ state, args }) => ({
      scalar: objectWithin(state, arg(args, 0, 0)) ? 1 : 0,
      nextState: { result_number: nextResult(state) },
    }),
    // Polls is_object_within until timeout. The world doesn't move during the
    // call, so one evaluation answers it.
    tile_sense_tof_wait_for_object: ({ state, args }) => ({
      scalar: objectWithin(state, arg(args, 0, 0)) ? 1 : 0,
      nextState: { result_number: nextResult(state) },
    }),
    // Raw distance (no saturation) + reliability*100/63, integer.
    tile_sense_tof_read_distance_with_confidence: ({ state }) => ({
      scalar: 1,
      outScalars: {
        mm: rawDistance(state),
        confidence_pct: Math.floor((rawReliability(state) * 100) / 63),
      },
      nextState: { result_number: nextResult(state) },
    }),

    // ── threshold interrupt ──
    // The chip accepts any window (low > high just means nothing is reported);
    // the driver returns whether App0 echoed the command.
    tile_sense_tof_set_threshold_interrupt: ({ args }) => ({
      scalar: 1,
      nextState: {
        threshold_persistence: arg(args, 0, 0) & 0xff,
        threshold_low_mm: arg(args, 1, 0) & 0xffff,
        threshold_high_mm: arg(args, 2, 0) & 0xffff,
        in_band_streak: 0,
      },
    }),
    tile_sense_tof_get_threshold_interrupt: ({ state }) => ({
      scalar: 1,
      outScalars: {
        persistence: state.threshold_persistence,
        low_mm: state.threshold_low_mm,
        high_mm: state.threshold_high_mm,
      },
    }),

    // ── configuration (the driver stops/restarts ranging around each) ──
    tile_sense_tof_set_distance_mode: ({ state, args }) => ({
      nextState: { distance_mode: arg(args, 0, state.distance_mode) & 0xff },
    }),
    tile_sense_tof_set_period: ({ state, args }) => ({
      nextState: { period_ms: arg(args, 0, state.period_ms) & 0xff },
    }),
    tile_sense_tof_set_kilo_iters: ({ state, args }) => ({
      nextState: { kilo_iters: arg(args, 0, state.kilo_iters) & 0xffff },
    }),
    tile_sense_tof_set_threshold: ({ state, args }) => ({
      nextState: { threshold: arg(args, 0, state.threshold) & 0x3f },
    }),

    // ── calibration / algorithm state ──
    // The real cycle runs ~40M iterations; the twin reports success at once.
    tile_sense_tof_factory_calibrate: () => ({ scalar: 1, nextState: { calib_valid: 1 } }),
    tile_sense_tof_set_calibration: ({ bufferIn }) => {
      const data = bufferIn?.data;
      if (!data) return;
      return { nextState: { calib_valid: 1, calib_data_b64: bytesToB64(data) } };
    },
    // Echoes the last blob loaded (14 zeros if none).
    tile_sense_tof_get_calibration: ({ state }) => ({
      array: b64ToBytes(state.calib_data_b64, 14),
    }),
    // The chip's 11 bytes of accumulators: echoes the last restore (zeros if none).
    tile_sense_tof_save_state: ({ state }) => ({ array: b64ToBytes(state.state_data_b64, 11) }),
    tile_sense_tof_restore_state: ({ bufferIn }) => {
      const data = bufferIn?.data;
      if (!data) return;
      return { nextState: { state_data_b64: bytesToB64(data) } };
    },

    // ── identity / diagnostics ──
    tile_sense_tof_get_app_version_flat: () => ({
      outScalars: { major: APP_VERSION[0], minor: APP_VERSION[1], patch: APP_VERSION[2] },
    }),
    tile_sense_tof_get_serial_number_flat: () => ({ array: DUMMY_SERIAL }),
    // The chip's SYS_CLOCK counter: a stand-in ramp tied to the result counter.
    tile_sense_tof_get_sys_clock_ticks: ({ state }) => ({
      scalar: (state.result_number * 4700) >>> 0,
    }),
    // Zero-filled is the flat variant's documented failure result.
    tile_sense_tof_read_histogram_flat: () => ({ array: new Array(128).fill(0) }),
  },

  provenance: {
    // canonical — datasheet result layout / conversions
    tile_sense_tof_find: 'canonical', // ID register check at 0x41
    tile_sense_tof_get_distance_mm: 'canonical', // MSB<<8 | LSB (mm), 0 saturates to max range
    tile_sense_tof_max_range_mm: 'canonical', // 200 / 2500 / 5000 by mode
    tile_sense_tof_get_result_flat: 'canonical', // result record layout
    tile_sense_tof_measure_single_flat: 'canonical',
    tile_sense_tof_read_distance_with_confidence: 'canonical', // reliability*100/63
    tile_sense_tof_is_object_within: 'canonical', // driver: reliability ≥ 32, distance > 0, ≤ mm
    tile_sense_tof_wait_for_object: 'inferred', // one evaluation stands in for the poll loop
    tile_sense_tof_set_distance_mode: 'canonical', // algo byte / ranges
    tile_sense_tof_set_period: 'canonical', // repetition period code
    tile_sense_tof_set_kilo_iters: 'canonical', // cmd_data1/0 kIters
    tile_sense_tof_set_threshold: 'canonical', // cmd_data3[5:0]
    tile_sense_tof_set_threshold_interrupt: 'canonical', // WR_ADD_CONFIG, §8.12
    tile_sense_tof_get_threshold_interrupt: 'canonical', // RD_ADD_CONFIG readback
    // inferred — behavioral simplifications
    tile_sense_tof_result_ready: 'inferred', // "measuring and awake", not a per-period INT_STATUS bit
    tile_sense_tof_get_sys_clock_ticks: 'inferred', // fabricated ramp, not the real 4.7 MHz counter
    tile_sense_tof_factory_calibrate: 'inferred',
    tile_sense_tof_get_app_version_flat: 'inferred',
    // hallucinated — stubbed / opaque
    tile_sense_tof_read_histogram_flat: 'hallucinated',
    tile_sense_tof_get_serial_number_flat: 'hallucinated',
    tile_sense_tof_get_calibration: 'hallucinated',
    tile_sense_tof_set_calibration: 'hallucinated',
    tile_sense_tof_save_state: 'hallucinated',
    tile_sense_tof_restore_state: 'hallucinated',
    power: 'inferred', // duty-cycle model over datasheet currents; matches the I_RANGING_AVG rows within 6 %, 5 m ranging current fitted
  },

  // One measurement per repetition period while ranging: bump the result counter
  // and count consecutive in-window results for the threshold interrupt's
  // persistence (HostDriverComm §8.12).
  deriveState(state, { t }) {
    const period = periodMs(state.period_ms);
    if (state.measuring !== 1 || state.sleeping === 1 || period === 0) return {};
    if (t - state.last_result_t < period) return {};
    return {
      last_result_t: t,
      result_number: nextResult(state),
      in_band_streak: inWindow(state) ? state.in_band_streak + 1 : 0,
    };
  },

  // INT (pad 9; active-low on hardware, 1 = asserted here). Persistence 0: a
  // result interrupt every measurement, object or not. Persistence n: only once
  // n consecutive results fall inside [low, high], then every in-window period.
  padOutputs(state) {
    if (state.sleeping === 1 || state.measuring !== 1) return { '9': 0 };
    if (state.threshold_persistence === 0) return { '9': 1 };
    return {
      '9': inWindow(state) && state.in_band_streak >= state.threshold_persistence ? 1 : 0,
    };
  },

  // Electrical: average supply current from the ranging duty cycle — see
  // `averageCurrentUa`. The chip's VCSEL pulses peak ~230 mA internally; what the
  // rail sees is the average.
  power(state) {
    const ua = averageCurrentUa(state);
    return {
      draw_ua: ua,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: 2800,
          i_ua: ua,
          pads: ['10'],
          note: currentNote(state),
        },
      ],
    };
  },
};

export default sim;
