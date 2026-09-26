// src/sims/sense_tof.ts
var PRESENCE_RELIABILITY_MIN = 32;
var DEFAULT_DETECTION_THRESHOLD = 6;
var APP_VERSION = [4, 14, 0];
var DUMMY_SERIAL = [171, 205, 239, 66];
var SYS_CLOCK_PER_MS = 4700;
function calibBlob(mode) {
  const b = [2];
  for (let i = 1; i < 14; i++) b.push(53 * i + 17 * mode + 90 & 255);
  return b;
}
function algStateBlob(s) {
  const b = [];
  for (let i = 0; i < 11; i++) b.push(s.result_number * 7 + i * 29 + s.distance_mode & 255);
  return b;
}
function referenceHistogram(s) {
  const peakBin = 8;
  const height = Math.min(65535, Math.round(s.kilo_iters * 20));
  const out = [];
  for (let bin = 0; bin < 64; bin++) {
    const d = bin - peakBin;
    const v = Math.min(65535, Math.round(height * Math.exp(-(d * d) / 4)) + 12 + bin * 37 % 9);
    out.push(v & 255, v >> 8);
  }
  return out;
}
function bytesToB64(bytes) {
  let bin = "";
  for (const b of bytes) bin += String.fromCharCode(b & 255);
  return typeof btoa === "function" ? btoa(bin) : "";
}
function b64ToBytes(s, len) {
  const bin = s && typeof atob === "function" ? atob(s) : "";
  const out = [];
  for (let i = 0; i < len; i++) out.push(i < bin.length ? bin.charCodeAt(i) & 255 : 0);
  return out;
}
function maxRangeMm(distanceMode) {
  return distanceMode === 0 ? 200 : distanceMode === 2 ? 5e3 : 2500;
}
function rawDistance(s) {
  if (s.fault_inject) return 0;
  if (s.distance_mm > maxRangeMm(s.distance_mode)) return 0;
  if (s.reliability < (s.threshold || DEFAULT_DETECTION_THRESHOLD)) return 0;
  return s.distance_mm;
}
function rawReliability(s) {
  const d = rawDistance(s);
  if (d === 0) return 0;
  if (s.distance_mode === 0 || d < 200) return s.calib_valid ? 10 : 1;
  const r = s.reliability & 63;
  return r === 1 || r === 10 ? r + 1 : r;
}
var isDetection = (r) => r === 1 || r === 10 || r >= PRESENCE_RELIABILITY_MIN;
function status(s) {
  return s.fault_inject ? 16 : 0;
}
function saturatedDistance(s) {
  return rawDistance(s) || maxRangeMm(s.distance_mode);
}
function objectWithin(s, mm) {
  const d = rawDistance(s);
  return isDetection(rawReliability(s)) && d > 0 && d <= mm;
}
var confidencePct = (r) => r === 10 ? 100 : r === 1 ? 50 : Math.floor(r * 100 / 63);
function inWindow(s) {
  if (s.threshold_low_mm > s.threshold_high_mm) return false;
  const d = rawDistance(s);
  return d > 0 && d >= s.threshold_low_mm && d <= s.threshold_high_mm;
}
function signalQuality(s) {
  const crosstalk = s.fault_inject ? 0 : 480;
  if (rawDistance(s) === 0) return { reference_hits: 0, object_hits: 0, crosstalk };
  const iterScale = s.kilo_iters / 900;
  return {
    reference_hits: Math.round(18e4 * iterScale),
    object_hits: Math.round(rawReliability(s) / 63 * 15e4 * iterScale),
    crosstalk
  };
}
var nextResult = (s) => s.result_number + 1 & 255;
var arg = (args, i, fallback) => args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;
var I_RANGING_UA = 32500;
var I_RANGING_DIV2_UA = 27e3;
var I_WAIT_UA = 140;
var I_STANDBY_UA = 85;
var VCSEL_HZ = 376e5;
function periodMs(code) {
  if (code === 254) return 1e3;
  if (code === 255) return 2e3;
  return code;
}
function rangingMs(distanceMode, kiloIters) {
  const hz = distanceMode === 2 ? VCSEL_HZ / 2 : VCSEL_HZ;
  return kiloIters * 1e3 * 1e3 / hz;
}
function averageCurrentUa(s) {
  if (s.sleeping === 1) return I_STANDBY_UA;
  const period = periodMs(s.period_ms);
  if (s.measuring !== 1 || period === 0) return I_WAIT_UA;
  const iRng = s.distance_mode === 2 ? I_RANGING_DIV2_UA : I_RANGING_UA;
  const tRng = rangingMs(s.distance_mode, s.kilo_iters);
  if (tRng >= period) return iRng;
  return Math.round((tRng * iRng + (period - tRng) * I_WAIT_UA) / period);
}
function currentNote(s) {
  if (s.sleeping === 1) return "standby (sleep)";
  const period = periodMs(s.period_ms);
  if (s.measuring !== 1 || period === 0) return "idle, waiting for a command";
  const tRng = rangingMs(s.distance_mode, s.kilo_iters);
  if (tRng >= period) return `ranging back-to-back (${tRng.toFixed(0)} ms each)`;
  return `ranging ${tRng.toFixed(1)} ms every ${period} ms`;
}
var sim = {
  tile: "Sense.TOF",
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
    period_ms: 30,
    kilo_iters: 900,
    threshold: 6,
    threshold_persistence: 0,
    threshold_low_mm: 0,
    threshold_high_mm: 0,
    in_band_streak: 0,
    calib_valid: 0,
    calib_data_b64: "",
    state_data_b64: "",
    chip_state_b64: ""
  },
  controls: [
    {
      type: "slider",
      field: "distance_mm",
      label: "Object distance",
      min: 0,
      max: 5e3,
      step: 10,
      unit: "mm",
      description: "Beyond the distance mode's reach the sensor reports no object."
    },
    {
      type: "slider",
      field: "reliability",
      label: "Return strength",
      min: 0,
      max: 63,
      step: 1,
      description: "Reliability 0\u201363. Below the detection threshold (6) no object is reported; the presence helpers need 32."
    },
    {
      type: "slider",
      field: "temperature_c",
      label: "Die temperature",
      min: -40,
      max: 85,
      step: 1,
      unit: "\xB0C"
    },
    {
      type: "toggle",
      field: "fault_inject",
      label: "Error status",
      description: "Report status 0x10 (an error) with distance 0."
    }
  ],
  // Where the object is. The slider spans the distance range the program chose,
  // so all of it is somewhere the sensor sees.
  stimuli: [
    {
      id: "distance",
      label: "Object distance",
      controls: [
        {
          kind: "slider",
          id: "distance_mm",
          label: "distance",
          field: "distance_mm",
          min: 0,
          max: 2500,
          step: 10,
          unit: "mm",
          maxBy: { field: "distance_mode", values: { "0": 200, "1": 2500, "2": 5e3 } }
        }
      ]
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    tile_sense_tof_find: () => ({ scalar: 1 }),
    tile_sense_tof_init: () => ({ scalar: 0 }),
    // stop, then PON=0.
    tile_sense_tof_sleep: () => ({ nextState: { measuring: 0, sleeping: 1, in_band_streak: 0 } }),
    // Re-runs the boot sequence; ranging stays stopped until start().
    tile_sense_tof_wake: () => ({ nextState: { sleeping: 0 } }),
    // CPU reset, then the driver boots App0 again: ranging stops, calibration
    // and algorithm state are dropped, the measurement config is kept; the
    // chip's App0 config (threshold window) is lost.
    tile_sense_tof_reset: () => ({
      nextState: {
        measuring: 0,
        sleeping: 0,
        result_number: 0,
        threshold_persistence: 0,
        threshold_low_mm: 0,
        threshold_high_mm: 0,
        in_band_streak: 0,
        calib_valid: 0,
        calib_data_b64: "",
        state_data_b64: "",
        chip_state_b64: "",
        last_result_t: 0
      }
    }),
    // Loads a restored algorithm state (only with valid calibration: algState
    // needs factoryCal) and consumes it. Period 0 is one measurement, not
    // continuous ranging.
    tile_sense_tof_start: ({ state }) => {
      const next = { sleeping: 0, state_data_b64: "" };
      if (state.state_data_b64 && state.calib_valid) next.chip_state_b64 = state.state_data_b64;
      if (periodMs(state.period_ms) === 0) {
        next.measuring = 0;
        next.result_number = nextResult(state);
      } else {
        next.measuring = 1;
      }
      return { nextState: next };
    },
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
        state.result_number
      ]
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
          seq
        },
        nextState: { result_number: seq }
      };
    },
    // INT_STATUS result bit: set by every completed measurement, object or not.
    tile_sense_tof_result_ready: ({ state }) => ({
      scalar: state.measuring === 1 && state.sleeping !== 1 ? 1 : 0
    }),
    // ── presence helpers (single-shot, reliability ≥ 32) ──
    tile_sense_tof_is_object_within: ({ state, args }) => ({
      scalar: objectWithin(state, arg(args, 0, 0)) ? 1 : 0,
      nextState: { result_number: nextResult(state) }
    }),
    // Polls is_object_within until timeout. The world doesn't move during the
    // call, so one evaluation answers it.
    tile_sense_tof_wait_for_object: ({ state, args }) => ({
      scalar: objectWithin(state, arg(args, 0, 0)) ? 1 : 0,
      nextState: { result_number: nextResult(state) }
    }),
    // Raw distance (no saturation) + reliability*100/63, integer.
    tile_sense_tof_read_distance_with_confidence: ({ state }) => ({
      scalar: 1,
      outScalars: {
        mm: rawDistance(state),
        confidence_pct: confidencePct(rawReliability(state))
      },
      nextState: { result_number: nextResult(state) }
    }),
    // Latest result block's hit counts; reading does not measure.
    tile_sense_tof_get_signal_quality_flat: ({ state }) => ({
      outScalars: signalQuality(state)
    }),
    // ── threshold interrupt ──
    // The chip accepts any window (low > high just means nothing is reported);
    // the driver returns whether App0 echoed the command.
    tile_sense_tof_set_threshold_interrupt: ({ args }) => ({
      scalar: 1,
      nextState: {
        threshold_persistence: arg(args, 0, 0) & 255,
        threshold_low_mm: arg(args, 1, 0) & 65535,
        threshold_high_mm: arg(args, 2, 0) & 65535,
        in_band_streak: 0
      }
    }),
    tile_sense_tof_get_threshold_interrupt: ({ state }) => ({
      scalar: 1,
      outScalars: {
        persistence: state.threshold_persistence,
        low_mm: state.threshold_low_mm,
        high_mm: state.threshold_high_mm
      }
    }),
    // ── configuration (the driver stops/restarts ranging around each) ──
    // 5 m needs its own calibration (datasheet §6.4): crossing into or out of
    // it drops the one loaded.
    tile_sense_tof_set_distance_mode: ({ state, args }) => {
      const mode = arg(args, 0, state.distance_mode) & 255;
      const crosses = mode === 2 !== (state.distance_mode === 2);
      return { nextState: { distance_mode: mode, ...crosses ? { calib_valid: 0 } : {} } };
    },
    tile_sense_tof_set_period: ({ state, args }) => ({
      nextState: { period_ms: arg(args, 0, state.period_ms) & 255 }
    }),
    tile_sense_tof_set_kilo_iters: ({ state, args }) => ({
      nextState: { kilo_iters: arg(args, 0, state.kilo_iters) & 65535 }
    }),
    tile_sense_tof_set_threshold: ({ state, args }) => ({
      nextState: { threshold: arg(args, 0, state.threshold) & 63 }
    }),
    // ── calibration / algorithm state ──
    // The real cycle runs ~40M iterations; the twin reports success at once
    // with a blob in the chip's format (byte 0 = format revision 2, §7.4.1;
    // the other 13 bytes are opaque, so modeled). Calibrating drops any
    // algorithm state, as the driver does.
    tile_sense_tof_factory_calibrate: ({ state }) => ({
      scalar: 1,
      nextState: {
        calib_valid: 1,
        calib_data_b64: bytesToB64(calibBlob(state.distance_mode)),
        state_data_b64: ""
      }
    }),
    tile_sense_tof_set_calibration: ({ bufferIn }) => {
      const data = bufferIn?.data;
      if (!data) return;
      return { nextState: { calib_valid: 1, calib_data_b64: bytesToB64(data) } };
    },
    // The driver's copy: the blob while it is valid, zeros once it isn't
    // (reset, or crossing the 5 m boundary).
    tile_sense_tof_get_calibration: ({ state }) => ({
      array: state.calib_valid ? b64ToBytes(state.calib_data_b64, 14) : new Array(14).fill(0)
    }),
    // A live read of STATE_DATA (0x28-0x32): zeros (their reset value) until
    // the chip has ranged; then its algorithm state — a restored blob if
    // start() loaded one, otherwise accumulators the twin can only model.
    tile_sense_tof_save_state: ({ state }) => ({
      array: state.result_number === 0 ? new Array(11).fill(0) : state.chip_state_b64 ? b64ToBytes(state.chip_state_b64, 11) : algStateBlob(state)
    }),
    // The driver holds it and writes it at the next start().
    tile_sense_tof_restore_state: ({ bufferIn }) => {
      const data = bufferIn?.data;
      if (!data) return;
      return { nextState: { state_data_b64: bytesToB64(data.slice(0, 11)) } };
    },
    // ── identity / diagnostics ──
    tile_sense_tof_get_app_version_flat: () => ({
      outScalars: { major: APP_VERSION[0], minor: APP_VERSION[1], patch: APP_VERSION[2] }
    }),
    // SERIAL_NUMBER_0..3 (§7.6): a fixed, non-zero stand-in (zero is the
    // flat variant's failure value). Every real chip has its own.
    tile_sense_tof_get_serial_number_flat: () => ({ array: DUMMY_SERIAL }),
    // SYS_CLOCK (0x24-0x27, §7.3.5): the last result's time stamp in units of
    // 1/4.7 MHz, valid only with its LSB set — so 0 before any result.
    tile_sense_tof_get_sys_clock_ticks: ({ state }) => ({
      scalar: state.result_number === 0 && state.last_result_t === 0 ? 0 : (Math.round(state.last_result_t * SYS_CLOCK_PER_MS) % 2 ** 32 | 1) >>> 0
    }),
    // The first block the driver reads is TDC0 bins 0-63 (§7.5.1, cmd 0x80):
    // the REFERENCE histogram, whose peak marks zero distance (§4 Figure 15),
    // so it doesn't move with the target. Needs a histogram type the chip
    // knows (bits 1, 4 or 7), else the driver times out and returns zeros.
    // Ranging is stopped afterwards.
    tile_sense_tof_read_histogram_flat: ({ state, args }) => {
      const type = arg(args, 0, 0) & 255;
      if (!(type & 146)) return { array: new Array(128).fill(0), nextState: { measuring: 0 } };
      return { array: referenceHistogram(state), nextState: { measuring: 0, in_band_streak: 0 } };
    }
  },
  provenance: {
    // canonical — datasheet result layout / conversions
    tile_sense_tof_find: "canonical",
    // ID register check at 0x41
    tile_sense_tof_get_distance_mm: "canonical",
    // MSB<<8 | LSB (mm), 0 saturates to max range
    tile_sense_tof_max_range_mm: "canonical",
    // 200 / 2500 / 5000 by mode
    tile_sense_tof_get_result_flat: "canonical",
    // result record layout
    tile_sense_tof_measure_single_flat: "canonical",
    tile_sense_tof_read_distance_with_confidence: "canonical",
    // long range *100/63; short-range codes 10 / 1
    tile_sense_tof_is_object_within: "canonical",
    // driver: 1 / 10 or ≥ 32, distance > 0, ≤ mm
    tile_sense_tof_wait_for_object: "inferred",
    // one evaluation stands in for the poll loop
    tile_sense_tof_set_distance_mode: "canonical",
    // algo byte / ranges
    tile_sense_tof_set_period: "canonical",
    // repetition period code
    tile_sense_tof_set_kilo_iters: "canonical",
    // cmd_data1/0 kIters
    tile_sense_tof_set_threshold: "canonical",
    // cmd_data3[5:0]
    tile_sense_tof_set_threshold_interrupt: "canonical",
    // WR_ADD_CONFIG, §8.12
    tile_sense_tof_get_threshold_interrupt: "canonical",
    // RD_ADD_CONFIG readback
    tile_sense_tof_get_app_version_flat: "canonical",
    // ROM App0 4.14.0, datasheet §6.4.1
    // inferred — behavioral simplifications
    tile_sense_tof_result_ready: "inferred",
    // "measuring and awake", not a per-period INT_STATUS bit
    tile_sense_tof_factory_calibrate: "inferred",
    tile_sense_tof_get_signal_quality_flat: "inferred",
    // zero-on-no-object is canonical (§7.3.11); magnitudes modeled
    // the chip-state and diagnostic calls: behavior per the datasheet, opaque contents modeled
    tile_sense_tof_get_sys_clock_ticks: "canonical",
    // last result's time stamp, 4.7 MHz units, LSB = valid (§7.3.5)
    tile_sense_tof_get_calibration: "canonical",
    // the driver's copy: blob while valid, else zeros
    tile_sense_tof_set_calibration: "canonical",
    tile_sense_tof_restore_state: "canonical",
    // held, loaded at the next start with valid calibration, consumed
    tile_sense_tof_read_histogram_flat: "inferred",
    // TDC0 reference block per §7.5.1; peak shape and position modeled
    tile_sense_tof_get_serial_number_flat: "inferred",
    // layout per §7.6; the value is per chip
    tile_sense_tof_save_state: "inferred",
    // zeros until ranging (reset value); accumulator contents opaque
    power: "inferred"
    // duty-cycle model over datasheet currents; matches the I_RANGING_AVG rows within 6 %, 5 m ranging current fitted
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
      in_band_streak: inWindow(state) ? state.in_band_streak + 1 : 0
    };
  },
  // INT (pad 9; active-low on hardware, 1 = asserted here). Persistence 0: a
  // result interrupt every measurement, object or not. Persistence n: only once
  // n consecutive results fall inside [low, high], then every in-window period.
  padOutputs(state) {
    if (state.sleeping === 1 || state.measuring !== 1) return { "9": 0 };
    if (state.threshold_persistence === 0) return { "9": 1 };
    return {
      "9": inWindow(state) && state.in_band_streak >= state.threshold_persistence ? 1 : 0
    };
  },
  // Electrical: average supply current from the ranging duty cycle — see
  // `averageCurrentUa`. The chip's VCSEL pulses peak ~230 mA internally; what the
  // rail sees is the average.
  power(state, ctx) {
    const ua = averageCurrentUa(state);
    const vdd = ctx?.padVoltage?.["10"] ?? 3300;
    return {
      draw_ua: ua,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: vdd,
          i_ua: ua,
          pads: ["10"],
          note: currentNote(state)
        }
      ]
    };
  }
};
var sense_tof_default = sim;
export {
  averageCurrentUa,
  sense_tof_default as default,
  maxRangeMm,
  periodMs,
  rangingMs
};
