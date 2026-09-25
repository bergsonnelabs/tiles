// src/sims/sense_mic.ts
var ADC_MAX = 4095;
var AMP_GAIN = 48;
var INPUT_DIVIDER = 0.5;
var MIC_MV_PER_PA = 12.589;
var RAIL_MV = 3300;
var REF_BIAS_MV = 1024;
var SAMPLES_PER_CYCLE = 16;
var TIER2_BUF_LEN = 64;
var SPL_OFFSET_DX_DB = 444;
var HALFWAVE_DX_DB = 30;
var HALFWAVE_REST_MAX = 256;
var LOG10_X20 = [
  0,
  0,
  60,
  95,
  120,
  140,
  156,
  169,
  181,
  191,
  200,
  208,
  216,
  223,
  229,
  235,
  241,
  246,
  251,
  256,
  260,
  264,
  268,
  272,
  276,
  280,
  283,
  286,
  289,
  292,
  295,
  298,
  301
];
var refMv = (ref) => ref & 4 ? 2048 : 3300;
var driverVrefMv = refMv;
var refDriven = (ref) => (ref & 6) === 6;
function acPeakMv(s) {
  const pa = Math.pow(10, (s.spl_db / 10 - 94) / 20);
  return MIC_MV_PER_PA * pa * Math.SQRT2 * INPUT_DIVIDER * AMP_GAIN;
}
function ampOutMv(s, i) {
  const bias = refDriven(s.ref_sel) ? REF_BIAS_MV * AMP_GAIN : 0;
  const ac = acPeakMv(s) * Math.sin(2 * Math.PI * i / SAMPLES_PER_CYCLE);
  return Math.max(0, Math.min(RAIL_MV, bias + ac));
}
function sampleAt(s, i) {
  if (s.channel !== 0) return 0;
  return Math.max(0, Math.min(ADC_MAX, Math.floor(ampOutMv(s, i) * 4096 / refMv(s.ref_sel))));
}
function samples(s, n) {
  return Array.from({ length: n }, (_, k) => sampleAt(s, s.sample_idx + k));
}
function dcLevel(buf) {
  if (buf.length === 0) return 0;
  let sum = 0;
  for (const v of buf) sum += v & 65535;
  return Math.floor(sum / buf.length) & 65535;
}
function peakToPeak(buf) {
  if (buf.length === 0) return 0;
  let lo = buf[0] & 65535;
  let hi = lo;
  for (const raw of buf) {
    const v = raw & 65535;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  return hi - lo;
}
function rms(buf, dcOffset) {
  if (buf.length === 0) return 0;
  let sumSq = 0;
  for (const v of buf) {
    const ac = (v & 65535) - (dcOffset & 65535);
    sumSq = sumSq + ac * ac >>> 0;
  }
  const meanSq = Math.floor(sumSq / buf.length);
  if (meanSq === 0) return 0;
  let x = meanSq;
  let y = Math.floor((x + 1) / 2);
  while (y < x) {
    x = y;
    y = Math.floor((x + Math.floor(meanSq / x)) / 2);
  }
  return x & 65535;
}
var toMv = (counts, vref) => Math.floor(counts * vref / 4096);
function splDx10(dmvRms, halfWave) {
  if (dmvRms === 0) return 300;
  let k = 0;
  while (dmvRms >> k > 32) k++;
  const idx = Math.min(32, k === 0 ? dmvRms : dmvRms + (1 << k - 1) >> k);
  const spl = LOG10_X20[idx] + Math.trunc(k * 602 / 10) - 200 + SPL_OFFSET_DX_DB + (halfWave ? HALFWAVE_DX_DB : 0);
  return Math.max(300, spl);
}
function readSpl(s) {
  const r = rms(samples(s, TIER2_BUF_LEN), s.dc_offset);
  return splDx10(Math.floor(r * s.vref_mv * 10 / 4096), s.dc_offset < HALFWAVE_REST_MAX);
}
var CLAP_SPL_DX10 = 900;
var advance = (s, n) => ({ sample_idx: (s.sample_idx + n) % 65536 });
var arg = (args, i, fallback) => args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;
var DEFAULT_DC_OFFSET = 0;
var sim = {
  tile: "Sense.MIC",
  // State AFTER init with no cfg: VDD reference, AIN0, single-channel scan,
  // internal clock, unipolar; dc_offset = the average of 64 samples of a quiet
  // (50 dB) room through the half-wave chain: 0 counts.
  defaultState: {
    spl_db: 500,
    // 50 dB — a quiet room
    clap_event: 0,
    vref_mv: 3300,
    dc_offset: DEFAULT_DC_OFFSET,
    ref_sel: 0,
    channel: 0,
    scan: 3,
    clock: 0,
    polarity: 0,
    sleeping: 0,
    sample_idx: 0
  },
  controls: [
    {
      type: "slider",
      field: "spl_db",
      label: "Sound level",
      min: 300,
      max: 1200,
      step: 10,
      unit: "0.1 dB SPL",
      description: "Ambient sound in 0.1 dB (700 = 70 dB). Readings start near 45 dB and clip near 112 dB."
    },
    {
      type: "toggle",
      field: "clap_event",
      label: "Clap",
      description: "Arms one clap: detect_clap returns 1 once and clears it."
    }
  ],
  // The sound in the room.
  stimuli: [
    {
      id: "sound",
      label: "Sound level",
      controls: [
        {
          kind: "slider",
          id: "spl_db",
          label: "sound",
          field: "spl_db",
          min: 300,
          max: 1100,
          step: 10,
          unit: "0.1 dB SPL"
        },
        { kind: "toggle", id: "clap", label: "Clap", fields: ["clap_event"] }
      ]
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    tile_sense_mic_find: () => ({ scalar: 1 }),
    tile_sense_mic_init: ({ state }) => ({
      scalar: 0,
      nextState: {
        vref_mv: 3300,
        ref_sel: 0,
        channel: 0,
        scan: 3,
        clock: 0,
        polarity: 0,
        sleeping: 0,
        dc_offset: dcLevel(samples({ ...state, ref_sel: 0, channel: 0 }, 64))
      }
    }),
    // Selects the VDD reference (the internal one off) until wake() restores
    // the setup; ref_sel keeps the host's choice for wake.
    tile_sense_mic_sleep: () => ({ nextState: { sleeping: 1 } }),
    tile_sense_mic_wake: () => ({ nextState: { sleeping: 0 } }),
    // Chip back to power-on setup (VDD, internal clock, unipolar) and config
    // (AIN0). The driver's cache (vref_mv, dc_offset) is untouched — re-init.
    tile_sense_mic_reset: () => ({
      nextState: { ref_sel: 0, clock: 0, polarity: 0, channel: 0, scan: 0, sleeping: 0 }
    }),
    // ── configuration ──
    tile_sense_mic_set_reference: ({ state, args }) => {
      const ref = arg(args, 0, state.ref_sel) & 7;
      return { nextState: { ref_sel: ref, vref_mv: driverVrefMv(ref) } };
    },
    tile_sense_mic_set_channel: ({ state, args }) => ({
      nextState: { channel: arg(args, 0, state.channel) & 1 }
    }),
    tile_sense_mic_set_scan_mode: ({ state, args }) => ({
      nextState: { scan: arg(args, 0, state.scan) & 3 }
    }),
    tile_sense_mic_set_clock_mode: ({ state, args }) => ({
      nextState: { clock: arg(args, 0, state.clock) ? 1 : 0 }
    }),
    tile_sense_mic_set_polarity: ({ state, args }) => ({
      nextState: { polarity: arg(args, 0, state.polarity) ? 1 : 0 }
    }),
    tile_sense_mic_get_vref_mv: ({ state }) => ({ scalar: state.vref_mv }),
    // ── single conversions ──
    tile_sense_mic_get_raw: ({ state }) => ({
      scalar: sampleAt(state, state.sample_idx),
      nextState: advance(state, 1)
    }),
    tile_sense_mic_get_raw_mv: ({ state }) => ({
      scalar: toMv(sampleAt(state, state.sample_idx), state.vref_mv),
      nextState: advance(state, 1)
    }),
    // Signed, relative to the calibrated offset.
    tile_sense_mic_get_audio_sample: ({ state }) => ({
      scalar: sampleAt(state, state.sample_idx) - state.dc_offset,
      nextState: advance(state, 1)
    }),
    tile_sense_mic_get_dc_offset: ({ state }) => ({ scalar: state.dc_offset }),
    tile_sense_mic_calibrate: ({ state }) => ({
      nextState: { dc_offset: dcLevel(samples(state, 64)), ...advance(state, 64) }
    }),
    // Burst: fills the caller's buffer, one conversion per slot.
    tile_sense_mic_get_samples: ({ state, caps }) => {
      const n = Math.max(0, caps?.buf ?? 0);
      return { out: { buf: samples(state, n) }, nextState: advance(state, n) };
    },
    // ── pure computation over the caller's samples (no I2C) ──
    tile_sense_mic_dc_level: ({ bufferIn }) => ({ scalar: dcLevel(bufferIn?.samples ?? []) }),
    tile_sense_mic_peak_to_peak: ({ bufferIn }) => ({
      scalar: peakToPeak(bufferIn?.samples ?? [])
    }),
    tile_sense_mic_rms: ({ bufferIn, args }) => ({
      scalar: rms(bufferIn?.samples ?? [], arg(args, 0, 0))
    }),
    tile_sense_mic_amplitude_mv: ({ state, args }) => ({
      scalar: toMv(arg(args, 0, 0) & 65535, state.vref_mv)
    }),
    // ── SPL helpers (0.1 dB units) ──
    tile_sense_mic_read_spl_db: ({ state }) => ({
      scalar: readSpl(state),
      nextState: advance(state, TIER2_BUF_LEN)
    }),
    tile_sense_mic_is_loud: ({ state, args }) => ({
      scalar: readSpl(state) > arg(args, 0, 0) ? 1 : 0,
      nextState: advance(state, TIER2_BUF_LEN)
    }),
    // Polls read_spl_db until it exceeds the threshold. The level doesn't move
    // during the call, so one evaluation answers it.
    tile_sense_mic_wait_for_sound: ({ state, args }) => ({
      scalar: readSpl(state) > arg(args, 0, 0) ? 1 : 0,
      nextState: advance(state, TIER2_BUF_LEN)
    }),
    // The driver's pattern is quiet, peak, quiet gap, peak, quiet. The twin
    // collapses its time: an armed clap is detected when the room is quiet
    // enough for the brackets (< 50 dB as the driver measures it) and a ~90 dB
    // clap reads above 70 dB through this chain. Detection consumes the clap.
    tile_sense_mic_detect_clap: ({ state }) => {
      if (!state.clap_event) return { scalar: 0 };
      const quiet = readSpl(state) < 500;
      const loud = readSpl({ ...state, spl_db: CLAP_SPL_DX10 }) > 700;
      return { scalar: quiet && loud ? 1 : 0, nextState: { clap_event: 0 } };
    }
  },
  provenance: {
    tile_sense_mic_find: "canonical",
    // MAX11645 ACK at 0x36
    tile_sense_mic_get_vref_mv: "canonical",
    // resolve_vref: 2048 internal / 3300
    tile_sense_mic_set_reference: "canonical",
    // setup byte SEL[2:0]
    tile_sense_mic_get_raw: "canonical",
    // 12-bit conversion of the modeled input
    tile_sense_mic_get_raw_mv: "canonical",
    // raw·vref >> 12
    tile_sense_mic_get_audio_sample: "canonical",
    // raw − dc_offset
    tile_sense_mic_get_dc_offset: "canonical",
    // mean of the resting level; the schematic's 0 V bias
    tile_sense_mic_calibrate: "canonical",
    tile_sense_mic_set_polarity: "canonical",
    // stored; single-ended ignores it (datasheet)
    tile_sense_mic_sleep: "canonical",
    tile_sense_mic_dc_level: "canonical",
    // driver math, exact
    tile_sense_mic_peak_to_peak: "canonical",
    tile_sense_mic_rms: "canonical",
    // incl. Newton isqrt + uint32 accumulator
    tile_sense_mic_amplitude_mv: "canonical",
    tile_sense_mic_read_spl_db: "canonical",
    // driver pipeline + LUT over the modeled stream
    tile_sense_mic_is_loud: "canonical",
    // inferred — the modeled stream itself, and time collapsed to one evaluation
    tile_sense_mic_get_samples: "inferred",
    // a pure tone through the modeled chain
    tile_sense_mic_wait_for_sound: "inferred",
    tile_sense_mic_detect_clap: "inferred",
    // the pattern's time collapsed; thresholds are the driver's
    power: "inferred"
    // datasheet typicals summed: mic 175 µA, AD8605 1 mA, ADC per reference
  },
  // Analog out (pad 8): the AD8605 output at the current point of the stream,
  // as a fraction of the rail.
  padOutputs(state) {
    return { "8": ampOutMv(state, state.sample_idx) / RAIL_MV };
  },
  // Electrical: a pure load on V+ (pad 10). The mic (175 µA, CMM-2718AT-38164W)
  // and the AD8605 (1 mA typ) run continuously. The MAX11645 shuts itself down
  // between conversions (0.5 µA) EXCEPT its internal reference when selected
  // "always on" (0x05 / 0x07), ~330 µA (IDD at 1 ksps, internal ref); sleep()
  // turns that off. Conversion current while the program samples is not
  // modeled (it paces that).
  power(state, ctx) {
    const refOn = !state.sleeping && (state.ref_sel === 5 || state.ref_sel === 7);
    const adc = refOn ? 330 : 0.5;
    const ua = Math.round(175 + 1e3 + adc);
    return {
      draw_ua: ua,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: ctx?.padVoltage?.["10"] ?? RAIL_MV,
          i_ua: ua,
          pads: ["10"],
          note: refOn ? "mic + AD8605 + ADC internal reference on" : "mic + AD8605; ADC auto-shutdown between conversions"
        }
      ]
    };
  }
};
var sense_mic_default = sim;
export {
  sense_mic_default as default
};
