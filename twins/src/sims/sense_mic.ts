// Digital twin for Sense.MIC — analog MEMS microphone + amplifier + 12-bit ADC
// (driver tile_sense_mic.{h,c}; MAX11645 datasheet 19-4544 Rev 3, CMM-2718AT,
// AD8605).
//
// Signal chain (schematic truth): a Same Sky/CUI CMM-2718AT analog MEMS mic
// (−42 dBV/Pa) is AC-coupled into an AD8605 op-amp with ~48× non-inverting gain
// (R4 47k / R3 1k), biased to mid-rail by R1/R2 330k/330k; the amplified output
// drives the MAX11645's AIN0 AND is tapped out to a pad.
//
// The world is one control: the sound level. The twin synthesizes the ADC's
// sample stream from it — mid-rail bias plus a ~780 Hz sine (16 samples per
// cycle) whose amplitude is SPL → Pa → mic mV → ×48 — and every host call then
// runs the DRIVER's own integer arithmetic over those samples. So read_spl_db
// gives exactly what the driver computes, including its LUT saturating at
// 72.5 dB and flooring at 30.0 dB, and the buffer helpers (dc_level / rms /
// peak_to_peak) see realistic data.
//
// The chip converts with the reference the chip has (`ref_sel`); the driver
// converts counts to mV with its CACHED `vref_mv`. They differ after reset()
// (the chip returns to VDD, the driver's cache does not) — as on hardware.
import type { TileSim } from '../tileSim';

interface State {
  // ── the world (controls) ──
  /** Ambient sound level in 0.1 dB SPL (700 = 70 dB), the driver's unit. */
  spl_db: number;
  /** A one-shot clap: detect_clap returns 1 once and clears it. The driver's
   * detector needs a quiet/peak/quiet/peak/quiet pattern in time, which a
   * static level can never produce — this stands in for that event. */
  clap_event: number;

  // ── the driver's cached state ──
  /** Reference in mV the driver converts with (resolve_vref: 2048 for the
   * internal references, else 3300). */
  vref_mv: number;
  /** DC bias in counts, averaged over 64 samples by init / calibrate. */
  dc_offset: number;

  // ── the chip's configuration (setup / config bytes) ──
  ref_sel: number; // sense_mic_ref_t: 0x00 VDD, 0x02 ext, 0x05 internal, 0x07 internal+REF out
  channel: number; // 0 AIN0 (mic), 1 AIN1 (not routed)
  scan: number; // sense_mic_scan_t (stored; conversion model is single-channel)
  clock: number; // 0 internal, 1 external (stored)
  polarity: number; // 0 unipolar, 1 bipolar (stored; samples modeled unipolar)
  sleeping: number; // sleep() — resets the config register only; see power()

  /** Position in the synthesized sample stream: every conversion advances it,
   * so consecutive reads walk along the waveform. */
  sample_idx: number;
}

const ADC_MAX = 4095;
const AMP_GAIN = 48; // AD8605: 1 + 47k/1k
const MIC_MV_PER_PA = 7.943; // −42 dBV/Pa
const RAIL_MV = 3300;
const BIAS_MV = RAIL_MV / 2; // R1/R2 330k/330k mid-rail bias
const SAMPLES_PER_CYCLE = 16; // ~780 Hz at the driver's ~12.5 ksps burst rate
const TIER2_BUF_LEN = 64; // MIC_TIER2_BUF_LEN
const AMP_GAIN_DX_DB = 336; // MIC_AMP_GAIN_DX_DB: 20·log10(48) in 0.1 dB

// k_log10_x20_table: 20·log10(n) in 0.1 dB, n = 1..32 (tile_sense_mic.c).
const LOG10_X20 = [
  0, 0, 60, 95, 120, 140, 156, 169, 181, 191, 200, 208, 216, 223, 229, 235, 241, 246, 251, 256, 260,
  264, 268, 272, 276, 280, 283, 286, 289, 292, 295, 298, 301,
];

/** resolve_vref(): the internal references are 2.048 V; VDD / external assume 3.3 V. */
const refMv = (ref: number) => (ref === 0x05 || ref === 0x07 ? 2048 : 3300);

/** The amplified signal's peak swing at the ADC input, mV. */
function peakMv(s: State): number {
  const pa = Math.pow(10, (s.spl_db / 10 - 94) / 20);
  return MIC_MV_PER_PA * pa * AMP_GAIN * Math.SQRT2;
}

/** One conversion: sample `i` of the stream, in counts, with the CHIP's reference.
 * The amp swings rail to rail at most; the ADC clips at its full scale. AIN1 is
 * not routed on this tile and reads 0. */
function sampleAt(s: State, i: number): number {
  if (s.channel !== 0) return 0;
  const ac = peakMv(s) * Math.sin((2 * Math.PI * i) / SAMPLES_PER_CYCLE);
  const mv = Math.max(0, Math.min(RAIL_MV, BIAS_MV + ac));
  return Math.max(0, Math.min(ADC_MAX, Math.floor((mv * 4096) / refMv(s.ref_sel))));
}
function samples(s: State, n: number): number[] {
  return Array.from({ length: n }, (_, k) => sampleAt(s, s.sample_idx + k));
}

// ── the driver's integer math ──
function dcLevel(buf: readonly number[]): number {
  if (buf.length === 0) return 0;
  let sum = 0;
  for (const v of buf) sum += v & 0xffff;
  return Math.floor(sum / buf.length) & 0xffff;
}
function peakToPeak(buf: readonly number[]): number {
  if (buf.length === 0) return 0;
  let lo = buf[0]! & 0xffff;
  let hi = lo;
  for (const raw of buf) {
    const v = raw & 0xffff;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  return hi - lo;
}
function rms(buf: readonly number[], dcOffset: number): number {
  if (buf.length === 0) return 0;
  let sumSq = 0;
  for (const v of buf) {
    const ac = (v & 0xffff) - (dcOffset & 0xffff);
    sumSq = (sumSq + ac * ac) >>> 0; // uint32 accumulator
  }
  const meanSq = Math.floor(sumSq / buf.length);
  if (meanSq === 0) return 0;
  // Integer square root, Newton's method — as the driver does it.
  let x = meanSq;
  let y = Math.floor((x + 1) / 2);
  while (y < x) {
    x = y;
    y = Math.floor((x + Math.floor(meanSq / x)) / 2);
  }
  return x & 0xffff;
}
const toMv = (counts: number, vref: number) => Math.floor((counts * vref) / 4096);
/** mv_rms_to_spl_dx10: LUT over 1..32 mV, saturating; 0 mV → 30.0 dB. */
function splDx10(mvRms: number): number {
  if (mvRms === 0) return 300;
  return LOG10_X20[Math.min(32, mvRms)]! + 760 - AMP_GAIN_DX_DB;
}
/** read_spl_db: 64 samples → RMS about dc_offset → mV (cached vref) → LUT. */
function readSpl(s: State): number {
  const r = rms(samples(s, TIER2_BUF_LEN), s.dc_offset);
  return splDx10(toMv(r, s.vref_mv));
}
const advance = (s: State, n: number) => ({ sample_idx: (s.sample_idx + n) % 0x10000 });
const arg = (args: number[], i: number, fallback: number) =>
  args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;

const sim: TileSim<State> = {
  tile: 'Sense.MIC',

  // State AFTER init with no cfg: VDD reference, AIN0, scan code 0 (the .c's
  // no-cfg default), internal clock, unipolar; dc_offset = the average of 64
  // samples of the mid-rail bias: 2047 counts at the 3.3 V reference (the floor
  // of each conversion pulls the mean just under 2048).
  defaultState: {
    spl_db: 500, // 50 dB — a quiet room
    clap_event: 0,
    vref_mv: 3300,
    dc_offset: 2047,
    ref_sel: 0x00,
    channel: 0,
    scan: 0,
    clock: 0,
    polarity: 0,
    sleeping: 0,
    sample_idx: 0,
  },

  controls: [
    {
      type: 'slider',
      field: 'spl_db',
      label: 'Sound level',
      min: 300,
      max: 1200,
      step: 10,
      unit: '0.1 dB SPL',
      description:
        "Ambient sound in 0.1 dB (700 = 70 dB). The driver's SPL readout saturates near 72.5 dB.",
    },
    {
      type: 'toggle',
      field: 'clap_event',
      label: 'Clap',
      description: 'Arms one clap: detect_clap returns 1 once and clears it.',
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_sense_mic_find: () => ({ scalar: 1 }),
    tile_sense_mic_init: ({ state }) => ({
      scalar: 0,
      nextState: {
        vref_mv: 3300,
        ref_sel: 0x00,
        channel: 0,
        scan: 0,
        clock: 0,
        polarity: 0,
        sleeping: 0,
        dc_offset: dcLevel(samples({ ...state, ref_sel: 0x00, channel: 0 }, 64)),
      },
    }),
    // A setup byte with RST=0: resets the config register, keeps the reference.
    tile_sense_mic_sleep: () => ({ nextState: { sleeping: 1 } }),
    tile_sense_mic_wake: () => ({ nextState: { sleeping: 0 } }),
    // Chip back to power-on setup (VDD, internal clock, unipolar) and config
    // (AIN0). The driver's cache (vref_mv, dc_offset) is untouched — re-init.
    tile_sense_mic_reset: () => ({
      nextState: { ref_sel: 0x00, clock: 0, polarity: 0, channel: 0, scan: 0, sleeping: 0 },
    }),

    // ── configuration ──
    tile_sense_mic_set_reference: ({ state, args }) => {
      const ref = arg(args, 0, state.ref_sel) & 0x07;
      return { nextState: { ref_sel: ref, vref_mv: refMv(ref) } };
    },
    tile_sense_mic_set_channel: ({ state, args }) => ({
      nextState: { channel: arg(args, 0, state.channel) & 0x01 },
    }),
    tile_sense_mic_set_scan_mode: ({ state, args }) => ({
      nextState: { scan: arg(args, 0, state.scan) & 0x03 },
    }),
    tile_sense_mic_set_clock_mode: ({ state, args }) => ({
      nextState: { clock: arg(args, 0, state.clock) ? 1 : 0 },
    }),
    tile_sense_mic_set_polarity: ({ state, args }) => ({
      nextState: { polarity: arg(args, 0, state.polarity) ? 1 : 0 },
    }),
    tile_sense_mic_get_vref_mv: ({ state }) => ({ scalar: state.vref_mv }),

    // ── single conversions ──
    tile_sense_mic_get_raw: ({ state }) => ({
      scalar: sampleAt(state, state.sample_idx),
      nextState: advance(state, 1),
    }),
    tile_sense_mic_get_raw_mv: ({ state }) => ({
      scalar: toMv(sampleAt(state, state.sample_idx), state.vref_mv),
      nextState: advance(state, 1),
    }),
    // Signed, relative to the calibrated offset.
    tile_sense_mic_get_audio_sample: ({ state }) => ({
      scalar: sampleAt(state, state.sample_idx) - state.dc_offset,
      nextState: advance(state, 1),
    }),
    tile_sense_mic_get_dc_offset: ({ state }) => ({ scalar: state.dc_offset }),
    tile_sense_mic_calibrate: ({ state }) => ({
      nextState: { dc_offset: dcLevel(samples(state, 64)), ...advance(state, 64) },
    }),
    // Burst: fills the caller's buffer, one conversion per slot.
    tile_sense_mic_get_samples: ({ state, caps }) => {
      const n = Math.max(0, caps?.buf ?? 0);
      return { out: { buf: samples(state, n) }, nextState: advance(state, n) };
    },

    // ── pure computation over the caller's samples (no I2C) ──
    tile_sense_mic_dc_level: ({ bufferIn }) => ({ scalar: dcLevel(bufferIn?.samples ?? []) }),
    tile_sense_mic_peak_to_peak: ({ bufferIn }) => ({
      scalar: peakToPeak(bufferIn?.samples ?? []),
    }),
    tile_sense_mic_rms: ({ bufferIn, args }) => ({
      scalar: rms(bufferIn?.samples ?? [], arg(args, 0, 0)),
    }),
    tile_sense_mic_amplitude_mv: ({ state, args }) => ({
      scalar: toMv(arg(args, 0, 0) & 0xffff, state.vref_mv),
    }),

    // ── SPL helpers (0.1 dB units) ──
    tile_sense_mic_read_spl_db: ({ state }) => ({
      scalar: readSpl(state),
      nextState: advance(state, TIER2_BUF_LEN),
    }),
    tile_sense_mic_is_loud: ({ state, args }) => ({
      scalar: readSpl(state) > arg(args, 0, 0) ? 1 : 0,
      nextState: advance(state, TIER2_BUF_LEN),
    }),
    // Polls read_spl_db until it exceeds the threshold. The level doesn't move
    // during the call, so one evaluation answers it.
    tile_sense_mic_wait_for_sound: ({ state, args }) => ({
      scalar: readSpl(state) > arg(args, 0, 0) ? 1 : 0,
      nextState: advance(state, TIER2_BUF_LEN),
    }),
    tile_sense_mic_detect_clap: ({ state }) =>
      state.clap_event ? { scalar: 1, nextState: { clap_event: 0 } } : { scalar: 0 },
  },

  provenance: {
    tile_sense_mic_find: 'canonical', // MAX11645 ACK at 0x36
    tile_sense_mic_get_vref_mv: 'canonical', // resolve_vref: 2048 internal / 3300
    tile_sense_mic_set_reference: 'canonical', // setup byte SEL[2:0]
    tile_sense_mic_get_raw: 'canonical', // 12-bit conversion of the modeled input
    tile_sense_mic_get_raw_mv: 'canonical', // raw·vref >> 12
    tile_sense_mic_get_audio_sample: 'canonical', // raw − dc_offset
    tile_sense_mic_get_dc_offset: 'canonical',
    tile_sense_mic_calibrate: 'canonical', // mean of 64 samples
    tile_sense_mic_dc_level: 'canonical', // driver math, exact
    tile_sense_mic_peak_to_peak: 'canonical',
    tile_sense_mic_rms: 'canonical', // incl. Newton isqrt + uint32 accumulator
    tile_sense_mic_amplitude_mv: 'canonical',
    tile_sense_mic_read_spl_db: 'canonical', // driver pipeline + LUT over the modeled stream
    tile_sense_mic_is_loud: 'canonical',
    // inferred — the modeled stream itself, and time collapsed to one evaluation
    tile_sense_mic_get_samples: 'inferred', // SPL → ×48 sine around mid-rail
    tile_sense_mic_wait_for_sound: 'inferred',
    // hallucinated — a toggle stands in for the time-shaped clap pattern
    tile_sense_mic_detect_clap: 'hallucinated',
    power: 'inferred', // datasheet typicals summed: mic 250 µA, AD8605 1 mA, ADC per reference
  },

  // AOUT (pad 6 in Sense-MIC-a.json; the schematic routes it to pad 8 — the DB
  // is mid-reconciliation): the amplified, mid-rail-biased signal, as a fraction
  // of the 3.3 V rail at the current point of the stream.
  padOutputs(state) {
    const ac = peakMv(state) * Math.sin((2 * Math.PI * state.sample_idx) / SAMPLES_PER_CYCLE);
    const mv = Math.max(0, Math.min(RAIL_MV, BIAS_MV + ac));
    return { '6': mv / RAIL_MV };
  },

  // Electrical: a pure load on V+ (pad 10). The mic (250 µA, CMM-2718AT) and the
  // AD8605 (1 mA typ) run continuously. The MAX11645 shuts itself down between
  // conversions (0.5 µA) EXCEPT its internal reference when selected "always on"
  // (0x05 / 0x07), ~330 µA (IDD at 1 ksps, internal ref). sleep() only resets the
  // config register — the setup byte keeps the reference — so it saves nothing.
  // Conversion current while the program samples is not modeled (it paces that).
  power(state) {
    const refOn = state.ref_sel === 0x05 || state.ref_sel === 0x07;
    const adc = refOn ? 330 : 0.5;
    const ua = Math.round(250 + 1000 + adc);
    return {
      draw_ua: ua,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: RAIL_MV,
          i_ua: ua,
          pads: ['10'],
          note: refOn
            ? 'mic + AD8605 + ADC internal reference on'
            : 'mic + AD8605; ADC auto-shutdown between conversions',
        },
      ],
    };
  },
};

export default sim;
