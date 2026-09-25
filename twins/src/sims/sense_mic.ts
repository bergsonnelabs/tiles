// Digital twin for Sense.MIC — analog MEMS microphone + amplifier + 12-bit ADC
// (driver tile_sense_mic.{h,c}; MAX11645 datasheet 19-4544 Rev 3,
// CMM-2718AT-38164W-TR, AD8605; the production schematic).
//
// Signal chain: a Same Sky CMM-2718AT-38164W analog MEMS mic (−38 dBV/Pa) is
// AC-coupled through C2 into an AD8605 non-inverting stage (48x: R4 47k /
// R3 1k). C3 from the amp's + input to GND halves the audio on the way in, so
// mic to ADC is 24x. The amp output drives the MAX11645's AIN0 and pad 8.
//
// Bias: the + input is biased from the ADC's REF pin through R2/R1
// (330k/330k), and R3 has no DC-blocking cap, so the bias is gained 48x too.
// REF is driven only with the buffered internal reference (SEL 11x): then the
// bias is 1.024 V x 48 and the output sits at the rail. Otherwise REF is not
// connected, the bias is 0 V and the amp rests at 0 V, passing only positive
// half-cycles. The twin models exactly that, so get_dc_offset reads near 0,
// peak_to_peak is the positive peak, and read_spl_db includes the driver's
// half-wave correction.
//
// The world is one control, the sound level. The twin synthesizes the ADC's
// sample stream from it (a ~780 Hz sine, 16 samples per cycle, through the
// chain above) and every host call runs the DRIVER's own integer arithmetic
// over those samples.
//
// The chip converts with the reference it has (`ref_sel`); the driver converts
// counts to mV with its CACHED `vref_mv`. They differ after reset() (the chip
// returns to VDD, the driver's cache does not), as on hardware.
import type { PowerCtx, TileSim } from '../tileSim';

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
const AMP_GAIN = 48; // AD8605: 1 + 47k/1k, for DC as well (no cap on R3)
const INPUT_DIVIDER = 0.5; // C2 / (C2 + C3), 100 nF each
const MIC_MV_PER_PA = 12.589; // −38 dBV/Pa
const RAIL_MV = 3300; // the driver's VDD-reference assumption
const REF_BIAS_MV = 1024; // 2.048 V REF through R2/R1, when REF is driven
const SAMPLES_PER_CYCLE = 16; // ~780 Hz at the driver's ~12.5 ksps burst rate
const TIER2_BUF_LEN = 64; // MIC_TIER2_BUF_LEN
const SPL_OFFSET_DX_DB = 444; // MIC_SPL_OFFSET_DX_DB: 94 − 20·log10(12.59) − 20·log10(24)
const HALFWAVE_DX_DB = 30; // MIC_HALFWAVE_DX_DB: 20·log10(√2)
const HALFWAVE_REST_MAX = 256; // MIC_HALFWAVE_REST_MAX, counts

// k_log10_x20_table: 20·log10(n) in 0.1 dB, n = 1..32 (tile_sense_mic.c).
const LOG10_X20 = [
  0, 0, 60, 95, 120, 140, 156, 169, 181, 191, 200, 208, 216, 223, 229, 235, 241, 246, 251, 256, 260,
  264, 268, 272, 276, 280, 283, 286, 289, 292, 295, 298, 301,
];

/** The chip's reference voltage: SEL2 set = internal 2.048 V, else VDD (3.3 V
 * assumed, as the driver does). */
const refMv = (ref: number) => (ref & 0x04 ? 2048 : 3300);
/** resolve_vref(), the driver's cached value: 2048 for the internal modes. */
const driverVrefMv = refMv;
/** REF is driven only in the buffered internal modes (SEL 11x, Table 6). */
const refDriven = (ref: number) => (ref & 0x06) === 0x06;

/** The mic signal's peak at the ADC, before the rails: SPL → Pa → mic mV →
 * C2/C3 divider → amp. */
function acPeakMv(s: State): number {
  const pa = Math.pow(10, (s.spl_db / 10 - 94) / 20);
  return MIC_MV_PER_PA * pa * Math.SQRT2 * INPUT_DIVIDER * AMP_GAIN;
}
/** The amp output (= pad 8 and AIN0) at stream position `i`, mV: 48x the REF
 * bias plus the audio, clamped to the rails (AD8605 is rail to rail). */
function ampOutMv(s: State, i: number): number {
  const bias = refDriven(s.ref_sel) ? REF_BIAS_MV * AMP_GAIN : 0;
  const ac = acPeakMv(s) * Math.sin((2 * Math.PI * i) / SAMPLES_PER_CYCLE);
  return Math.max(0, Math.min(RAIL_MV, bias + ac));
}

/** One conversion: sample `i` of the stream, in counts, with the CHIP's
 * reference; the ADC clips at its full scale. AIN1 is not connected on this
 * tile and reads 0. */
function sampleAt(s: State, i: number): number {
  if (s.channel !== 0) return 0;
  return Math.max(0, Math.min(ADC_MAX, Math.floor((ampOutMv(s, i) * 4096) / refMv(s.ref_sel))));
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
/** dmv_rms_to_spl_dx10: LUT over 1..32 (0.1 mV); above that, halve k times
 * (rounded) into range and add k·6.02 dB; −20 dB for 0.1 mV → mV; +3 dB when
 * the signal is half-wave. 0 → 30.0 dB, which is also the floor. */
function splDx10(dmvRms: number, halfWave: boolean): number {
  if (dmvRms === 0) return 300;
  let k = 0;
  while (dmvRms >> k > 32) k++;
  const idx = Math.min(32, k === 0 ? dmvRms : (dmvRms + (1 << (k - 1))) >> k);
  const spl =
    LOG10_X20[idx]! +
    Math.trunc((k * 602) / 10) -
    200 +
    SPL_OFFSET_DX_DB +
    (halfWave ? HALFWAVE_DX_DB : 0);
  return Math.max(300, spl);
}
/** read_spl_db: 64 samples → RMS about dc_offset → 0.1 mV (cached vref) → LUT. */
function readSpl(s: State): number {
  const r = rms(samples(s, TIER2_BUF_LEN), s.dc_offset);
  return splDx10(Math.floor((r * s.vref_mv * 10) / 4096), s.dc_offset < HALFWAVE_REST_MAX);
}
/** detect_clap's pattern: a quiet bracket (< 50 dB), then peaks above 70 dB.
 * A clap is ~90 dB SPL near the tile. */
const CLAP_SPL_DX10 = 900;
const advance = (s: State, n: number) => ({ sample_idx: (s.sample_idx + n) % 0x10000 });
const arg = (args: number[], i: number, fallback: number) =>
  args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;

/** dc_offset after init in a 50 dB room at the VDD reference (see defaultState). */
const DEFAULT_DC_OFFSET = 0;

const sim: TileSim<State> = {
  tile: 'Sense.MIC',

  // State AFTER init with no cfg: VDD reference, AIN0, single-channel scan,
  // internal clock, unipolar; dc_offset = the average of 64 samples of a quiet
  // (50 dB) room through the half-wave chain: 0 counts.
  defaultState: {
    spl_db: 500, // 50 dB — a quiet room
    clap_event: 0,
    vref_mv: 3300,
    dc_offset: DEFAULT_DC_OFFSET,
    ref_sel: 0x00,
    channel: 0,
    scan: 3,
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
        'Ambient sound in 0.1 dB (700 = 70 dB). Readings start near 45 dB and clip near 112 dB.',
    },
    {
      type: 'toggle',
      field: 'clap_event',
      label: 'Clap',
      description: 'Arms one clap: detect_clap returns 1 once and clears it.',
    },
  ],

  // The sound in the room.
  stimuli: [
    {
      id: 'sound',
      label: 'Sound level',
      controls: [
        {
          kind: 'slider',
          id: 'spl_db',
          label: 'sound',
          field: 'spl_db',
          min: 300,
          max: 1100,
          step: 10,
          unit: '0.1 dB SPL',
        },
        { kind: 'toggle', id: 'clap', label: 'Clap', fields: ['clap_event'] },
      ],
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
        scan: 3,
        clock: 0,
        polarity: 0,
        sleeping: 0,
        dc_offset: dcLevel(samples({ ...state, ref_sel: 0x00, channel: 0 }, 64)),
      },
    }),
    // Selects the VDD reference (the internal one off) until wake() restores
    // the setup; ref_sel keeps the host's choice for wake.
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
      return { nextState: { ref_sel: ref, vref_mv: driverVrefMv(ref) } };
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
    // The driver's pattern is quiet, peak, quiet gap, peak, quiet. The twin
    // collapses its time: an armed clap is detected when the room is quiet
    // enough for the brackets (< 50 dB as the driver measures it) and a ~90 dB
    // clap reads above 70 dB through this chain. Detection consumes the clap.
    tile_sense_mic_detect_clap: ({ state }) => {
      if (!state.clap_event) return { scalar: 0 };
      const quiet = readSpl(state) < 500;
      const loud = readSpl({ ...state, spl_db: CLAP_SPL_DX10 }) > 700;
      return { scalar: quiet && loud ? 1 : 0, nextState: { clap_event: 0 } };
    },
  },

  provenance: {
    tile_sense_mic_find: 'canonical', // MAX11645 ACK at 0x36
    tile_sense_mic_get_vref_mv: 'canonical', // resolve_vref: 2048 internal / 3300
    tile_sense_mic_set_reference: 'canonical', // setup byte SEL[2:0]
    tile_sense_mic_get_raw: 'canonical', // 12-bit conversion of the modeled input
    tile_sense_mic_get_raw_mv: 'canonical', // raw·vref >> 12
    tile_sense_mic_get_audio_sample: 'canonical', // raw − dc_offset
    tile_sense_mic_get_dc_offset: 'canonical', // mean of the resting level; the schematic's 0 V bias
    tile_sense_mic_calibrate: 'canonical',
    tile_sense_mic_set_polarity: 'canonical', // stored; single-ended ignores it (datasheet)
    tile_sense_mic_sleep: 'canonical',
    tile_sense_mic_dc_level: 'canonical', // driver math, exact
    tile_sense_mic_peak_to_peak: 'canonical',
    tile_sense_mic_rms: 'canonical', // incl. Newton isqrt + uint32 accumulator
    tile_sense_mic_amplitude_mv: 'canonical',
    tile_sense_mic_read_spl_db: 'canonical', // driver pipeline + LUT over the modeled stream
    tile_sense_mic_is_loud: 'canonical',
    // inferred — the modeled stream itself, and time collapsed to one evaluation
    tile_sense_mic_get_samples: 'inferred', // a pure tone through the modeled chain
    tile_sense_mic_wait_for_sound: 'inferred',
    tile_sense_mic_detect_clap: 'inferred', // the pattern's time collapsed; thresholds are the driver's
    power: 'inferred', // datasheet typicals summed: mic 175 µA, AD8605 1 mA, ADC per reference
  },

  // Analog out (pad 8): the AD8605 output at the current point of the stream,
  // as a fraction of the rail.
  padOutputs(state) {
    return { '8': ampOutMv(state, state.sample_idx) / RAIL_MV };
  },

  // Electrical: a pure load on V+ (pad 10). The mic (175 µA, CMM-2718AT-38164W)
  // and the AD8605 (1 mA typ) run continuously. The MAX11645 shuts itself down
  // between conversions (0.5 µA) EXCEPT its internal reference when selected
  // "always on" (0x05 / 0x07), ~330 µA (IDD at 1 ksps, internal ref); sleep()
  // turns that off. Conversion current while the program samples is not
  // modeled (it paces that).
  power(state, ctx?: PowerCtx) {
    const refOn = !state.sleeping && (state.ref_sel === 0x05 || state.ref_sel === 0x07);
    const adc = refOn ? 330 : 0.5;
    const ua = Math.round(175 + 1000 + adc);
    return {
      draw_ua: ua,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: ctx?.padVoltage?.['10'] ?? RAIL_MV,
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
