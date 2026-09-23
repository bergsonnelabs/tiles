// Digital twin for Sense.BP — ST ILPS22QS barometric pressure / temperature sensor.
//
// Pads (Sense-BP-a.json): GND (1), AD0 (2: pulled up → 0x5D, GND → 0x5C),
// I2C.EN (3), I²C CLK/DAT (4/5), INT (9), V+ (10, 1.8–3.6 V).
//
// The dialed pressure / temperature are the WORLD; what the driver reads is the
// chip's sampled output. The tick runs the chip's chain at the configured ODR —
// averaging noise, the optional low-pass filter, DRDY flags, the FIFO, and the
// threshold interrupt — so every setter shapes what the program sees. Sampling
// stops in power-down (sleep, or after a software reset); a one-shot still
// converts once.
//
// Driver behavior follows tile_sense_bp.c: init is called with NULL cfg, so the
// state after init is 25 Hz, AVG 4, mode-1 full scale (4096 LSB/hPa), LPF ON at
// ODR/4, BDU on; mhPa = raw·125/512 (mode 1) or raw·125/256 (mode 2), truncated;
// temperature is 100 LSB/°C; set_odr / set_avg rewrite CTRL_REG1 with a live ODR
// (so they also end a sleep); SWRESET leaves the chip powered down.
//
// No ILPS22QS datasheet in the reference folder: the noise figure, the LPF
// response and the supply currents are marked hallucinated / inferred below.
import type { TileSim } from '../tileSim';

// sense_bp_odr_t → Hz
const ODR_HZ: Readonly<Record<number, number>> = {
  0: 0,
  1: 1,
  2: 4,
  3: 10,
  4: 25,
  5: 50,
  6: 75,
  7: 100,
  8: 200,
};
// sense_bp_avg_t → samples averaged
const AVG_N: Readonly<Record<number, number>> = { 0: 4, 1: 8, 2: 16, 3: 32, 4: 64, 5: 128, 7: 512 };
// Single-pole α at the ODR for a −3 dB corner of ODR/4 or ODR/9: 1 − e^(−2π·fc/ODR).
const LPF_ALPHA_ODR_4 = 1 - Math.exp((-2 * Math.PI) / 4);
const LPF_ALPHA_ODR_9 = 1 - Math.exp((-2 * Math.PI) / 9);
const FIFO_DEPTH = 128;
const FIFO_FILLING = new Set([0x01, 0x02]); // FIFO (stop when full), continuous
// Pressure noise at AVG 4 (hPa RMS), shrinking as 1/√(N/4). Placeholder figure.
const NOISE_HPA_AVG4 = 0.01;
// Supply (hallucinated — no datasheet on file): power-down floor + a charge per
// conversion that grows with the averaging count, capped at continuous converting.
const IDD_PD_UA = 0.5;
const IDD_PER_HZ_AVG4_UA = 1.2;
const IDD_MAX_UA = 25;
const VDD_MIN_MV = 1700;

// INTERRUPT_CFG bits
const INT_PHE = 0x01;
const INT_PLE = 0x02;
const INT_LIR = 0x04;
const INT_RESET_AZ = 0x10;
const INT_AUTOZERO = 0x20;
const INT_RESET_ARP = 0x40;
const INT_AUTOREFP = 0x80;

interface State {
  // ── the world (stimuli) ──
  pressure_hPa: number;
  temperature_c: number;

  // ── chip output registers ──
  sampled_pressure_hPa: number;
  sampled_temp_c: number;
  last_sample_ms: number;
  pressure_drdy: number;
  temp_drdy: number;
  oneshot_pending: number;

  // ── configuration ──
  odr_hz: number;
  avg_n: number;
  fs_lsb_per_hpa: number; // 4096 (mode 1) or 2048 (mode 2)
  lpf_alpha: number; // 1 = LPF off
  sleeping: number; // ODR forced to power-down by sleep()
  pressure_offset_lsb: number; // RPDS

  // ── FIFO (comma-separated raw samples, oldest first) ──
  fifo_mode: number;
  fifo_data: string;
  fifo_watermark: number;

  // ── interrupt + reference ──
  int_threshold_hPa: number;
  int_cfg: number;
  int_ref_hPa: number; // REF_P, in hPa
  autozero: number; // outputs show P − REF_P
  int_latched: number; // PH / PL latched (LIR)
  int_prev_high: boolean;
  int_prev_low: boolean;

  // ── supply stimulus (standalone) ──
  vplus_mv: number;
}

const num = (args: number[], i: number, dflt: number) =>
  args.length > i && Number.isFinite(args[i]) ? Math.trunc(args[i]!) : dflt;

const fifoParse = (s: string): number[] => (s ? s.split(',').map(Number) : []);
const fifoSerialize = (a: readonly number[]) => a.join(',');

// Deterministic unit gaussian from a seed — deriveState must be pure.
function pseudoGaussian(seed: number): number {
  const s1 = Math.sin(seed * 12.9898) * 43758.5453;
  const s2 = Math.sin(seed * 78.233) * 43758.5453;
  const u1 = Math.abs(s1 - Math.floor(s1));
  const u2 = Math.abs(s2 - Math.floor(s2));
  if (u1 < 1e-9) return 0;
  return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}

// PRESS_OUT as the chip presents it: autozero-relative, RPDS subtracted, 24-bit.
function rawOf(s: State, hPa: number): number {
  const p = s.autozero ? hPa - s.int_ref_hPa : hPa;
  const raw = Math.round(p * s.fs_lsb_per_hpa) - s.pressure_offset_lsb;
  return Math.max(-0x800000, Math.min(0x7fffff, raw));
}
// The driver's integer conversion (C division truncates toward zero).
const mhpaOf = (s: State, raw: number) =>
  Math.trunc((raw * 125) / (s.fs_lsb_per_hpa === 2048 ? 256 : 512));
const tempRaw = (s: State) => Math.max(-32768, Math.min(32767, Math.round(s.sampled_temp_c * 100)));

// Threshold comparison: against REF_P when a reference is captured, else absolute.
function thresholdHits(s: State, p: number): { high: boolean; low: boolean } {
  const phe = (s.int_cfg & INT_PHE) !== 0;
  const ple = (s.int_cfg & INT_PLE) !== 0;
  if (s.int_ref_hPa === 0) {
    return { high: phe && p > s.int_threshold_hPa, low: ple && p < s.int_threshold_hPa };
  }
  const d = p - s.int_ref_hPa;
  return { high: phe && d > s.int_threshold_hPa, low: ple && -d > s.int_threshold_hPa };
}

const sim: TileSim<State> = {
  tile: 'Sense.BP',

  // After tile_sense_bp_init(…, NULL): 25 Hz, AVG 4, mode 1, LPF on at ODR/4.
  defaultState: {
    pressure_hPa: 1013.25,
    temperature_c: 22,

    sampled_pressure_hPa: 1013.25,
    sampled_temp_c: 22,
    last_sample_ms: 0,
    pressure_drdy: 0,
    temp_drdy: 0,
    oneshot_pending: 0,

    odr_hz: 25,
    avg_n: 4,
    fs_lsb_per_hpa: 4096,
    lpf_alpha: LPF_ALPHA_ODR_4,
    sleeping: 0,
    pressure_offset_lsb: 0,

    fifo_mode: 0,
    fifo_data: '',
    fifo_watermark: 0,

    int_threshold_hPa: 0,
    int_cfg: 0,
    int_ref_hPa: 0,
    autozero: 0,
    int_latched: 0,
    int_prev_high: false,
    int_prev_low: false,

    vplus_mv: 3300,
  },

  // Legacy SimulatorPane drift: deltas on the dialed pressure / temperature.
  automatic: ({ t }) => {
    const s = t / 1000;
    return {
      pressure_hPa: Math.sin(s * (Math.PI / 6)) * 6,
      temperature_c: Math.sin(s * (Math.PI / 15) + 1.0) * 4,
    };
  },

  controls: [
    {
      type: 'slider',
      field: 'pressure_hPa',
      label: 'Pressure',
      min: 260,
      max: 1260,
      step: 0.1,
      unit: 'hPa',
      description:
        'Air pressure (sea level ≈ 1013.25 hPa). The driver reads it through the chip: at the ODR, averaged and low-pass filtered.',
    },
    {
      type: 'slider',
      field: 'temperature_c',
      label: 'Temperature',
      min: -40,
      max: 85,
      step: 0.5,
      unit: '°C',
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_sense_bp_find: () => ({ scalar: 1 }),
    tile_sense_bp_init: () => ({ scalar: 0 }),
    tile_sense_bp_sleep: () => ({ nextState: { sleeping: 1 } }),
    tile_sense_bp_wake: () => ({ nextState: { sleeping: 0 } }),
    // SWRESET: registers to their reset values — power-down, LPF off, mode 1,
    // FIFO bypass, interrupts and references cleared.
    tile_sense_bp_reset: () => ({
      nextState: {
        odr_hz: 0,
        avg_n: 4,
        fs_lsb_per_hpa: 4096,
        lpf_alpha: 1,
        sleeping: 0,
        pressure_offset_lsb: 0,
        fifo_mode: 0,
        fifo_data: '',
        fifo_watermark: 0,
        pressure_drdy: 0,
        temp_drdy: 0,
        oneshot_pending: 0,
        int_cfg: 0,
        int_threshold_hPa: 0,
        int_ref_hPa: 0,
        autozero: 0,
        int_latched: 0,
        int_prev_high: false,
        int_prev_low: false,
      },
    }),

    // ── configuration (set_odr / set_avg rewrite CTRL_REG1 → a live ODR) ──
    tile_sense_bp_set_odr: ({ args }) => ({
      nextState: { odr_hz: ODR_HZ[num(args, 0, 4) & 0x0f] ?? 0, sleeping: 0 },
    }),
    tile_sense_bp_set_avg: ({ args }) => ({
      nextState: { avg_n: AVG_N[num(args, 0, 0) & 0x07] ?? 4, sleeping: 0 },
    }),
    tile_sense_bp_set_fullscale: ({ args }) => ({
      nextState: { fs_lsb_per_hpa: num(args, 0, 0) ? 2048 : 4096 },
    }),
    tile_sense_bp_set_lpf: ({ args }) => ({
      nextState: {
        lpf_alpha: num(args, 0, 0) === 0 ? 1 : num(args, 1, 0) ? LPF_ALPHA_ODR_9 : LPF_ALPHA_ODR_4,
      },
    }),
    tile_sense_bp_oneshot: () => ({ nextState: { oneshot_pending: 1 } }),

    // ── output registers (reading clears the matching DRDY) ──
    tile_sense_bp_get_pressure_raw: ({ state }) => ({
      scalar: rawOf(state, state.sampled_pressure_hPa),
      nextState: { pressure_drdy: 0 },
    }),
    tile_sense_bp_get_pressure_mhpa: ({ state }) => ({
      scalar: mhpaOf(state, rawOf(state, state.sampled_pressure_hPa)),
      nextState: { pressure_drdy: 0 },
    }),
    tile_sense_bp_get_temp_raw: ({ state }) => ({
      scalar: tempRaw(state),
      nextState: { temp_drdy: 0 },
    }),
    tile_sense_bp_get_temp_cdeg: ({ state }) => ({
      scalar: tempRaw(state),
      nextState: { temp_drdy: 0 },
    }),
    tile_sense_bp_get_status: ({ state }) => ({
      scalar: (state.pressure_drdy ? 0x01 : 0) | (state.temp_drdy ? 0x02 : 0),
    }),
    tile_sense_bp_pressure_ready: ({ state }) => ({ scalar: state.pressure_drdy ? 1 : 0 }),
    tile_sense_bp_temp_ready: ({ state }) => ({ scalar: state.temp_drdy ? 1 : 0 }),

    // ── FIFO ──
    tile_sense_bp_set_fifo_mode: ({ state, args }) => {
      const m = num(args, 0, 0);
      const mode = [0x00, 0x01, 0x02, 0x05, 0x06, 0x07].includes(m) ? m : 0;
      // Bypass empties the FIFO.
      return {
        nextState:
          mode === 0 && state.fifo_data !== ''
            ? { fifo_mode: 0, fifo_data: '' }
            : { fifo_mode: mode },
      };
    },
    tile_sense_bp_set_fifo_watermark: ({ args }) => ({
      nextState: { fifo_watermark: num(args, 0, 0) & 0x7f },
    }),
    tile_sense_bp_get_fifo_level: ({ state }) => ({ scalar: fifoParse(state.fifo_data).length }),
    tile_sense_bp_get_fifo_status: ({ state }) => {
      const level = fifoParse(state.fifo_data).length;
      const full = level >= FIFO_DEPTH;
      return {
        scalar:
          (state.fifo_watermark > 0 && level >= state.fifo_watermark ? 0x80 : 0) |
          (full && state.fifo_mode === 0x01 ? 0x40 : 0) |
          (full ? 0x20 : 0),
      };
    },
    // Pops the oldest sample; an empty FIFO reads the current output instead.
    tile_sense_bp_read_fifo_raw: ({ state }) => {
      const buf = fifoParse(state.fifo_data);
      if (buf.length === 0) return { scalar: rawOf(state, state.sampled_pressure_hPa) };
      return { scalar: buf[0]!, nextState: { fifo_data: fifoSerialize(buf.slice(1)) } };
    },
    // Drains up to `count` samples (never more than the FIFO holds); the count read
    // is returned.
    tile_sense_bp_read_fifo_batch: ({ state, caps }) => {
      const buf = fifoParse(state.fifo_data);
      const n = Math.min(buf.length, Math.max(0, caps?.buf ?? 0), 0xff);
      return {
        out: { buf: buf.slice(0, n) },
        nextState: { fifo_data: fifoSerialize(buf.slice(n)) },
      };
    },

    // ── interrupt ──
    tile_sense_bp_set_threshold_hpa: ({ args }) => ({
      nextState: { int_threshold_hPa: num(args, 0, 0) & 0xffff },
    }),
    tile_sense_bp_set_interrupt_cfg: ({ state, args }) => {
      const cfg = num(args, 0, 0) & 0xff;
      const next: Partial<State> = { int_cfg: cfg };
      if (cfg & (INT_AUTOZERO | INT_AUTOREFP)) next.int_ref_hPa = state.sampled_pressure_hPa;
      if (cfg & INT_AUTOZERO) next.autozero = 1;
      if (cfg & (INT_RESET_AZ | INT_RESET_ARP)) {
        next.int_ref_hPa = 0;
        next.autozero = 0;
      }
      if (!(cfg & INT_LIR)) next.int_latched = 0;
      return { nextState: next };
    },
    // INT_SOURCE: PH / PL / IA, cleared on read; BOOT_ON is 0 once booted.
    tile_sense_bp_get_int_source: ({ state }) => {
      const phpl = state.int_latched & 0x03;
      return { scalar: phpl | (phpl ? 0x04 : 0), nextState: { int_latched: 0 } };
    },
    tile_sense_bp_is_boot_complete: () => ({ scalar: 1 }),

    // ── reference / offset ──
    tile_sense_bp_set_autozero: ({ state }) => ({
      nextState: {
        int_cfg: state.int_cfg | INT_AUTOZERO,
        int_ref_hPa: state.sampled_pressure_hPa,
        autozero: 1,
      },
    }),
    tile_sense_bp_reset_autozero: ({ state }) => ({
      nextState: { int_cfg: state.int_cfg & ~INT_AUTOZERO, int_ref_hPa: 0, autozero: 0 },
    }),
    tile_sense_bp_set_autorefp: ({ state }) => ({
      nextState: { int_cfg: state.int_cfg | INT_AUTOREFP, int_ref_hPa: state.sampled_pressure_hPa },
    }),
    tile_sense_bp_reset_autorefp: ({ state }) => ({
      nextState: { int_cfg: state.int_cfg & ~INT_AUTOREFP, int_ref_hPa: 0 },
    }),
    tile_sense_bp_set_pressure_offset: ({ args }) => ({
      nextState: { pressure_offset_lsb: ((num(args, 0, 0) & 0xffff) << 16) >> 16 },
    }),
    // REF_P holds the upper 16 bits of the 24-bit pressure word.
    tile_sense_bp_get_ref_pressure: ({ state }) => {
      const v = Math.round((state.int_ref_hPa * state.fs_lsb_per_hpa) / 256);
      return { scalar: Math.max(-32768, Math.min(32767, v)) };
    },

    // ── tier-2 helpers ──
    tile_sense_bp_read_altitude_mm: ({ state, args }) => {
      const seaPa = num(args, 0, 101325);
      const pa = Math.trunc(mhpaOf(state, rawOf(state, state.sampled_pressure_hPa)) / 10);
      return { scalar: Math.trunc((8430 * (seaPa - pa)) / 100) };
    },
    // Polls for a change from the call-time reading; the dial is constant within
    // a call, so "has the world moved that far from the last output?" is the answer.
    tile_sense_bp_wait_for_pressure_change: ({ state, args }) => {
      const threshold = num(args, 0, 0) & 0xffff;
      if (num(args, 1, 0) <= 0) return { scalar: 0 }; // no poll happens at timeout 0
      return {
        scalar: Math.abs(state.pressure_hPa - state.sampled_pressure_hPa) >= threshold ? 1 : 0,
      };
    },
  },

  provenance: {
    tile_sense_bp_find: 'canonical', // WHO_AM_I 0xB4 at 0x5D
    tile_sense_bp_init: 'canonical', // NULL cfg → 25 Hz / AVG 4 / mode 1 / LPF ODR/4
    tile_sense_bp_sleep: 'canonical',
    tile_sense_bp_wake: 'canonical',
    tile_sense_bp_reset: 'inferred', // register reset values without the datasheet
    tile_sense_bp_set_odr: 'canonical',
    tile_sense_bp_set_avg: 'canonical',
    tile_sense_bp_set_fullscale: 'canonical', // 4096 / 2048 LSB/hPa
    tile_sense_bp_set_lpf: 'inferred', // single-pole stand-in for the chip's filter
    tile_sense_bp_oneshot: 'canonical',
    tile_sense_bp_get_pressure_raw: 'canonical',
    tile_sense_bp_get_pressure_mhpa: 'canonical', // raw·125/512 (or /256), truncated
    tile_sense_bp_get_temp_raw: 'canonical', // 100 LSB/°C
    tile_sense_bp_get_temp_cdeg: 'canonical',
    tile_sense_bp_get_status: 'canonical', // P_DA bit 0, T_DA bit 1
    tile_sense_bp_pressure_ready: 'canonical',
    tile_sense_bp_temp_ready: 'canonical',
    tile_sense_bp_set_fifo_mode: 'canonical',
    tile_sense_bp_set_fifo_watermark: 'canonical',
    tile_sense_bp_get_fifo_level: 'canonical',
    tile_sense_bp_get_fifo_status: 'inferred', // WTM / OVR / FULL bits; OVR only in FIFO mode
    tile_sense_bp_read_fifo_raw: 'inferred', // empty-FIFO read behavior guessed
    tile_sense_bp_read_fifo_batch: 'canonical', // clipped to level, returns count
    tile_sense_bp_set_threshold_hpa: 'canonical',
    tile_sense_bp_set_interrupt_cfg: 'inferred',
    tile_sense_bp_get_int_source: 'canonical',
    tile_sense_bp_is_boot_complete: 'inferred', // no boot transient modeled
    tile_sense_bp_set_autozero: 'canonical', // outputs become P − REF_P
    tile_sense_bp_reset_autozero: 'canonical',
    tile_sense_bp_set_autorefp: 'canonical',
    tile_sense_bp_reset_autorefp: 'canonical',
    tile_sense_bp_set_pressure_offset: 'inferred', // RPDS applied as a raw subtraction
    tile_sense_bp_get_ref_pressure: 'inferred', // REF_P = upper 16 bits of the 24-bit word
    tile_sense_bp_read_altitude_mm: 'canonical', // driver's linear 8.43 mm/Pa
    tile_sense_bp_wait_for_pressure_change: 'inferred',
    power: 'hallucinated', // no datasheet on file
  },

  // Each tick: at the ODR (or on a pending one-shot) take a sample through the
  // averaging + LPF chain, raise DRDY, push the FIFO; then track the interrupt.
  deriveState(state, { t }) {
    const update: Partial<State> = {};
    const running = state.odr_hz > 0 && state.sleeping !== 1;
    const periodMs = running ? 1000 / state.odr_hz : 0;
    const due = (running && t - state.last_sample_ms >= periodMs) || state.oneshot_pending === 1;
    if (due) {
      const seed = Math.floor(t / Math.max(periodMs, 1));
      const noise = pseudoGaussian(seed) * (NOISE_HPA_AVG4 / Math.sqrt(state.avg_n / 4));
      const a = state.lpf_alpha;
      const p = a * (state.pressure_hPa + noise) + (1 - a) * state.sampled_pressure_hPa;
      const tc = a * state.temperature_c + (1 - a) * state.sampled_temp_c;
      update.sampled_pressure_hPa = p;
      update.sampled_temp_c = tc;
      update.last_sample_ms = t;
      update.pressure_drdy = 1;
      update.temp_drdy = 1;
      if (state.oneshot_pending) update.oneshot_pending = 0;
      if (FIFO_FILLING.has(state.fifo_mode)) {
        const buf = fifoParse(state.fifo_data);
        const raw = rawOf(state, p);
        if (state.fifo_mode === 0x01) {
          if (buf.length < FIFO_DEPTH) buf.push(raw);
        } else {
          buf.push(raw);
          while (buf.length > FIFO_DEPTH) buf.shift();
        }
        update.fifo_data = fifoSerialize(buf);
      }
    }

    // Threshold interrupt: latched edges when LIR is set, else live.
    if (!(state.int_cfg & INT_LIR)) {
      if (state.int_latched !== 0 || state.int_prev_high || state.int_prev_low) {
        update.int_latched = 0;
        update.int_prev_high = false;
        update.int_prev_low = false;
      }
    } else {
      const hit = thresholdHits(state, update.sampled_pressure_hPa ?? state.sampled_pressure_hPa);
      let latched = state.int_latched;
      if (hit.high && !state.int_prev_high) latched |= 0x01;
      if (hit.low && !state.int_prev_low) latched |= 0x02;
      if (latched !== state.int_latched) update.int_latched = latched;
      if (hit.high !== state.int_prev_high) update.int_prev_high = hit.high;
      if (hit.low !== state.int_prev_low) update.int_prev_low = hit.low;
    }
    return update;
  },

  // INT (pad 9): the latched event with LIR, else the live threshold comparison.
  padOutputs(state) {
    if (state.int_cfg & INT_LIR) return { '9': state.int_latched !== 0 ? 1 : 0 };
    const hit = thresholdHits(state, state.sampled_pressure_hPa);
    return { '9': hit.high || hit.low ? 1 : 0 };
  },

  // Electrical: V+ (pad 10). Draw grows with ODR × averaging; power-down floor.
  power(state, ctx) {
    const vdd = ctx ? (ctx.padVoltage['10'] ?? 0) : state.vplus_mv;
    const odr = state.sleeping === 1 ? 0 : state.odr_hz;
    const draw =
      vdd < VDD_MIN_MV
        ? 0
        : IDD_PD_UA + Math.min(IDD_MAX_UA, odr * IDD_PER_HZ_AVG4_UA * (state.avg_n / 4));
    return {
      draw_ua: draw,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: vdd,
          i_ua: draw,
          pads: ['10'],
          note: vdd < VDD_MIN_MV ? 'unpowered' : odr > 0 ? `sampling at ${odr} Hz` : 'power-down',
        },
      ],
    };
  },
};

export default sim;
