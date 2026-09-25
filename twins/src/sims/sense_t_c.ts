// Digital twin for Sense.T.C — Azoteq IQS323 ProxFusion capacitive touch
// (driver tile_sense_t_c.{h,c}; IQS323 datasheet v1.11).
//
// A counts-based charge-transfer cap sensor (NOT a femtofarad CDC — it reports
// filtered conversion counts vs a long-term-average baseline, never absolute
// capacitance). This is a DOUBLE-SIDED tile: the IC + passives are on the
// bottom; the ENTIRE TOP SURFACE is the self-cap electrode for channel CH1 (via
// a 470Ω series R), and CH0 is a second external electrode brought out to pad 8.
// CH2 exists in the chip but is not routed on this tile, and init disables it.
// A single surface cannot form a slider, so slider and gesture reads stay at rest.
//
// Pad map (Sense-T-C-a.json): GND (1), RDY (3, open-drain output, on-tile 4.7k
// pull-up and 100 nF to GND), I2C (4/5), C0 external electrode (8), V+ (10).
//
// Model: a finger near / on an electrode (the controls) lowers that channel's
// counts — self-capacitance counts DECREASE with touch (datasheet §5.7) — and the
// chip compares (LTA − counts) against the channel's prox threshold (absolute
// counts, A.16) and touch threshold (threshold/256 × LTA, A.17).
// The LTA follows the counts while a channel is idle; ATI / reseed snap it.
//
// The driver CACHES the System Status word: process() reads it, and
// get_status / is_touched / is_prox / is_touched_any report that cached copy —
// so a program that never calls process() never sees a touch, exactly as on
// hardware. Counts, LTA, delta, slider and gestures are live register reads.
import type { PowerCtx, TileSim } from '../tileSim';

interface State {
  // ── the world (controls) ──
  ch0_touched: number; // finger on C0 (pad 8)
  ch1_touched: number; // finger on the top surface
  ch0_prox: number; // finger near C0, not touching
  ch1_prox: number; // hand near the surface

  // ── the chip's data ──
  ch0_lta: number; // long-term average (baseline), counts
  ch1_lta: number;
  ch2_lta: number;
  status_flags: number; // latched event bits (PROX_EVENT / TOUCH_EVENT), cleared by a read
  prev_ch_bits: number; // per-channel prox/touch bits at the last tick (edge memory)
  int_latched: number; // an enabled event is pending (event mode RDY)
  /** Effective power mode (0 NP … 3 Halt): power_mode, or where AUTO has got to. */
  active_mode: number;
  idle_since_t: number; // deriveState clock: when the last prox/touch ended

  // ── the driver's cache ──
  last_status: number; // System Status as of the last process()
  ready: number; // TILE_STATE_READY: process() does nothing otherwise
  sleeping: number;

  // ── configuration (System Control / per-channel registers) ──
  power_mode: number; // sense_t_c_power_mode_t: 0 NP, 1 LP, 2 ULP, 3 Halt, 4 Auto, 5 Auto-no-ULP
  comm_mode: number; // 0 streaming, 1 event (System Control bit 7)
  events_enable: number; // REG 0xD3
  counts_filter: number; // REG 0xB0
  np_rate_ms: number;
  lp_rate_ms: number;
  ulp_rate_ms: number;
  halt_rate_ms: number;
  power_timeout_ms: number;
  ch0_prox_th: number;
  ch0_touch_th: number;
  ch0_ati_setup: number;
  ch0_conv_freq: number;
  ch0_mode: number;
  ch0_ref_id: number;
  ch0_compensation: number;
  ch1_prox_th: number;
  ch1_touch_th: number;
  ch1_ati_setup: number;
  ch1_conv_freq: number;
  ch1_mode: number;
  ch1_ref_id: number;
  ch1_compensation: number;
  ch2_prox_th: number;
  ch2_touch_th: number;
  ch2_ati_setup: number;
  ch2_conv_freq: number;
  ch2_mode: number;
  ch2_ref_id: number;
  ch2_compensation: number;

  // write_reg / read_reg echo
  last_reg: number;
  last_reg_value: number;
}

const CHANNELS = 3;

// System Status bits (IQS323_STATUS_*).
const ST_PROX_EVENT = 1 << 0;
const ST_TOUCH_EVENT = 1 << 1;
const chProxBit = (ch: number) => 1 << (8 + ch * 2);
const chTouchBit = (ch: number) => 1 << (9 + ch * 2);
const CH_BITS_MASK = 0x3f00;

const HW_ID_3DD = 0xf003;

// Modeled counts: ATI settles the working point near its target (512 in the
// datasheet's self-cap setup); a finger pulls counts down.
const COUNTS_REST = 512;
const PROX_DROP = 30;
const TOUCH_DROP = 110;
// Nominal compensation (divider 31, value ~934), per the Azoteq EV-kit.
const COMPENSATION_NOMINAL = (31 << 11) | 934;

// Power-mode codes.
const PM_NORMAL = 0;
const PM_LOW = 1;
const PM_ULP = 2;
const PM_HALT = 3;
const PM_AUTO = 4;
const PM_AUTO_NO_ULP = 5;
// Typical current, self-capacitive (3 ch), 3.3 V — datasheet §3.4 table.
const MODE_UA = [125, 37.5, 4, 2];
const MODE_NAME = ['NP', 'LP', 'ULP', 'Halt'];

type ChKey<F extends string> = `ch${0 | 1 | 2}_${F}`;
const key = <F extends string>(ch: number, f: F) => `ch${ch}_${f}` as ChKey<F>;

/** What the electrode physically does to the counts. CH2 is not routed. */
function countsOf(s: State, ch: number): number {
  if (ch === 0) return COUNTS_REST - (s.ch0_touched ? TOUCH_DROP : s.ch0_prox ? PROX_DROP : 0);
  if (ch === 1) return COUNTS_REST - (s.ch1_touched ? TOUCH_DROP : s.ch1_prox ? PROX_DROP : 0);
  return COUNTS_REST;
}
const ltaOf = (s: State, ch: number) => s[key(ch, 'lta')];
/** (LTA − counts): positive with a finger (self-cap, non-inverted logic). */
const drop = (s: State, ch: number) => ltaOf(s, ch) - countsOf(s, ch);

/** Per-channel prox / touch bits, from the thresholds (datasheet §5.7, A.16,
 * A.17): prox is absolute counts, touch is threshold/256 of the LTA. */
function channelBits(s: State): number {
  let bits = 0;
  // CH2 is disabled by init (no electrode), so it never reports.
  for (let ch = 0; ch < 2; ch++) {
    const d = drop(s, ch);
    if (d > s[key(ch, 'prox_th')]) bits |= chProxBit(ch);
    if (d > Math.floor((s[key(ch, 'touch_th')] * ltaOf(s, ch)) / 256)) bits |= chTouchBit(ch);
  }
  return bits;
}
/** The live System Status register (0x10). */
const statusWord = (s: State) => (s.status_flags & 0xff) | channelBits(s);

/** process(): read the status into the driver's cache; the read clears the
 * chip's event flags (and its RDY). Nothing happens unless the tile is READY. */
function process(s: State): Partial<State> {
  if (!s.ready) return {};
  return { last_status: statusWord(s), status_flags: 0, int_latched: 0 };
}

const arg = (args: number[], i: number, fallback: number) =>
  args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;
const chArg = (args: number[]) => arg(args, 0, 0);
const validCh = (ch: number) => ch >= 0 && ch < CHANNELS;

const sim: TileSim<State> = {
  tile: 'Sense.T.C',

  // State AFTER init with no cfg: soft reset + ACK, CH0 / CH1 on their own
  // electrodes and CH2 off, Azoteq's EV-kit settings (prox 20, touch 40,
  // counts filter 0x0202, NP 16 / LP 60 / ULP 160 / Halt 3000 ms, timeout
  // 2000 ms), events = TOUCH | PROX, AUTO power, re-ATI, then EVENT mode.
  defaultState: {
    ch0_touched: 0,
    ch1_touched: 0,
    ch0_prox: 0,
    ch1_prox: 0,

    ch0_lta: COUNTS_REST,
    ch1_lta: COUNTS_REST,
    ch2_lta: COUNTS_REST,
    status_flags: 0,
    prev_ch_bits: 0,
    int_latched: 0,
    active_mode: PM_NORMAL,
    idle_since_t: 0,

    last_status: 0,
    ready: 1,
    sleeping: 0,

    power_mode: PM_AUTO,
    comm_mode: 1,
    events_enable: ST_TOUCH_EVENT | ST_PROX_EVENT,
    counts_filter: 0x0202,
    np_rate_ms: 16,
    lp_rate_ms: 60,
    ulp_rate_ms: 160,
    halt_rate_ms: 3000,
    power_timeout_ms: 2000,
    ch0_prox_th: 20,
    ch0_touch_th: 40,
    ch0_ati_setup: 0,
    ch0_conv_freq: 0,
    ch0_mode: 0,
    ch0_ref_id: 0,
    ch0_compensation: COMPENSATION_NOMINAL,
    ch1_prox_th: 20,
    ch1_touch_th: 40,
    ch1_ati_setup: 0,
    ch1_conv_freq: 0,
    ch1_mode: 0,
    ch1_ref_id: 0,
    ch1_compensation: COMPENSATION_NOMINAL,
    ch2_prox_th: 20,
    ch2_touch_th: 40,
    ch2_ati_setup: 0,
    ch2_conv_freq: 0,
    ch2_mode: 0,
    ch2_ref_id: 0,
    ch2_compensation: COMPENSATION_NOMINAL,

    last_reg: 0,
    last_reg_value: 0,
  },

  controls: [
    { type: 'toggle', field: 'ch1_touched', label: 'Touch top surface (CH1)' },
    { type: 'toggle', field: 'ch0_touched', label: 'Touch C0 (pad 8)' },
    { type: 'toggle', field: 'ch1_prox', label: 'Approach surface (prox)' },
    { type: 'toggle', field: 'ch0_prox', label: 'Approach C0 (prox)' },
  ],

  // What a finger does: approach or touch either electrode.
  stimuli: [
    {
      id: 'finger',
      label: 'Finger',
      controls: [
        {
          kind: 'toggle',
          id: 'touch_surface',
          label: 'Touch top surface',
          fields: ['ch1_touched'],
        },
        { kind: 'toggle', id: 'near_surface', label: 'Hand near surface', fields: ['ch1_prox'] },
        { kind: 'toggle', id: 'touch_c0', label: 'Touch pad 8 electrode', fields: ['ch0_touched'] },
        { kind: 'toggle', id: 'near_c0', label: 'Near pad 8 electrode', fields: ['ch0_prox'] },
      ],
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_sense_t_c_find: () => ({ scalar: 1 }),
    tile_sense_t_c_init: () => ({ scalar: 0 }),
    tile_sense_t_c_process: ({ state }) => ({ nextState: process(state) }),
    // The callback fires from process(); nothing to hold in state.
    tile_sense_t_c_on_event: () => undefined,
    // Halt; power_mode keeps the host's choice, which wake() restores.
    tile_sense_t_c_sleep: () => ({
      nextState: { sleeping: 1, ready: 0 },
    }),
    tile_sense_t_c_wake: () => ({
      nextState: { sleeping: 0, ready: 1 },
    }),

    // ── status: the driver's cached copy ──
    tile_sense_t_c_get_status: ({ state }) => ({ scalar: state.last_status }),
    tile_sense_t_c_is_touched: ({ state, args }) => {
      const ch = chArg(args);
      return { scalar: validCh(ch) && state.last_status & chTouchBit(ch) ? 1 : 0 };
    },
    tile_sense_t_c_is_prox: ({ state, args }) => {
      const ch = chArg(args);
      return { scalar: validCh(ch) && state.last_status & chProxBit(ch) ? 1 : 0 };
    },
    tile_sense_t_c_is_touched_any: ({ state }) => ({
      scalar: state.last_status & (chTouchBit(0) | chTouchBit(1) | chTouchBit(2)) ? 1 : 0,
    }),
    // process(), then is_touched_any, polled until timeout. The world holds
    // still during the call, so one pass answers it.
    tile_sense_t_c_wait_for_touch: ({ state }) => {
      if (!state.ready) return { scalar: 0 };
      const next = process(state);
      const st = next.last_status ?? state.last_status;
      return {
        scalar: st & (chTouchBit(0) | chTouchBit(1) | chTouchBit(2)) ? 1 : 0,
        nextState: next,
      };
    },

    // ── live register reads ──
    tile_sense_t_c_get_counts: ({ state, args }) => {
      const ch = chArg(args);
      return { scalar: validCh(ch) ? countsOf(state, ch) : 0 };
    },
    tile_sense_t_c_get_lta: ({ state, args }) => {
      const ch = chArg(args);
      return { scalar: validCh(ch) ? ltaOf(state, ch) : 0 };
    },
    // counts − LTA as int16: NEGATIVE with a finger on a self-cap channel.
    tile_sense_t_c_get_delta: ({ state, args }) => {
      const ch = chArg(args);
      if (!validCh(ch)) return { scalar: 0 };
      return { scalar: (((countsOf(state, ch) - ltaOf(state, ch)) & 0xffff) << 16) >> 16 };
    },
    // No slider / gestures on a single surface: the registers sit at rest.
    tile_sense_t_c_get_slider: () => ({ scalar: 0 }),
    // Slider Resolution (0x93) is 0 until a slider is configured, so the driver
    // returns 0 and leaves out_pct alone.
    tile_sense_t_c_read_slider_pct: () => ({ scalar: 0 }),
    tile_sense_t_c_get_gestures: () => ({ scalar: 0 }),
    tile_sense_t_c_wait_for_gesture: () => ({ scalar: 0 }),

    // ── configuration ──
    // The driver writes each threshold only when non-zero.
    tile_sense_t_c_set_thresholds: ({ args }) => {
      const ch = chArg(args);
      if (!validCh(ch)) return;
      const prox = arg(args, 1, 0) & 0xff;
      const touch = arg(args, 2, 0) & 0xff;
      const next: Partial<State> = {};
      if (prox) next[key(ch, 'prox_th')] = prox;
      if (touch) next[key(ch, 'touch_th')] = touch;
      return { nextState: next };
    },
    // All enabled channels (CH0, CH1); 0 is ignored.
    tile_sense_t_c_set_touch_threshold: ({ args }) => {
      const th = arg(args, 0, 0) & 0xff;
      return th ? { nextState: { ch0_touch_th: th, ch1_touch_th: th } } : undefined;
    },
    tile_sense_t_c_set_prox_threshold: ({ args }) => {
      const th = arg(args, 0, 0) & 0xff;
      return th ? { nextState: { ch0_prox_th: th, ch1_prox_th: th } } : undefined;
    },
    // Modes 6 and 7 are reserved; the driver ignores them.
    tile_sense_t_c_set_power_mode: ({ state, args }) => {
      const mode = arg(args, 0, state.power_mode);
      return mode >= PM_NORMAL && mode <= PM_AUTO_NO_ULP
        ? { nextState: { power_mode: mode } }
        : undefined;
    },
    // Events Enable: only bits 0-4 and 6 are defined (A.33).
    tile_sense_t_c_enable_events: ({ args }) => ({
      nextState: { events_enable: arg(args, 0, 0) & 0x5f },
    }),
    // Re-ATI: the chip re-tunes and reseeds, so every LTA lands on its counts.
    tile_sense_t_c_ati: ({ state }) => ({
      nextState: {
        ch0_lta: countsOf(state, 0),
        ch1_lta: countsOf(state, 1),
        ch2_lta: countsOf(state, 2),
      },
    }),
    tile_sense_t_c_reseed: ({ state }) => ({
      nextState: {
        ch0_lta: countsOf(state, 0),
        ch1_lta: countsOf(state, 1),
        ch2_lta: countsOf(state, 2),
      },
    }),
    tile_sense_t_c_set_ati_setup: ({ args }) => {
      const ch = chArg(args);
      if (!validCh(ch)) return;
      return { nextState: { [key(ch, 'ati_setup')]: arg(args, 1, 0) & 0xffff } };
    },
    tile_sense_t_c_set_counts_filter: ({ args }) => ({
      nextState: { counts_filter: arg(args, 0, 0) & 0xffff },
    }),
    tile_sense_t_c_set_conversion_freq: ({ args }) => {
      const ch = chArg(args);
      if (!validCh(ch)) return;
      return { nextState: { [key(ch, 'conv_freq')]: arg(args, 1, 0) & 0xffff } };
    },
    tile_sense_t_c_set_channel_mode: ({ args }) => {
      const ch = chArg(args);
      if (!validCh(ch)) return;
      return {
        nextState: {
          [key(ch, 'mode')]: arg(args, 1, 0) & 0x0f,
          [key(ch, 'ref_id')]: arg(args, 2, 0) & 0x0f,
        },
      };
    },
    tile_sense_t_c_set_comm_mode: ({ args }) => ({
      nextState: { comm_mode: arg(args, 0, 0) ? 1 : 0 },
    }),
    // AUTO / AUTO_NO_ULP have no rate register; the driver caps at 3000 ms.
    tile_sense_t_c_set_report_rate: ({ args }) => {
      const mode = arg(args, 0, PM_NORMAL);
      const ms = Math.min(3000, arg(args, 1, 16) & 0xffff);
      const field =
        mode === PM_NORMAL
          ? 'np_rate_ms'
          : mode === PM_LOW
            ? 'lp_rate_ms'
            : mode === PM_ULP
              ? 'ulp_rate_ms'
              : mode === PM_HALT
                ? 'halt_rate_ms'
                : undefined;
      return field ? { nextState: { [field]: ms } } : undefined;
    },
    tile_sense_t_c_set_power_timeout: ({ args }) => ({
      nextState: { power_timeout_ms: Math.min(65000, arg(args, 0, 2000) & 0xffff) },
    }),
    tile_sense_t_c_get_compensation: ({ state, args }) => {
      const ch = chArg(args);
      return { scalar: validCh(ch) ? state[key(ch, 'compensation')] : 0 };
    },
    // Compensation (A.14): [15:11] divider, [9:0] value.
    tile_sense_t_c_set_compensation: ({ args }) => {
      const ch = chArg(args);
      if (!validCh(ch)) return;
      const value = ((arg(args, 2, 0) & 0x1f) << 11) | (arg(args, 1, 0) & 0x03ff);
      return { nextState: { [key(ch, 'compensation')]: value } };
    },

    // ── low-level ──
    tile_sense_t_c_read_reg: ({ state, args }) => {
      const reg = arg(args, 0, 0) & 0xff;
      if (reg === 0x10) return { scalar: statusWord(state) };
      if (reg === 0x11 || reg === 0x12) return { scalar: 0 }; // gestures / slider at rest
      if (reg === 0xe1) return { scalar: HW_ID_3DD };
      if (reg === state.last_reg) return { scalar: state.last_reg_value };
      return { scalar: 0 };
    },
    tile_sense_t_c_write_reg: ({ args }) => ({
      nextState: { last_reg: arg(args, 0, 0) & 0xff, last_reg_value: arg(args, 1, 0) & 0xffff },
    }),
  },

  provenance: {
    tile_sense_t_c_find: 'canonical', // ACK at 0x44 (order code 001)
    tile_sense_t_c_process: 'canonical', // caches System Status, READY-gated
    tile_sense_t_c_get_status: 'canonical', // cached word, bit layout per §A.2
    tile_sense_t_c_is_touched: 'canonical',
    tile_sense_t_c_is_prox: 'canonical',
    tile_sense_t_c_is_touched_any: 'canonical',
    tile_sense_t_c_get_lta: 'canonical',
    tile_sense_t_c_get_delta: 'canonical', // counts − LTA, int16; negative on self-cap touch
    tile_sense_t_c_set_power_mode: 'canonical', // SYSTEM_CONTROL[6:4]
    tile_sense_t_c_set_comm_mode: 'canonical', // SYSTEM_CONTROL bit 7 (1 = event)
    tile_sense_t_c_set_thresholds: 'canonical', // written only when non-zero; touch is x/256 of LTA
    tile_sense_t_c_set_touch_threshold: 'canonical', // A.17 on CH0 + CH1
    tile_sense_t_c_set_prox_threshold: 'canonical', // A.16 on CH0 + CH1
    tile_sense_t_c_enable_events: 'canonical', // 0xD3, defined bits only
    tile_sense_t_c_reseed: 'canonical', // SYSTEM_CONTROL.RESEED: LTA → counts
    tile_sense_t_c_set_report_rate: 'canonical', // 0xC1-0xC4, ms, capped 3000
    tile_sense_t_c_set_power_timeout: 'canonical', // 0xC5, ms
    tile_sense_t_c_set_compensation: 'canonical', // A.14 encoding
    tile_sense_t_c_get_compensation: 'inferred', // nominal EV-kit working point until set
    // inferred — modeled magnitudes / time collapsed
    tile_sense_t_c_get_counts: 'inferred', // ~512 at rest, finger drops are modeled
    tile_sense_t_c_ati: 'inferred',
    tile_sense_t_c_wait_for_touch: 'inferred',
    // no slider or gestures unless the program configures them (§9: Slider
    // Resolution and Gesture Enable are 0 after reset; init leaves them)
    tile_sense_t_c_get_slider: 'inferred',
    tile_sense_t_c_read_slider_pct: 'inferred', // resolution 0 → returns 0
    tile_sense_t_c_get_gestures: 'inferred',
    tile_sense_t_c_wait_for_gesture: 'inferred',
    // hallucinated — the register file isn't modeled beyond a few registers
    tile_sense_t_c_read_reg: 'hallucinated',
    power: 'inferred', // datasheet §3.4 per-mode current is for 3 self-cap channels at the EV-kit rates; the tile runs 2, and AUTO's stepping is modeled
  },

  // Each tick: idle channels' LTA follows the counts; prox/touch changes latch
  // the event flags (and RDY, when enabled); AUTO power steps NP → LP → ULP after
  // `power_timeout_ms` of no activity (datasheet §6 auto mode), back to NP on any.
  deriveState(state, { t }) {
    const update: Partial<State> = {};
    const bits = channelBits(state);
    for (let ch = 0; ch < CHANNELS; ch++) {
      const active = bits & (chProxBit(ch) | chTouchBit(ch));
      const counts = countsOf(state, ch);
      if (!active && ltaOf(state, ch) !== counts) update[key(ch, 'lta')] = counts;
    }

    if (bits !== state.prev_ch_bits) {
      const changed = bits ^ state.prev_ch_bits;
      let events = 0;
      if (changed & 0x1500) events |= ST_PROX_EVENT; // CHx prox bits
      if (changed & 0x2a00) events |= ST_TOUCH_EVENT; // CHx touch bits
      update.prev_ch_bits = bits & CH_BITS_MASK;
      update.status_flags = state.status_flags | events;
      if (events & state.events_enable) update.int_latched = 1;
    }

    const anyActive = bits !== 0;
    if (anyActive) update.idle_since_t = t;
    let mode = state.power_mode;
    if (state.power_mode === PM_AUTO || state.power_mode === PM_AUTO_NO_ULP) {
      const idle = anyActive ? 0 : t - state.idle_since_t;
      const timeout = state.power_timeout_ms;
      // A timeout of 0 keeps the chip in normal power (datasheet §5.3).
      if (timeout === 0 || idle < timeout) mode = PM_NORMAL;
      else if (idle < 2 * timeout || state.power_mode === PM_AUTO_NO_ULP) mode = PM_LOW;
      else mode = PM_ULP;
    }
    if (mode !== state.active_mode) update.active_mode = mode;
    return update;
  },

  // RDY (pad 3): open-drain, asserted LOW (shown 1 = asserted). In event mode
  // (the driver's init) it asserts only for an enabled event, until the host
  // reads status; in streaming mode the chip opens a window every report cycle,
  // so RDY pulses continuously. Halt / not-ready: quiet.
  padOutputs(state) {
    if (state.sleeping) return { '3': 0 };
    if (state.comm_mode === 0) return { '3': 1 };
    return { '3': state.int_latched ? 1 : 0 };
  },

  // Electrical: a pure load on V+ (pad 10) / GND (pad 1). Draw follows the
  // effective power mode (datasheet §3.4, self-cap; the same at 1.8 and 3.3 V).
  power(state, ctx?: PowerCtx) {
    const mode = state.sleeping
      ? PM_HALT
      : state.power_mode <= PM_HALT
        ? state.power_mode
        : state.active_mode;
    const ua = MODE_UA[mode] ?? MODE_UA[PM_NORMAL]!;
    return {
      draw_ua: ua,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: ctx?.padVoltage?.['10'] ?? 3300,
          i_ua: ua,
          pads: ['10'],
          note: `IQS323 ${MODE_NAME[mode] ?? 'NP'}${!state.sleeping && state.power_mode >= PM_AUTO ? ' (auto)' : ''}`,
        },
      ],
    };
  },
};

export default sim;
