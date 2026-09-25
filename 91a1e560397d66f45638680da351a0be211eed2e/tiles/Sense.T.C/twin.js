// src/sims/sense_t_c.ts
var CHANNELS = 3;
var ST_PROX_EVENT = 1 << 0;
var ST_TOUCH_EVENT = 1 << 1;
var chProxBit = (ch) => 1 << 8 + ch * 2;
var chTouchBit = (ch) => 1 << 9 + ch * 2;
var CH_BITS_MASK = 16128;
var HW_ID_3DD = 61443;
var COUNTS_REST = 512;
var PROX_DROP = 30;
var TOUCH_DROP = 110;
var COMPENSATION_NOMINAL = 31 << 11 | 934;
var PM_NORMAL = 0;
var PM_LOW = 1;
var PM_ULP = 2;
var PM_HALT = 3;
var PM_AUTO = 4;
var PM_AUTO_NO_ULP = 5;
var MODE_UA = [125, 37.5, 4, 2];
var MODE_NAME = ["NP", "LP", "ULP", "Halt"];
var key = (ch, f) => `ch${ch}_${f}`;
function countsOf(s, ch) {
  if (ch === 0) return COUNTS_REST - (s.ch0_touched ? TOUCH_DROP : s.ch0_prox ? PROX_DROP : 0);
  if (ch === 1) return COUNTS_REST - (s.ch1_touched ? TOUCH_DROP : s.ch1_prox ? PROX_DROP : 0);
  return COUNTS_REST;
}
var ltaOf = (s, ch) => s[key(ch, "lta")];
var drop = (s, ch) => ltaOf(s, ch) - countsOf(s, ch);
function channelBits(s) {
  let bits = 0;
  for (let ch = 0; ch < 2; ch++) {
    const d = drop(s, ch);
    if (d > s[key(ch, "prox_th")]) bits |= chProxBit(ch);
    if (d > Math.floor(s[key(ch, "touch_th")] * ltaOf(s, ch) / 256)) bits |= chTouchBit(ch);
  }
  return bits;
}
var statusWord = (s) => s.status_flags & 255 | channelBits(s);
function process(s) {
  if (!s.ready) return {};
  return { last_status: statusWord(s), status_flags: 0, int_latched: 0 };
}
var arg = (args, i, fallback) => args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;
var chArg = (args) => arg(args, 0, 0);
var validCh = (ch) => ch >= 0 && ch < CHANNELS;
var DEFAULT_STATE = {
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
  counts_filter: 514,
  np_rate_ms: 16,
  lp_rate_ms: 60,
  ulp_rate_ms: 160,
  halt_rate_ms: 3e3,
  power_timeout_ms: 2e3,
  ch0_prox_th: 20,
  ch0_touch_th: 40,
  ch0_ati_setup: 1036,
  ch0_conv_freq: 1407,
  ch0_mode: 0,
  ch0_ref_id: 0,
  ch0_compensation: COMPENSATION_NOMINAL,
  ch1_prox_th: 20,
  ch1_touch_th: 40,
  ch1_ati_setup: 1036,
  ch1_conv_freq: 1407,
  ch1_mode: 0,
  ch1_ref_id: 0,
  ch1_compensation: COMPENSATION_NOMINAL,
  ch2_prox_th: 20,
  ch2_touch_th: 40,
  ch2_ati_setup: 1036,
  ch2_conv_freq: 1407,
  ch2_mode: 0,
  ch2_ref_id: 0,
  ch2_compensation: COMPENSATION_NOMINAL,
  ch0_prox_debounce: 68,
  ch1_prox_debounce: 68,
  ch2_prox_debounce: 68,
  ch0_touch_hyst: 0,
  ch1_touch_hyst: 0,
  ch2_touch_hyst: 0,
  reg_file: "{}"
};
var configAfterInit = Object.fromEntries(
  Object.entries(DEFAULT_STATE).filter(
    ([k]) => !/^ch[012]_(touched|prox|lta)$|^(status_flags|prev_ch_bits|int_latched|active_mode|idle_since_t|last_status|ready|sleeping)$/.test(
      k
    )
  )
);
var PRODUCT_NUMBER = 1106;
var VERSION_MAJOR = 1;
var VERSION_MINOR = 3;
var INVALID = 61166;
var SENSOR_PINS = [
  [257, 463],
  [513, 719],
  [0, 463]
];
var INIT_REGS = {
  177: 1285,
  // LTA filter betas (init)
  178: 771,
  // LTA fast filter betas (init)
  180: 30,
  // fast filter band (init)
  209: 200
  // I2C transaction timeout, 200 ms (§9)
};
function regExists(reg) {
  if (reg <= 9) return true;
  if (reg >= 16 && reg <= 24) return true;
  if (reg >= 32 && reg <= 37) return true;
  const lo = reg & 15;
  const hi = reg >> 4;
  if (hi >= 3 && hi <= 5) return lo <= 9;
  if (hi >= 6 && hi <= 8) return lo <= 4;
  if (reg >= 144 && reg <= 153) return true;
  if (reg >= 160 && reg <= 166) return true;
  if (reg >= 176 && reg <= 180) return true;
  if (reg >= 192 && reg <= 197) return true;
  if (reg >= 208 && reg <= 212) return true;
  return reg === 224 || reg === 225;
}
var regFile = (s) => {
  try {
    return JSON.parse(s.reg_file);
  } catch {
    return {};
  }
};
function readReg(s, reg) {
  if (!regExists(reg)) return INVALID;
  const stored = regFile(s)[String(reg)];
  const hi = reg >> 4;
  const lo = reg & 15;
  if (reg === 0) return PRODUCT_NUMBER;
  if (reg === 1) return VERSION_MAJOR;
  if (reg === 2) return VERSION_MINOR;
  if (reg <= 9) return 0;
  if (reg === 16) return statusWord(s);
  if (reg === 17 || reg === 18) return 0;
  if (reg >= 19 && reg <= 24) {
    const ch = reg - 19 >> 1;
    return reg - 19 & 1 ? ltaOf(s, ch) : countsOf(s, ch);
  }
  if (reg >= 32 && reg <= 34) return ltaOf(s, reg - 32);
  if (reg >= 35 && reg <= 37) return 0;
  if (reg === 225) return HW_ID_3DD;
  if (hi >= 3 && hi <= 5) {
    const ch = hi - 3;
    if (lo === 1) return s[key(ch, "conv_freq")];
    if (lo === 6) return s[key(ch, "ati_setup")];
    if (lo === 9) return s[key(ch, "compensation")];
    if (stored !== void 0) return stored;
    if (lo === 0) return SENSOR_PINS[ch][0];
    if (lo === 3) return SENSOR_PINS[ch][1];
    if (lo === 2) return 4752;
    if (lo === 4) return 778;
    if (lo === 7) return 100;
    return 0;
  }
  if (hi >= 6 && hi <= 8) {
    const ch = hi - 6;
    if (lo === 0)
      return (stored ?? 0) & 65280 | s[key(ch, "ref_id")] << 4 | s[key(ch, "mode")];
    if (lo === 1) return s[key(ch, "prox_debounce")] << 8 | s[key(ch, "prox_th")];
    if (lo === 2) return s[key(ch, "touch_hyst")] << 12 | s[key(ch, "touch_th")];
    return stored ?? 0;
  }
  if (reg === 176) return s.counts_filter;
  if (reg === 192)
    return (s.comm_mode ? 128 : 0) | (s.power_mode & 7) << 4 | (stored ?? 0) & 1792;
  if (reg === 193) return s.np_rate_ms;
  if (reg === 194) return s.lp_rate_ms;
  if (reg === 195) return s.ulp_rate_ms;
  if (reg === 196) return s.halt_rate_ms;
  if (reg === 197) return s.power_timeout_ms;
  if (reg === 211) return s.events_enable;
  return stored ?? INIT_REGS[reg] ?? 0;
}
function writeReg(s, reg, v) {
  if (!regExists(reg) || reg <= 37 || reg === 225) return void 0;
  const hi = reg >> 4;
  const lo = reg & 15;
  const store = (value) => {
    const file = regFile(s);
    file[String(reg)] = value;
    return { reg_file: JSON.stringify(file) };
  };
  if (hi >= 3 && hi <= 5) {
    const ch = hi - 3;
    if (lo === 1) return { [key(ch, "conv_freq")]: v };
    if (lo === 6) return { [key(ch, "ati_setup")]: v };
    if (lo === 9) return { [key(ch, "compensation")]: v & 64511 };
    return store(v);
  }
  if (hi >= 6 && hi <= 8) {
    const ch = hi - 6;
    if (lo === 0)
      return {
        [key(ch, "mode")]: v & 15,
        [key(ch, "ref_id")]: v >> 4 & 15,
        ...store(v & 65280)
      };
    if (lo === 1) return { [key(ch, "prox_th")]: v & 255, [key(ch, "prox_debounce")]: v >> 8 };
    if (lo === 2) return { [key(ch, "touch_th")]: v & 255, [key(ch, "touch_hyst")]: v >> 12 };
    return store(v);
  }
  if (reg === 176) return { counts_filter: v };
  if (reg === 192) {
    if (v & 2) return { ...configAfterInit, ready: s.ready };
    const next = { comm_mode: v & 128 ? 1 : 0, ...store(v & 1792) };
    const mode = v >> 4 & 7;
    if (mode <= PM_AUTO_NO_ULP) next.power_mode = mode;
    if (v & 12) {
      next.ch0_lta = countsOf(s, 0);
      next.ch1_lta = countsOf(s, 1);
      next.ch2_lta = countsOf(s, 2);
    }
    return next;
  }
  if (reg === 193) return { np_rate_ms: Math.min(3e3, v) };
  if (reg === 194) return { lp_rate_ms: Math.min(3e3, v) };
  if (reg === 195) return { ulp_rate_ms: Math.min(3e3, v) };
  if (reg === 196) return { halt_rate_ms: Math.min(3e3, v) };
  if (reg === 197) return { power_timeout_ms: Math.min(65e3, v) };
  if (reg === 211) return { events_enable: v & 95 };
  return store(v);
}
var sim = {
  tile: "Sense.T.C",
  defaultState: DEFAULT_STATE,
  controls: [
    { type: "toggle", field: "ch1_touched", label: "Touch top surface (CH1)" },
    { type: "toggle", field: "ch0_touched", label: "Touch C0 (pad 8)" },
    { type: "toggle", field: "ch1_prox", label: "Approach surface (prox)" },
    { type: "toggle", field: "ch0_prox", label: "Approach C0 (prox)" }
  ],
  // What a finger does: approach or touch either electrode.
  stimuli: [
    {
      id: "finger",
      label: "Finger",
      controls: [
        {
          kind: "toggle",
          id: "touch_surface",
          label: "Touch top surface",
          fields: ["ch1_touched"]
        },
        { kind: "toggle", id: "near_surface", label: "Hand near surface", fields: ["ch1_prox"] },
        { kind: "toggle", id: "touch_c0", label: "Touch pad 8 electrode", fields: ["ch0_touched"] },
        { kind: "toggle", id: "near_c0", label: "Near pad 8 electrode", fields: ["ch0_prox"] }
      ]
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    tile_sense_t_c_find: () => ({ scalar: 1 }),
    tile_sense_t_c_init: () => ({ scalar: 0 }),
    tile_sense_t_c_process: ({ state }) => ({ nextState: process(state) }),
    // The callback fires from process(); nothing to hold in state.
    tile_sense_t_c_on_event: () => void 0,
    // Halt; power_mode keeps the host's choice, which wake() restores.
    tile_sense_t_c_sleep: () => ({
      nextState: { sleeping: 1, ready: 0 }
    }),
    tile_sense_t_c_wake: () => ({
      nextState: { sleeping: 0, ready: 1 }
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
      scalar: state.last_status & (chTouchBit(0) | chTouchBit(1) | chTouchBit(2)) ? 1 : 0
    }),
    // process(), then is_touched_any, polled until timeout. The world holds
    // still during the call, so one pass answers it.
    tile_sense_t_c_wait_for_touch: ({ state }) => {
      if (!state.ready) return { scalar: 0 };
      const next = process(state);
      const st = next.last_status ?? state.last_status;
      return {
        scalar: st & (chTouchBit(0) | chTouchBit(1) | chTouchBit(2)) ? 1 : 0,
        nextState: next
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
      return { scalar: (countsOf(state, ch) - ltaOf(state, ch) & 65535) << 16 >> 16 };
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
      const prox = arg(args, 1, 0) & 255;
      const touch = arg(args, 2, 0) & 255;
      const next = {};
      if (prox) {
        next[key(ch, "prox_th")] = prox;
        next[key(ch, "prox_debounce")] = 68;
      }
      if (touch) {
        next[key(ch, "touch_th")] = touch;
        next[key(ch, "touch_hyst")] = 0;
      }
      return { nextState: next };
    },
    // All enabled channels (CH0, CH1); 0 is ignored.
    tile_sense_t_c_set_touch_threshold: ({ args }) => {
      const th = arg(args, 0, 0) & 255;
      return th ? {
        nextState: { ch0_touch_th: th, ch1_touch_th: th, ch0_touch_hyst: 0, ch1_touch_hyst: 0 }
      } : void 0;
    },
    tile_sense_t_c_set_prox_threshold: ({ args }) => {
      const th = arg(args, 0, 0) & 255;
      return th ? {
        nextState: {
          ch0_prox_th: th,
          ch1_prox_th: th,
          ch0_prox_debounce: 68,
          ch1_prox_debounce: 68
        }
      } : void 0;
    },
    // Modes 6 and 7 are reserved; the driver ignores them.
    tile_sense_t_c_set_power_mode: ({ state, args }) => {
      const mode = arg(args, 0, state.power_mode);
      return mode >= PM_NORMAL && mode <= PM_AUTO_NO_ULP ? { nextState: { power_mode: mode } } : void 0;
    },
    // Events Enable: only bits 0-4 and 6 are defined (A.33).
    tile_sense_t_c_enable_events: ({ args }) => ({
      nextState: { events_enable: arg(args, 0, 0) & 95 }
    }),
    // Re-ATI: the chip re-tunes and reseeds, so every LTA lands on its counts.
    tile_sense_t_c_ati: ({ state }) => ({
      nextState: {
        ch0_lta: countsOf(state, 0),
        ch1_lta: countsOf(state, 1),
        ch2_lta: countsOf(state, 2)
      }
    }),
    tile_sense_t_c_reseed: ({ state }) => ({
      nextState: {
        ch0_lta: countsOf(state, 0),
        ch1_lta: countsOf(state, 1),
        ch2_lta: countsOf(state, 2)
      }
    }),
    tile_sense_t_c_set_ati_setup: ({ args }) => {
      const ch = chArg(args);
      if (!validCh(ch)) return;
      return { nextState: { [key(ch, "ati_setup")]: arg(args, 1, 0) & 65535 } };
    },
    tile_sense_t_c_set_counts_filter: ({ args }) => ({
      nextState: { counts_filter: arg(args, 0, 0) & 65535 }
    }),
    tile_sense_t_c_set_conversion_freq: ({ args }) => {
      const ch = chArg(args);
      if (!validCh(ch)) return;
      return { nextState: { [key(ch, "conv_freq")]: arg(args, 1, 0) & 65535 } };
    },
    tile_sense_t_c_set_channel_mode: ({ args }) => {
      const ch = chArg(args);
      if (!validCh(ch)) return;
      return {
        nextState: {
          [key(ch, "mode")]: arg(args, 1, 0) & 15,
          [key(ch, "ref_id")]: arg(args, 2, 0) & 15
        }
      };
    },
    tile_sense_t_c_set_comm_mode: ({ args }) => ({
      nextState: { comm_mode: arg(args, 0, 0) ? 1 : 0 }
    }),
    // AUTO / AUTO_NO_ULP have no rate register; the driver caps at 3000 ms.
    tile_sense_t_c_set_report_rate: ({ args }) => {
      const mode = arg(args, 0, PM_NORMAL);
      const ms = Math.min(3e3, arg(args, 1, 16) & 65535);
      const field = mode === PM_NORMAL ? "np_rate_ms" : mode === PM_LOW ? "lp_rate_ms" : mode === PM_ULP ? "ulp_rate_ms" : mode === PM_HALT ? "halt_rate_ms" : void 0;
      return field ? { nextState: { [field]: ms } } : void 0;
    },
    tile_sense_t_c_set_power_timeout: ({ args }) => ({
      nextState: { power_timeout_ms: Math.min(65e3, arg(args, 0, 2e3) & 65535) }
    }),
    tile_sense_t_c_get_compensation: ({ state, args }) => {
      const ch = chArg(args);
      return { scalar: validCh(ch) ? state[key(ch, "compensation")] : 0 };
    },
    // Compensation (A.14): [15:11] divider, [9:0] value.
    tile_sense_t_c_set_compensation: ({ args }) => {
      const ch = chArg(args);
      if (!validCh(ch)) return;
      const value = (arg(args, 2, 0) & 31) << 11 | arg(args, 1, 0) & 1023;
      return { nextState: { [key(ch, "compensation")]: value } };
    },
    // ── low-level ──
    tile_sense_t_c_read_reg: ({ state, args }) => ({
      scalar: readReg(state, arg(args, 0, 0) & 255)
    }),
    tile_sense_t_c_write_reg: ({ state, args }) => {
      const next = writeReg(state, arg(args, 0, 0) & 255, arg(args, 1, 0) & 65535);
      return next ? { nextState: next } : void 0;
    }
  },
  provenance: {
    tile_sense_t_c_find: "canonical",
    // ACK at 0x44 (order code 001)
    tile_sense_t_c_process: "canonical",
    // caches System Status, READY-gated
    tile_sense_t_c_get_status: "canonical",
    // cached word, bit layout per §A.2
    tile_sense_t_c_is_touched: "canonical",
    tile_sense_t_c_is_prox: "canonical",
    tile_sense_t_c_is_touched_any: "canonical",
    tile_sense_t_c_get_lta: "canonical",
    tile_sense_t_c_get_delta: "canonical",
    // counts − LTA, int16; negative on self-cap touch
    tile_sense_t_c_set_power_mode: "canonical",
    // SYSTEM_CONTROL[6:4]
    tile_sense_t_c_set_comm_mode: "canonical",
    // SYSTEM_CONTROL bit 7 (1 = event)
    tile_sense_t_c_set_thresholds: "canonical",
    // written only when non-zero; touch is x/256 of LTA
    tile_sense_t_c_set_touch_threshold: "canonical",
    // A.17 on CH0 + CH1
    tile_sense_t_c_set_prox_threshold: "canonical",
    // A.16 on CH0 + CH1
    tile_sense_t_c_enable_events: "canonical",
    // 0xD3, defined bits only
    tile_sense_t_c_reseed: "canonical",
    // SYSTEM_CONTROL.RESEED: LTA → counts
    tile_sense_t_c_set_report_rate: "canonical",
    // 0xC1-0xC4, ms, capped 3000
    tile_sense_t_c_set_power_timeout: "canonical",
    // 0xC5, ms
    tile_sense_t_c_set_compensation: "canonical",
    // A.14 encoding
    tile_sense_t_c_get_compensation: "inferred",
    // nominal EV-kit working point until set
    // inferred — modeled magnitudes / time collapsed
    tile_sense_t_c_get_counts: "inferred",
    // ~512 at rest, finger drops are modeled
    tile_sense_t_c_ati: "inferred",
    tile_sense_t_c_wait_for_touch: "inferred",
    // no slider or gestures unless the program configures them (§9: Slider
    // Resolution and Gesture Enable are 0 after reset; init leaves them)
    tile_sense_t_c_get_slider: "inferred",
    tile_sense_t_c_read_slider_pct: "inferred",
    // resolution 0 → returns 0
    tile_sense_t_c_get_gestures: "inferred",
    tile_sense_t_c_wait_for_gesture: "inferred",
    // the §9 memory map: modeled registers read and write the twin's state,
    // the rest hold their value after init; missing ones read 0xEEEE (§8.10)
    tile_sense_t_c_read_reg: "canonical",
    tile_sense_t_c_write_reg: "canonical",
    power: "inferred"
    // datasheet §3.4 per-mode current is for 3 self-cap channels at the EV-kit rates; the tile runs 2, and AUTO's stepping is modeled
  },
  // Each tick: idle channels' LTA follows the counts; prox/touch changes latch
  // the event flags (and RDY, when enabled); AUTO power steps NP → LP → ULP after
  // `power_timeout_ms` of no activity (datasheet §6 auto mode), back to NP on any.
  deriveState(state, { t }) {
    const update = {};
    const bits = channelBits(state);
    for (let ch = 0; ch < CHANNELS; ch++) {
      const active = bits & (chProxBit(ch) | chTouchBit(ch));
      const counts = countsOf(state, ch);
      if (!active && ltaOf(state, ch) !== counts) update[key(ch, "lta")] = counts;
    }
    if (bits !== state.prev_ch_bits) {
      const changed = bits ^ state.prev_ch_bits;
      let events = 0;
      if (changed & 5376) events |= ST_PROX_EVENT;
      if (changed & 10752) events |= ST_TOUCH_EVENT;
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
    if (state.sleeping) return { "3": 0 };
    if (state.comm_mode === 0) return { "3": 1 };
    return { "3": state.int_latched ? 1 : 0 };
  },
  // Electrical: a pure load on V+ (pad 10) / GND (pad 1). Draw follows the
  // effective power mode (datasheet §3.4, self-cap; the same at 1.8 and 3.3 V).
  power(state, ctx) {
    const mode = state.sleeping ? PM_HALT : state.power_mode <= PM_HALT ? state.power_mode : state.active_mode;
    const ua = MODE_UA[mode] ?? MODE_UA[PM_NORMAL];
    return {
      draw_ua: ua,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: ctx?.padVoltage?.["10"] ?? 3300,
          i_ua: ua,
          pads: ["10"],
          note: `IQS323 ${MODE_NAME[mode] ?? "NP"}${!state.sleeping && state.power_mode >= PM_AUTO ? " (auto)" : ""}`
        }
      ]
    };
  }
};
var sense_t_c_default = sim;
export {
  sense_t_c_default as default
};
