// src/sims/display_rgbw.ts
var DC_DEFAULT = 5;
var VOUT_INIT_MV = 4500;
var VOUT_RESET_MV = 3e3;
var BOOST_EFF = 0.9;
var VIN_UVLO_MV = 1800;
var EN_VIL_MV = 400;
var ISD_UA = 0.1;
var ISTB_UA = 26;
var INOR_UA = 450;
var CH = ["r", "b", "g", "w"];
var HELPER_PULSE = 1;
var HELPER_BREATHE = 2;
var HELPER_FLASH = 3;
var num = (args, i, dflt) => args.length > i && Number.isFinite(args[i]) ? Math.trunc(args[i]) : dflt;
var u8 = (args, i) => num(args, i, 0) & 255;
var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
var fullScaleMa = (s) => s.max_current_mode === 1 ? 51 : 25.5;
var autoOn = (s, ch) => (ch === "r" ? s.auto_r : ch === "g" ? s.auto_g : ch === "b" ? s.auto_b : s.auto_w) === 1;
var pwmOf = (s, ch) => ch === "r" ? s.r : ch === "g" ? s.g : ch === "b" ? s.b : s.w;
var dcOf = (s, ch) => ch === "r" ? s.r_curr : ch === "g" ? s.g_curr : ch === "b" ? s.b_curr : s.w_curr;
var bitOf = (ch) => 1 << CH.indexOf(ch);
function animDuty(s) {
  if (s.anim_period_ms <= 0) return s.anim_peak / 255;
  const phase = s.anim_clock % s.anim_period_ms / s.anim_period_ms;
  return s.anim_peak / 255 * (1 - Math.abs(2 * phase - 1));
}
function duty(s, ch) {
  if (autoOn(s, ch)) {
    return s.animating === 1 && s.anim_channel === CH.indexOf(ch) ? animDuty(s) : 0;
  }
  return pwmOf(s, ch) / 255;
}
var ofaf = (s) => s.lsd_shutdown === 1 && s.fault_short_mask !== 0;
function level(s, ch) {
  if (s.enabled !== 1 || ofaf(s)) return 0;
  if (s.fault_open_mask & bitOf(ch)) return 0;
  return clamp(duty(s, ch) * (dcOf(s, ch) / 255), 0, 1);
}
function supplies(s, ctx) {
  if (!ctx) return { vin: s.vplus_mv, enHigh: true };
  const en = ctx.padVoltage["8"];
  return { vin: ctx.padVoltage["10"] ?? 0, enHigh: en == null || en >= EN_VIL_MV };
}
var chipAlive = (s, ctx) => {
  const { vin, enHigh } = supplies(s, ctx);
  return vin >= VIN_UVLO_MV && enHigh;
};
var cancelHelper = { helper_running: 0, helper_kind: 0, helper_start_ms: -1 };
var startHelper = (kind, args) => ({
  helper_running: 1,
  helper_kind: kind,
  helper_r: u8(args, 0),
  helper_g: u8(args, 1),
  helper_b: u8(args, 2),
  helper_start_ms: -1
});
var RESET_STATE = {
  r: 0,
  g: 0,
  b: 0,
  w: 0,
  r_curr: 0,
  g_curr: 0,
  b_curr: 0,
  w_curr: 0,
  enabled: 0,
  max_current_mode: 0,
  boost_mv: VOUT_RESET_MV,
  lsd_threshold: 0,
  lsd_shutdown: 0,
  lod_shutdown: 1,
  fault_short_mask: 0,
  fault_open_mask: 0,
  fault_tsd: 0,
  fault_config: 0,
  auto_r: 0,
  auto_g: 0,
  auto_b: 0,
  auto_w: 0,
  animating: 0,
  anim_paused: 0,
  anim_clock: 0,
  exp_mask: 0,
  phase_align: 0,
  aeu_configured: 0,
  ...cancelHelper
};
var sim = {
  tile: "Display.RGBW",
  // After tile_display_rgbw_init(): CHIP_EN 1, boost 4.5 V, MC 51 mA, all four
  // LEDs enabled, DC = 5, PWM 0, lod_action 1 / lsd_action 0 / threshold 3.
  defaultState: {
    r: 0,
    g: 0,
    b: 0,
    w: 0,
    r_curr: DC_DEFAULT,
    g_curr: DC_DEFAULT,
    b_curr: DC_DEFAULT,
    w_curr: DC_DEFAULT,
    enabled: 1,
    max_current_mode: 1,
    boost_mv: VOUT_INIT_MV,
    lsd_threshold: 3,
    lsd_shutdown: 0,
    lod_shutdown: 1,
    fault_inject: 0,
    fault_short_mask: 0,
    fault_open_mask: 0,
    fault_tsd: 0,
    fault_config: 0,
    helper_running: 0,
    helper_kind: 0,
    helper_r: 0,
    helper_g: 0,
    helper_b: 0,
    helper_ms: 0,
    breathe_period_ms: 0,
    flash_count: 0,
    helper_start_ms: -1,
    auto_r: 0,
    auto_g: 0,
    auto_b: 0,
    auto_w: 0,
    anim_channel: 0,
    anim_peak: 255,
    anim_period_ms: 2e3,
    animating: 0,
    anim_paused: 0,
    anim_clock: 0,
    exp_mask: 0,
    phase_align: 0,
    aeu_configured: 0,
    vplus_mv: 3300,
    last_tick_ms: 0
  },
  controls: [
    {
      type: "toggle",
      field: "fault_inject",
      label: "Shorted LED (LSD)",
      description: "A short on every channel: latches the LSD status (is_faulted \u2192 1) and, after set_short_shutdown(1), trips OFAF \u2014 all LEDs off until clear_faults."
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    tile_display_rgbw_find: () => ({ scalar: 1 }),
    tile_display_rgbw_init: () => ({ scalar: 0 }),
    tile_display_rgbw_sleep: () => ({ nextState: { enabled: 0, ...cancelHelper } }),
    tile_display_rgbw_wake: () => ({ nextState: { enabled: 1 } }),
    tile_display_rgbw_reset: () => ({ nextState: RESET_STATE }),
    // ── manual output ──
    tile_display_rgbw_set: ({ args }) => ({
      nextState: {
        r: u8(args, 0),
        g: u8(args, 1),
        b: u8(args, 2),
        w: u8(args, 3),
        ...cancelHelper
      }
    }),
    tile_display_rgbw_set_color: ({ args }) => ({
      nextState: { r: u8(args, 0), g: u8(args, 1), b: u8(args, 2), w: 0, ...cancelHelper }
    }),
    tile_display_rgbw_off: () => ({ nextState: { r: 0, g: 0, b: 0, w: 0, ...cancelHelper } }),
    tile_display_rgbw_set_current: ({ args }) => ({
      nextState: {
        r_curr: u8(args, 0),
        g_curr: u8(args, 1),
        b_curr: u8(args, 2),
        w_curr: u8(args, 3)
      }
    }),
    tile_display_rgbw_set_max_current: ({ args }) => ({
      nextState: { max_current_mode: num(args, 0, 1) ? 1 : 0 }
    }),
    // ── software helpers: set, wait, off (the tick plays the envelope) ──
    tile_display_rgbw_pulse: ({ args }) => ({
      nextState: {
        ...startHelper(HELPER_PULSE, args),
        helper_ms: num(args, 3, 0) & 65535,
        r: u8(args, 0),
        g: u8(args, 1),
        b: u8(args, 2),
        w: 0
      }
    }),
    tile_display_rgbw_breathe: ({ args }) => ({
      nextState: {
        ...startHelper(HELPER_BREATHE, args),
        breathe_period_ms: num(args, 3, 0) & 65535,
        r: 0,
        g: 0,
        b: 0,
        w: 0
      }
    }),
    tile_display_rgbw_flash: ({ args }) => ({
      nextState: {
        ...startHelper(HELPER_FLASH, args),
        flash_count: u8(args, 3),
        r: u8(args, 0),
        g: u8(args, 1),
        b: u8(args, 2),
        w: 0
      }
    }),
    // ── faults ──
    // Fills a disp_rgbw_faults_t the DSL can't read back; nothing to return.
    tile_display_rgbw_read_faults: () => ({}),
    // Fault_Clear (W1C 0x07): clears the latches and leaves OFAF. A short that's
    // still there re-latches on the next tick.
    tile_display_rgbw_clear_faults: () => ({
      nextState: { fault_short_mask: 0, fault_open_mask: 0, fault_tsd: 0, fault_config: 0 }
    }),
    tile_display_rgbw_is_faulted: ({ state }) => ({
      scalar: state.fault_short_mask || state.fault_open_mask || state.fault_tsd || state.fault_config || state.fault_inject ? 1 : 0
    }),
    tile_display_rgbw_set_short_threshold: ({ args }) => ({
      nextState: { lsd_threshold: num(args, 0, 3) & 3 }
    }),
    tile_display_rgbw_set_short_shutdown: ({ args }) => ({
      nextState: { lsd_shutdown: num(args, 0, 0) ? 1 : 0 }
    }),
    tile_display_rgbw_set_open_shutdown: ({ args }) => ({
      nextState: { lod_shutdown: num(args, 0, 1) ? 1 : 0 }
    }),
    // ── autonomous engine ──
    tile_display_rgbw_ms_to_slope: ({ args }) => {
      const table = [
        0,
        90,
        180,
        360,
        540,
        800,
        1070,
        1520,
        2060,
        2500,
        3040,
        4020,
        5010,
        5990,
        7060,
        8050
      ];
      const ms = num(args, 0, 0) & 65535;
      let best = 0;
      for (let i = 1; i < 16; i++)
        if (Math.abs(ms - table[i]) < Math.abs(ms - table[best])) best = i;
      return { scalar: best };
    },
    tile_display_rgbw_set_autonomous: ({ args }) => {
      const ch = num(args, 0, 0);
      if (ch < 0 || ch > 3) return {};
      const on = num(args, 1, 1) ? 1 : 0;
      const key = CH[ch];
      return {
        nextState: key === "r" ? { auto_r: on } : key === "g" ? { auto_g: on } : key === "b" ? { auto_b: on } : { auto_w: on }
      };
    },
    // The keyframe struct can't cross the flat ABI; record that a program exists.
    tile_display_rgbw_set_aeu: ({ args }) => {
      const ch = num(args, 0, 0);
      const aeu = num(args, 1, 0);
      if (ch < 0 || ch > 3 || aeu < 1 || aeu > 3) return {};
      return { nextState: { aeu_configured: 1, anim_channel: ch } };
    },
    tile_display_rgbw_set_animation: ({ args }) => {
      const ch = num(args, 0, 0);
      return ch < 0 || ch > 3 ? {} : { nextState: { anim_channel: ch } };
    },
    tile_display_rgbw_set_exp_dimming: ({ state, args }) => {
      const ch = num(args, 0, 0);
      if (ch < 0 || ch > 3) return {};
      const bit = 1 << ch;
      return {
        nextState: { exp_mask: num(args, 1, 1) ? state.exp_mask | bit : state.exp_mask & ~bit }
      };
    },
    tile_display_rgbw_set_phase_align: ({ state, args }) => {
      const ch = num(args, 0, 0);
      if (ch < 0 || ch > 3) return {};
      const shift = ch * 2;
      return {
        nextState: {
          phase_align: state.phase_align & ~(3 << shift) | (num(args, 1, 0) & 3) << shift
        }
      };
    },
    tile_display_rgbw_update: () => ({}),
    tile_display_rgbw_animate_start: () => ({
      nextState: { animating: 1, anim_paused: 0, anim_clock: 0 }
    }),
    tile_display_rgbw_animate_stop: () => ({
      nextState: { animating: 0, anim_paused: 0, anim_clock: 0 }
    }),
    tile_display_rgbw_animate_pause: () => ({ nextState: { anim_paused: 1 } }),
    tile_display_rgbw_animate_continue: () => ({ nextState: { anim_paused: 0 } }),
    tile_display_rgbw_breathe_auto: ({ state, args }) => {
      const ch = num(args, 0, 0);
      if (ch < 0 || ch > 3) return {};
      const key = CH[ch];
      return {
        nextState: {
          anim_channel: ch,
          anim_peak: u8(args, 1),
          anim_period_ms: num(args, 2, 2e3) & 65535,
          aeu_configured: 1,
          exp_mask: state.exp_mask | 1 << ch,
          ...key === "r" ? { auto_r: 1 } : key === "g" ? { auto_g: 1 } : key === "b" ? { auto_b: 1 } : { auto_w: 1 },
          animating: 1,
          anim_paused: 0,
          anim_clock: 0
        }
      };
    }
  },
  provenance: {
    tile_display_rgbw_find: "canonical",
    tile_display_rgbw_init: "canonical",
    // defaultState = post-init registers
    tile_display_rgbw_sleep: "canonical",
    // CHIP_EN = 0
    tile_display_rgbw_wake: "canonical",
    tile_display_rgbw_reset: "canonical",
    // register reset values
    tile_display_rgbw_set: "canonical",
    tile_display_rgbw_set_color: "canonical",
    tile_display_rgbw_off: "canonical",
    tile_display_rgbw_set_current: "canonical",
    tile_display_rgbw_set_max_current: "canonical",
    tile_display_rgbw_pulse: "inferred",
    // driver sequence; timing played by the tick
    tile_display_rgbw_breathe: "inferred",
    // 32-step ramp modeled as a triangle
    tile_display_rgbw_flash: "inferred",
    // 100 ms on / 100 ms off
    tile_display_rgbw_read_faults: "inferred",
    // struct out not readable from the DSL
    tile_display_rgbw_clear_faults: "canonical",
    tile_display_rgbw_is_faulted: "canonical",
    tile_display_rgbw_set_short_threshold: "inferred",
    // stored; the injected short trips at any threshold
    tile_display_rgbw_set_short_shutdown: "canonical",
    // lsd_action → OFAF
    tile_display_rgbw_set_open_shutdown: "inferred",
    // stored; no open-LED stimulus
    tile_display_rgbw_ms_to_slope: "canonical",
    // lp_time_ms table
    tile_display_rgbw_set_autonomous: "canonical",
    tile_display_rgbw_set_aeu: "hallucinated",
    // keyframes can't cross the flat ABI
    tile_display_rgbw_set_animation: "inferred",
    // only the channel is captured
    tile_display_rgbw_set_exp_dimming: "canonical",
    tile_display_rgbw_set_phase_align: "canonical",
    tile_display_rgbw_update: "inferred",
    // config latching not modeled
    tile_display_rgbw_animate_start: "canonical",
    tile_display_rgbw_animate_stop: "canonical",
    tile_display_rgbw_animate_pause: "canonical",
    tile_display_rgbw_animate_continue: "canonical",
    tile_display_rgbw_breathe_auto: "inferred",
    // AEU program modeled as a triangle, loops forever
    power: "inferred"
    // ISD / ISTB / INOR canonical; boost efficiency 90 % (datasheet guidance)
  },
  // Each tick: advance the engine clock, play a software helper's envelope, and
  // latch an injected short into LSD_STATUS.
  deriveState(state, { t }) {
    const update = { last_tick_ms: t };
    const dt = state.last_tick_ms > 0 ? Math.max(0, t - state.last_tick_ms) : 0;
    if (state.animating === 1 && state.anim_paused !== 1 && dt > 0)
      update.anim_clock = state.anim_clock + dt;
    if (state.fault_inject === 1 && state.fault_short_mask !== 15) update.fault_short_mask = 15;
    if (state.helper_running === 1) {
      const start = state.helper_start_ms < 0 ? t : state.helper_start_ms;
      if (state.helper_start_ms < 0) update.helper_start_ms = t;
      const el = t - start;
      const peak = { r: state.helper_r, g: state.helper_g, b: state.helper_b };
      let on = 0;
      let done = false;
      if (state.helper_kind === HELPER_PULSE) {
        done = el >= state.helper_ms;
        on = done ? 0 : 1;
      } else if (state.helper_kind === HELPER_BREATHE) {
        const p = Math.max(64, state.breathe_period_ms);
        done = el >= p;
        on = done ? 0 : 1 - Math.abs(2 * el / p - 1);
      } else if (state.helper_kind === HELPER_FLASH) {
        done = el >= state.flash_count * 200;
        on = !done && el % 200 < 100 ? 1 : 0;
      } else done = true;
      update.r = Math.floor(peak.r * on);
      update.g = Math.floor(peak.g * on);
      update.b = Math.floor(peak.b * on);
      update.w = 0;
      if (done) Object.assign(update, cancelHelper);
    }
    return update;
  },
  // The on-tile RGBW LED, one indicator per die; dark without power or with EN low.
  indicators(state, ctx) {
    const alive = chipAlive(state, ctx);
    const lit = (ch) => alive ? level(state, ch) : 0;
    const ind = [
      { id: "led.r", label: "Red", color: "#ff4d4d", level: lit("r") },
      { id: "led.g", label: "Green", color: "#3ddc84", level: lit("g") },
      { id: "led.b", label: "Blue", color: "#4d8dff", level: lit("b") },
      { id: "led.w", label: "White", color: "#fff6e0", level: lit("w") }
    ];
    return ind;
  },
  // Electrical: V+ (pad 10) feeds the boost; EN (pad 8) is read so a grounded EN
  // shuts the chip down. LED current comes out of VOUT, so the input pays for it
  // at the boost ratio.
  power(state, ctx) {
    const { vin, enHigh } = supplies(state, ctx);
    const ledMa = CH.reduce((sum, ch) => sum + level(state, ch) * fullScaleMa(state), 0);
    let draw;
    let note;
    if (vin < VIN_UVLO_MV) {
      draw = 0;
      note = "unpowered (below UVLO)";
    } else if (!enHigh) {
      draw = ISD_UA;
      note = "shutdown (EN low)";
    } else if (state.enabled !== 1 || ledMa <= 0) {
      draw = ISTB_UA;
      note = state.enabled !== 1 ? "standby (CHIP_EN = 0)" : "standby (no LED on)";
    } else {
      const vout = Math.max(state.boost_mv, vin);
      draw = Math.round(INOR_UA + ledMa * 1e3 * vout / (BOOST_EFF * vin));
      note = "boost + LED current";
    }
    const rails = [
      { name: "V+", role: "supply", v_mv: vin, i_ua: draw, pads: ["10"], note },
      {
        name: "EN",
        role: "supply",
        v_mv: 0,
        i_ua: 0,
        pads: ["8"],
        note: "enable input (pulled up)"
      }
    ];
    return { draw_ua: draw, rails };
  }
};
var display_rgbw_default = sim;
export {
  display_rgbw_default as default
};
