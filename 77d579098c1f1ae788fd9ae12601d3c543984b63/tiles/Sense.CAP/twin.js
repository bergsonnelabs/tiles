// src/sims/sense_cap.ts
var OK = 0;
var BUSY = 1;
var ERR_STATE = 2;
var ERR_ARG = 3;
var ERR_FORBIDDEN = 5;
var ERR_RESET = 11;
var ERR_NO_CALLBACK = 12;
var LAYOUT_NONE = 0;
var LAYOUT_SLIDER_1X4 = 4;
var MODE_ACTIVE = 0;
var MODE_IDLE_TOUCH = 1;
var MODE_IDLE = 2;
var MODE_LP1 = 3;
var MODE_LP2 = 4;
var MODE_NAME = ["Active", "Idle-Touch", "Idle", "LP1", "LP2"];
var POWER_AUTO = 0;
var POWER_FORCE_ACTIVE = 1;
var POWER_FORCE_LP1 = 2;
var POWER_FORCE_LP2 = 3;
var GESTURES_DRIVER = 0;
var GESTURES_CHIP = 1;
var ATI_TRACKPAD = 1;
var ATI_ALP = 2;
var INFO_MODE_MASK = 7;
var INFO_RE_ATI_OCCURRED = 1 << 4;
var INFO_SHOW_RESET = 1 << 7;
var INFO_NUM_FINGERS_SHIFT = 8;
var INFO_TP_MOVEMENT = 1 << 10;
var INFO_ALP_OUTPUT = 1 << 14;
var INFO_ATI_ERROR = 1 << 3;
var INFO_ALP_ATI_ERROR = 1 << 5;
var G_TAP = 1 << 0;
var G_HOLD = 1 << 1;
var G_X_NEG = 1 << 2;
var G_X_POS = 1 << 3;
var G_Y_POS = 1 << 4;
var G_Y_NEG = 1 << 5;
var G_SWIPES = G_X_NEG | G_X_POS | G_Y_POS | G_Y_NEG;
var G_ALL = 63;
var EV_TOUCH_DOWN = 1 << 0;
var EV_TOUCH_UP = 1 << 1;
var EV_TAP = 1 << 2;
var EV_DOUBLE_TAP = 1 << 3;
var EV_LONG_PRESS = 1 << 4;
var EV_SWIPE = 1 << 5;
var EV_DRAG = 1 << 6;
var EV_RESET = 1 << 7;
var DIR_LEFT = 0;
var DIR_RIGHT = 1;
var DIR_UP = 2;
var DIR_DOWN = 3;
var DIR_EV = [1 << 8, 1 << 9, 1 << 10, 1 << 11];
var DIR_GESTURE = [G_X_NEG, G_X_POS, G_Y_NEG, G_Y_POS];
var CFG_TP_RE_ATI_EN = 1 << 2;
var CFG_ALP_RE_ATI_EN = 1 << 3;
var CFG_COMMS_REQUEST_EN = 1 << 4;
var CFG_WDT_EN = 1 << 5;
var CFG_MANUAL_CONTROL = 1 << 7;
var CFG_EVENT_MODE = 1 << 8;
var CFG_GESTURE_EVENT = 1 << 9;
var PITCH_PX = 256;
var TOUCH_TAP_DIST = 48;
var TOUCH_MOVE_DEAD = 4;
var TOUCH_FLING_PXS = 400;
var TOUCH_DOUBLE_MS = 350;
var TOUCH_CHIP_GATE_MS = 400;
var XY_INVALID = 65535;
var PROFILES = [
  [
    [10, 10, 10, 10, 10],
    [0, 0, 0, 0]
  ],
  [
    [10, 20, 20, 50, 100],
    [10, 30, 20, 60]
  ],
  [
    [10, 50, 50, 100, 200],
    [5, 30, 10, 30]
  ],
  [
    [20, 100, 100, 160, 320],
    [2, 20, 3, 10]
  ]
];
var RATE_FIELDS = [
  "rate_active",
  "rate_idle_touch",
  "rate_idle",
  "rate_lp1",
  "rate_lp2"
];
var TMO_FIELDS = ["tmo_active", "tmo_idle_touch", "tmo_idle", "tmo_lp1"];
var TAN64 = [
  0,
  1,
  2,
  3,
  4,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  14,
  15,
  16,
  17,
  18,
  20,
  21,
  22,
  23,
  25,
  26,
  27,
  28,
  30,
  31,
  33,
  34,
  35,
  37,
  38,
  40,
  42,
  43,
  45,
  46,
  48,
  50,
  52,
  54,
  56,
  58,
  60,
  62,
  64,
  66,
  69,
  71,
  74,
  76,
  79,
  82,
  85,
  88,
  91,
  95,
  99,
  102,
  107,
  111,
  115,
  120,
  126,
  131,
  137,
  144,
  151,
  158,
  167,
  176,
  186,
  197,
  209,
  223,
  239
];
var SENS_NUM = [2, 3, 1, 3, 1];
var SENS_DEN = [1, 2, 1, 4, 2];
var PRESETS = [
  { n_rx: 2, n_tx: 3, has_srf: false, alp: false, max_touches: 2 },
  // NONE: every channel disabled
  { n_rx: 2, n_tx: 3, has_srf: true, alp: true, max_touches: 2 },
  // GRID_2X3
  { n_rx: 2, n_tx: 3, has_srf: true, alp: true, max_touches: 2 },
  // BUTTONS_2X3
  { n_rx: 1, n_tx: 3, has_srf: true, alp: true, max_touches: 1 },
  // SLIDER_1X3 (RX3)
  { n_rx: 1, n_tx: 4, has_srf: true, alp: true, max_touches: 1 }
  // SLIDER_1X4 (RX6 as Tx)
];
var PRESET_SWITCH = 1;
var SURFACE_SET_MULT = 8;
var SURFACE_CLEAR_MULT = 5;
var ATI_TARGET = 900;
var ALP_ATI_TARGET = 200;
var FINGER_DELTA_MIN = 100;
var FINGER_DELTA_MAX = 190;
var FINGER_SPREAD = 1.5;
var ALP_NEAR_DELTA = 40;
var ALP_TOUCH_DELTA = 120;
var JOB_SYNC_POLLS = 4;
var ATI_TP_MS = 1e3;
var ATI_ALP_MS = 600;
var ATI_POLL_FALLBACK = 150;
var MODE_UA = [1150, 160, 160, 9, 6];
var arg = (args, i, fallback = 0) => args.length > i && Number.isFinite(args[i]) ? Math.trunc(args[i]) : fallback;
var u8 = (v) => v & 255;
var u16 = (v) => v & 65535;
var s16 = (v) => (v & 65535) << 16 >> 16;
var abs = Math.abs;
var preset = (s) => PRESETS[s.layout] ?? PRESETS[LAYOUT_NONE];
var nCh = (s) => preset(s).n_rx * preset(s).n_tx;
var effSwitch = (s) => (PRESET_SWITCH ^ s.user_switch) & 1;
var effFlipX = (s) => s.user_flip_x & 1;
var effFlipY = (s) => s.user_flip_y & 1;
var nX = (s) => effSwitch(s) ? preset(s).n_tx : preset(s).n_rx;
var nY = (s) => effSwitch(s) ? preset(s).n_rx : preset(s).n_tx;
var autoRes = (n) => n > 1 ? (n - 1) * PITCH_PX : 0;
var effXres = (s) => s.user_xres || autoRes(nX(s));
var effYres = (s) => s.user_yres || autoRes(nY(s));
var alpInUse = (s) => s.alp_wake === 1 && preset(s).alp;
var effTimeout = (s, i) => i === MODE_IDLE && !alpInUse(s) ? 0 : s[TMO_FIELDS[i]];
function pixelsPerPitch(s) {
  const nx = nX(s);
  const ny = nY(s);
  let best = nx > 1 ? Math.floor(effXres(s) / (nx - 1)) : 0;
  if (ny > 1) best = Math.max(best, Math.floor(effYres(s) / (ny - 1)));
  return best === 0 ? PITCH_PX : Math.min(best, 65535);
}
var scalePx = (s, c) => Math.min(65535, Math.max(1, Math.floor(c * pixelsPerPitch(s) / PITCH_PX)));
var effSwipeDist = (s) => s.swipe_dist_user || scalePx(s, TOUCH_TAP_DIST);
var tanAngle = (s) => TAN64[s.swipe_angle_deg <= 75 ? s.swipe_angle_deg : 45];
function sensitivityMults(sens) {
  const l = sens >= 0 && sens <= 4 ? sens : 2;
  const num = SENS_NUM[l];
  const den = SENS_DEN[l];
  let set = Math.floor((SURFACE_SET_MULT * num + Math.floor(den / 2)) / den);
  let clr = Math.floor((SURFACE_CLEAR_MULT * num + Math.floor(den / 2)) / den);
  set = Math.min(255, Math.max(1, set));
  if (clr > set) clr = set;
  if (clr === set && set > 1 && SURFACE_CLEAR_MULT < SURFACE_SET_MULT) clr = set - 1;
  return { mult_set: set, mult_clear: clr };
}
function configWord(s) {
  let v = CFG_TP_RE_ATI_EN | CFG_COMMS_REQUEST_EN | CFG_EVENT_MODE;
  if (alpInUse(s)) v |= CFG_ALP_RE_ATI_EN;
  if (s.wdt) v |= CFG_WDT_EN;
  if (s.power_mode !== POWER_AUTO || s.ati_busy && s.ati_targets & ATI_ALP)
    v |= CFG_MANUAL_CONTROL;
  if (s.gest_src === GESTURES_CHIP) v |= CFG_GESTURE_EVENT;
  return v;
}
var sensing = (s) => preset(s).has_srf && s.srf_applied === 1 && s.srf_tuned === 1 && !s.job_busy && !s.ati_busy && s.mode <= MODE_IDLE;
var touchThreshold = (s) => Math.floor(ATI_TARGET * s.mult_set / 128);
function channelDelta(s, ch) {
  const p = preset(s);
  if (!sensing(s) || !s.touching || ch < 0 || ch >= nCh(s)) return 0;
  const txI = Math.floor(ch / p.n_rx);
  const rxI = ch % p.n_rx;
  const dTx = p.n_tx > 1 ? abs(s.finger_x_pm / 1e3 * (p.n_tx - 1) - txI) : 0;
  const dRx = p.n_rx > 1 ? abs(s.finger_y_pm / 1e3 * (p.n_rx - 1) - rxI) : 0;
  const w = Math.max(0, 1 - dTx / FINGER_SPREAD) * Math.max(0, 1 - dRx / FINGER_SPREAD);
  const peak = FINGER_DELTA_MIN + (FINGER_DELTA_MAX - FINGER_DELTA_MIN) * s.pressure_pct / 100;
  return Math.round(peak * w);
}
function touchStatus(s) {
  const thr = touchThreshold(s);
  let st = 0;
  for (let ch = 0; ch < nCh(s); ch++) if (channelDelta(s, ch) > thr) st |= 1 << ch;
  return st;
}
var channelCount = (s, ch) => preset(s).has_srf && s.srf_applied && s.srf_tuned && ch >= 0 && ch < nCh(s) ? ATI_TARGET + channelDelta(s, ch) : 0;
function liveFinger(s) {
  const thr = touchThreshold(s);
  let strength = 0;
  let area = 0;
  for (let ch = 0; ch < nCh(s); ch++) {
    const d = channelDelta(s, ch);
    if (d > 0) strength += d;
    if (d > thr) area++;
  }
  if (area === 0) return { present: false, x: XY_INVALID, y: XY_INVALID, strength: 0, area: 0 };
  const tx = s.finger_x_pm / 1e3;
  const rx = s.finger_y_pm / 1e3;
  let ax = effSwitch(s) ? tx : rx;
  let ay = effSwitch(s) ? rx : tx;
  if (effFlipX(s)) ax = 1 - ax;
  if (effFlipY(s)) ay = 1 - ay;
  const x = nX(s) > 1 ? Math.round(ax * effXres(s)) : 0;
  const y = nY(s) > 1 ? Math.round(ay * effYres(s)) : 0;
  return { present: true, x, y, strength: Math.min(strength, 65535), area };
}
var alpDelta = (s) => s.touching ? ALP_TOUCH_DELTA : s.hand_near ? ALP_NEAR_DELTA : 0;
var alpOutput = (s) => alpInUse(s) && (s.mode === MODE_LP1 || s.mode === MODE_LP2) && alpDelta(s) > s.alp_threshold;
var alpLta = (s) => preset(s).alp ? ALP_ATI_TARGET * preset(s).n_rx : 0;
function liveInfo(s, f, moved) {
  let info = s.mode & INFO_MODE_MASK;
  if (s.reati_flag) info |= INFO_RE_ATI_OCCURRED;
  if (s.chip_reset_pending) info |= INFO_SHOW_RESET;
  if (f.present) info |= 1 << INFO_NUM_FINGERS_SHIFT;
  if (moved) info |= INFO_TP_MOVEMENT;
  if (alpOutput(s)) info |= INFO_ALP_OUTPUT;
  return info;
}
var cachedFingers = (s) => s.blk_info >> INFO_NUM_FINGERS_SHIFT & 3;
var firstFingerValid = (s) => cachedFingers(s) > 0 && s.blk_f1_x !== XY_INVALID && s.blk_f1_y !== XY_INVALID;
var CLEARED_BLOCK = {
  blk_info: 0,
  blk_gestures: 0,
  blk_rel_x: 0,
  blk_rel_y: 0,
  blk_f1_x: 0,
  blk_f1_y: 0,
  blk_f1_strength: 0,
  blk_f1_area: 0,
  blk_f2_x: 0,
  blk_f2_y: 0,
  blk_f2_strength: 0,
  blk_f2_area: 0
};
var startJob = (s) => ({
  job_busy: 1,
  job_polls: 0,
  job_t0: s.t_ms,
  srf_applied: 0,
  srf_tuned: 0
});
function chipReset(w) {
  if (w.was_down) w.ev_latch |= EV_TOUCH_UP;
  Object.assign(w, CLEARED_BLOCK, startJob(w), {
    was_down: 0,
    presence_cnt: 0,
    moved_far: 0,
    hold_fired: 0,
    prev_gestures: 0,
    ati_busy: 0,
    chip_reset_pending: 0,
    chip_was: 0,
    mode: MODE_ACTIVE,
    mode_t: w.t_ms,
    last_act_t: w.t_ms
  });
  w.ev_latch |= EV_RESET;
}
function jobStep(w, now) {
  w.job_polls++;
  if (w.job_polls < JOB_SYNC_POLLS) return BUSY;
  w.srf_applied = preset(w).has_srf ? 1 : 0;
  if (preset(w).has_srf) {
    const atiMs = ATI_TP_MS + (alpInUse(w) ? ATI_ALP_MS : 0);
    if (now - w.job_t0 < atiMs && w.job_polls < ATI_POLL_FALLBACK) return BUSY;
    w.srf_tuned = 1;
    w.reati_flag = 1;
  }
  w.job_busy = 0;
  return OK;
}
function atiPoll(w, now) {
  if (!w.ati_busy) return OK;
  w.ati_polls++;
  const dur = (w.ati_targets & ATI_TRACKPAD ? ATI_TP_MS : 0) + (w.ati_targets & ATI_ALP ? ATI_ALP_MS : 0);
  if (now - w.ati_t0 < dur && w.ati_polls < ATI_POLL_FALLBACK) return BUSY;
  w.ati_busy = 0;
  if (w.ati_targets & ATI_TRACKPAD) w.srf_tuned = 1;
  w.reati_flag = 1;
  return OK;
}
function chipGestures(w, f, now) {
  const enabled = w.gest_src === GESTURES_CHIP ? w.gest_mask & G_ALL : 0;
  let bits = 0;
  let tapX = XY_INVALID;
  let tapY = XY_INVALID;
  const tapDist = scalePx(w, TOUCH_TAP_DIST);
  if (f.present && !w.chip_was) {
    Object.assign(w, {
      chip_down_t: now,
      chip_down_x: f.x,
      chip_down_y: f.y,
      chip_moved: 0,
      chip_done: 0
    });
  }
  if (f.present) {
    const ddx = f.x - w.chip_down_x;
    const ddy = f.y - w.chip_down_y;
    if (abs(ddx) + abs(ddy) > tapDist) w.chip_moved = 1;
    const held = now - w.chip_down_t;
    if (!(w.chip_done & G_HOLD) && !w.chip_moved && held >= w.tap_ms + w.hold_ms) bits |= G_HOLD;
    if (!(w.chip_done & G_SWIPES) && held <= w.swipe_ms) {
      const dist = effSwipeDist(w);
      const ang = tanAngle(w);
      if (abs(ddx) >= dist && abs(ddy) * 64 <= ang * abs(ddx)) bits |= ddx < 0 ? G_X_NEG : G_X_POS;
      else if (abs(ddy) >= dist && abs(ddx) * 64 <= ang * abs(ddy))
        bits |= ddy < 0 ? G_Y_NEG : G_Y_POS;
    }
    w.chip_last_x = f.x;
    w.chip_last_y = f.y;
  } else if (w.chip_was) {
    if (!w.chip_moved && !(w.chip_done & (G_HOLD | G_SWIPES)) && now - w.chip_down_t <= w.tap_ms) {
      bits |= G_TAP;
      tapX = w.chip_last_x;
      tapY = w.chip_last_y;
    }
  }
  w.chip_was = f.present ? 1 : 0;
  w.chip_done |= bits;
  bits &= enabled;
  return { bits, tapX: bits & G_TAP ? tapX : XY_INVALID, tapY: bits & G_TAP ? tapY : XY_INVALID };
}
function fireTap(w, now) {
  if (w.last_tap_t >= 0 && now - w.last_tap_t <= TOUCH_DOUBLE_MS) {
    w.last_tap_t = -1;
    w.ev_latch |= EV_DOUBLE_TAP;
  } else {
    w.last_tap_t = now;
    w.ev_latch |= EV_TAP;
  }
}
var fireSwipe = (w, dir) => {
  w.ev_latch |= EV_SWIPE | DIR_EV[dir & 3];
};
function touchUpdate(w, now) {
  const drv = w.gest_src === GESTURES_DRIVER;
  const tapDist = scalePx(w, TOUCH_TAP_DIST);
  const moveDead = scalePx(w, TOUCH_MOVE_DEAD);
  const fling = scalePx(w, TOUCH_FLING_PXS);
  let x = w.blk_f1_x;
  let y = w.blk_f1_y;
  let present = cachedFingers(w) > 0 && x !== XY_INVALID && y !== XY_INVALID;
  const was = w.was_down === 1;
  if (present !== was) {
    if (++w.presence_cnt < 2) present = was;
    else w.presence_cnt = 0;
  } else w.presence_cnt = 0;
  if (present && !was) {
    Object.assign(w, {
      px: x,
      py: y,
      down_x: x,
      down_y: y,
      down_t: now,
      move_t: now,
      was_down: 1,
      moved_far: 0,
      hold_fired: 0
    });
    w.ev_latch |= EV_TOUCH_DOWN;
  } else if (present && was) {
    if (x === XY_INVALID || y === XY_INVALID) {
      x = w.px;
      y = w.py;
    }
    const dx = s16(x - w.px);
    const dy = s16(y - w.py);
    if (abs(dx) + abs(dy) >= moveDead) {
      const dt = Math.max(1, now - w.move_t);
      w.vx = s16(Math.trunc(dx * 1e3 / dt));
      w.vy = s16(Math.trunc(dy * 1e3 / dt));
      if (abs(x - w.down_x) + abs(y - w.down_y) > tapDist) {
        w.moved_far = 1;
        w.ev_latch |= EV_DRAG;
      }
      w.px = x;
      w.py = y;
      w.move_t = now;
    }
    if (drv && w.gest_mask & G_HOLD && !w.hold_fired && !w.moved_far && now - w.down_t >= w.tap_ms + w.hold_ms) {
      w.hold_fired = 1;
      w.ev_latch |= EV_LONG_PRESS;
    }
  } else if (!present && was) {
    w.was_down = 0;
    w.last_up_t = now;
    w.ev_latch |= EV_TOUCH_UP;
    if (drv) {
      const travel = abs(w.px - w.down_x) + abs(w.py - w.down_y);
      const avx = abs(w.vx);
      const avy = abs(w.vy);
      if (w.moved_far && travel >= effSwipeDist(w) && (avx > fling || avy > fling)) {
        const major = Math.max(avx, avy);
        const minor = Math.min(avx, avy);
        const dir = avx >= avy ? w.vx > 0 ? DIR_RIGHT : DIR_LEFT : w.vy > 0 ? DIR_DOWN : DIR_UP;
        if (minor * 64 <= tanAngle(w) * major && w.gest_mask & DIR_GESTURE[dir])
          fireSwipe(w, dir);
      }
      if (w.gest_mask & G_TAP && !w.moved_far && !w.hold_fired && now - w.down_t <= w.tap_ms)
        fireTap(w, now);
    }
  }
  if (!drv) {
    const gest = w.blk_gestures & w.gest_mask;
    let fresh = gest & ~w.prev_gestures;
    w.prev_gestures = gest;
    if (!w.was_down && (w.last_up_t < 0 || now - w.last_up_t > TOUCH_CHIP_GATE_MS)) fresh = 0;
    if (fresh & G_TAP) fireTap(w, now);
    if (fresh & G_HOLD) w.ev_latch |= EV_LONG_PRESS;
    if (fresh & G_X_NEG) fireSwipe(w, DIR_LEFT);
    if (fresh & G_X_POS) fireSwipe(w, DIR_RIGHT);
    if (fresh & G_Y_NEG) fireSwipe(w, DIR_UP);
    if (fresh & G_Y_POS) fireSwipe(w, DIR_DOWN);
  }
}
function processOn(w, now) {
  if (w.job_busy) return jobStep(w, now);
  if (w.ati_busy) return atiPoll(w, now);
  if (w.chip_reset_pending) {
    chipReset(w);
    return ERR_RESET;
  }
  const f = liveFinger(w);
  const prevValid = firstFingerValid(w);
  const relX = f.present && prevValid ? s16(f.x - w.blk_f1_x) : 0;
  const relY = f.present && prevValid ? s16(f.y - w.blk_f1_y) : 0;
  const info = liveInfo(w, f, relX !== 0 || relY !== 0);
  const g = chipGestures(w, f, now);
  Object.assign(w, {
    blk_info: info,
    blk_gestures: g.bits,
    blk_rel_x: relX,
    blk_rel_y: relY,
    blk_f1_x: f.present ? f.x : g.tapX,
    blk_f1_y: f.present ? f.y : g.tapY,
    blk_f1_strength: f.strength,
    blk_f1_area: f.area,
    blk_f2_x: XY_INVALID,
    blk_f2_y: XY_INVALID,
    blk_f2_strength: 0,
    blk_f2_area: 0,
    reati_flag: 0
  });
  touchUpdate(w, now);
  return OK;
}
function changed(s, w) {
  const out = {};
  for (const k of Object.keys(w)) if (w[k] !== s[k]) out[k] = w[k];
  return out;
}
function run(s, fn) {
  const w = { ...s };
  const scalar = fn(w);
  return { scalar, nextState: changed(s, w) };
}
function commit(s, next) {
  if (!s.job_busy && s.ati_busy) return { scalar: 0 };
  return { scalar: 1, nextState: next };
}
function nearestElectrode(pos, res, n) {
  if (n <= 1 || res === 0) return 0;
  const k = Math.floor((pos * (n - 1) + Math.floor(res / 2)) / res);
  return Math.min(k, n - 1);
}
function zoneAt(s, x, y) {
  if (!s.srf_applied) return -1;
  const nx = nX(s);
  const ny = nY(s);
  let ix = nearestElectrode(x, effXres(s), nx);
  let iy = nearestElectrode(y, effYres(s), ny);
  if (effFlipX(s)) ix = nx - 1 - ix;
  if (effFlipY(s)) iy = ny - 1 - iy;
  const txI = effSwitch(s) ? ix : iy;
  const rxI = effSwitch(s) ? iy : ix;
  return txI * preset(s).n_rx + rxI;
}
function pct(s, pos, res) {
  if (!firstFingerValid(s) || res === 0) return -1;
  return Math.min(100, Math.floor(pos * 100 / res));
}
var PRODUCT_NUMBER = 763;
var VERSION_MAJOR = 1;
var VERSION_MINOR = 3;
var INVALID_RESPONSE = 61166;
var regFile = (s) => {
  try {
    return JSON.parse(s.reg_file);
  } catch {
    return {};
  }
};
function readReg(s, reg) {
  const f = liveFinger(s);
  switch (reg) {
    case 0:
      return PRODUCT_NUMBER;
    case 1:
      return VERSION_MAJOR;
    case 2:
      return VERSION_MINOR;
    case 16:
      return liveInfo(s, f, false);
    case 20:
      return f.x;
    case 21:
      return f.y;
    case 22:
      return f.strength;
    case 23:
      return f.area;
    case 24:
    case 25:
      return XY_INVALID;
    case 32:
      return touchStatus(s) & 65535;
    case 35:
      return alpLta(s) + (s.mode >= MODE_LP1 ? alpDelta(s) : 0);
    case 36:
      return alpLta(s);
    case 73:
      if (s.ref_update) return s.ref_update;
      break;
    case 74:
      return s.i2c_timeout;
    case 81:
      return configWord(s);
    case 83:
      return s.mult_clear << 8 | s.mult_set;
    case 84:
      return s.alp_threshold;
    case 96:
      return preset(s).n_rx << 8 | 40 | // MAV + IIR filters (the surface default)
      (effSwitch(s) ? 4 : 0) | (effFlipY(s) ? 2 : 0) | effFlipX(s);
    case 97:
      return (s.max_touches_user || preset(s).max_touches) << 8 | preset(s).n_tx;
    case 98:
      return effXres(s);
    case 99:
      return effYres(s);
    case 128:
      return s.gest_src === GESTURES_CHIP ? s.gest_mask & G_ALL : 0;
    case 129:
      return s.tap_ms;
    case 130:
      return scalePx(s, TOUCH_TAP_DIST);
    case 131:
      return s.hold_ms;
    case 132:
      return s.swipe_ms;
    case 133:
    case 134:
      return effSwipeDist(s);
    case 135:
      return tanAngle(s);
  }
  if (reg >= 64 && reg <= 68) return s[RATE_FIELDS[reg - 64]];
  if (reg >= 69 && reg <= 72) return effTimeout(s, reg - 69);
  return regFile(s)[String(reg)] ?? 0;
}
function writeForbidden(reg, v) {
  const mapBad = (b) => b === 0 || b === 10;
  if (v === INVALID_RESPONSE) return true;
  if (reg === 80 && v & 1 << 15) return true;
  if (reg === 114 && v & 1) return true;
  if (reg === 115 && v & (1 << 0 | 1 << 10)) return true;
  return reg >= 144 && reg <= 149 && (mapBad(v & 255) || mapBad(v >> 8));
}
function writeReg(s, reg, v) {
  if (reg === 80) return v & 1 << 9 ? { chip_reset_pending: 1 } : {};
  if (reg >= 64 && reg <= 68) return { [RATE_FIELDS[reg - 64]]: v };
  if (reg >= 69 && reg <= 72) return { [TMO_FIELDS[reg - 69]]: v };
  if (reg === 74) return { i2c_timeout: v };
  if (reg === 83) return { mult_set: v & 255, mult_clear: v >> 8 };
  if (reg === 84) return { alp_threshold: v };
  if (reg === 129) return { tap_ms: v };
  if (reg === 131) return { hold_ms: v };
  if (reg === 132) return { swipe_ms: v };
  const file = regFile(s);
  file[String(reg)] = v;
  return { reg_file: JSON.stringify(file) };
}
var DEFAULT_STATE = {
  touching: 0,
  finger_x_pm: 500,
  finger_y_pm: 500,
  pressure_pct: 50,
  hand_near: 0,
  mode: MODE_ACTIVE,
  mode_t: 0,
  last_act_t: 0,
  alp_seen_t: -1,
  tick_fx: 500,
  tick_fy: 500,
  tick_finger: 0,
  reati_flag: 0,
  chip_reset_pending: 0,
  reg_file: "{}",
  chip_was: 0,
  chip_down_t: 0,
  chip_down_x: 0,
  chip_down_y: 0,
  chip_last_x: 0,
  chip_last_y: 0,
  chip_moved: 0,
  chip_done: 0,
  t_ms: 0,
  layout: LAYOUT_NONE,
  sens: 2,
  mult_set: SURFACE_SET_MULT,
  mult_clear: SURFACE_CLEAR_MULT,
  profile: 1,
  rate_active: 10,
  rate_idle_touch: 20,
  rate_idle: 20,
  rate_lp1: 50,
  rate_lp2: 100,
  tmo_active: 10,
  tmo_idle_touch: 30,
  tmo_idle: 20,
  tmo_lp1: 60,
  power_mode: POWER_AUTO,
  alp_wake: 0,
  alp_threshold: 20,
  user_flip_x: 0,
  user_flip_y: 0,
  user_switch: 0,
  user_xres: 0,
  user_yres: 0,
  tap_ms: 300,
  hold_ms: 300,
  swipe_ms: 500,
  swipe_dist_user: 0,
  swipe_angle_deg: 45,
  gest_src: GESTURES_DRIVER,
  gest_mask: G_ALL,
  max_touches_user: 0,
  ref_update: 0,
  i2c_timeout: 50,
  wdt: 1,
  srf_applied: 0,
  srf_tuned: 0,
  job_busy: 1,
  // init() leaves the baseline job pending
  job_polls: 0,
  job_t0: 0,
  ati_busy: 0,
  ati_targets: 0,
  ati_polls: 0,
  ati_t0: 0,
  blk_info: 0,
  blk_gestures: 0,
  blk_rel_x: 0,
  blk_rel_y: 0,
  blk_f1_x: 0,
  blk_f1_y: 0,
  blk_f1_strength: 0,
  blk_f1_area: 0,
  blk_f2_x: 0,
  blk_f2_y: 0,
  blk_f2_strength: 0,
  blk_f2_area: 0,
  was_down: 0,
  presence_cnt: 0,
  down_x: 0,
  down_y: 0,
  down_t: 0,
  move_t: 0,
  px: 0,
  py: 0,
  moved_far: 0,
  hold_fired: 0,
  vx: 0,
  vy: 0,
  last_tap_t: -1,
  last_up_t: -1,
  prev_gestures: 0,
  ev_latch: 0
};
var FINGER_CONTROLS = [
  { kind: "toggle", id: "touch", label: "Finger on surface", fields: ["touching"] },
  {
    kind: "slider",
    id: "finger_x",
    label: "Finger along the Tx columns",
    field: "finger_x_pm",
    min: 0,
    max: 1e3,
    step: 1,
    unit: "\u2030"
  },
  {
    kind: "slider",
    id: "finger_y",
    label: "Finger along the Rx rows",
    field: "finger_y_pm",
    min: 0,
    max: 1e3,
    step: 1,
    unit: "\u2030"
  },
  {
    kind: "slider",
    id: "pressure",
    label: "Finger pressure",
    field: "pressure_pct",
    min: 0,
    max: 100,
    step: 1,
    unit: "%"
  },
  { kind: "toggle", id: "hand_near", label: "Hand near surface (ALP)", fields: ["hand_near"] }
];
var sim = {
  tile: "Sense.CAP",
  defaultState: DEFAULT_STATE,
  controls: [
    { type: "toggle", field: "touching", label: "Finger on surface" },
    {
      type: "slider",
      field: "finger_x_pm",
      label: "Finger along the Tx columns",
      min: 0,
      max: 1e3,
      step: 1,
      unit: "\u2030"
    },
    {
      type: "slider",
      field: "finger_y_pm",
      label: "Finger along the Rx rows",
      min: 0,
      max: 1e3,
      step: 1,
      unit: "\u2030"
    },
    {
      type: "slider",
      field: "pressure_pct",
      label: "Finger pressure",
      min: 0,
      max: 100,
      step: 1,
      unit: "%"
    },
    { type: "toggle", field: "hand_near", label: "Hand near surface (ALP)" }
  ],
  stimuli: [{ id: "finger", label: "Finger", controls: FINGER_CONTROLS }],
  hostCalls: {
    // ── lifecycle ──
    tile_sense_cap_find: () => ({ scalar: 1 }),
    tile_sense_cap_init: () => void 0,
    tile_sense_cap_set_layout: ({ state, args }) => {
      const layout = arg(args, 0);
      if (layout < LAYOUT_NONE || layout > LAYOUT_SLIDER_1X4) return { scalar: ERR_ARG };
      if (state.ati_busy) return { scalar: ERR_STATE };
      return {
        scalar: OK,
        nextState: { layout, ...sensitivityMults(state.sens), ...startJob(state) }
      };
    },
    tile_sense_cap_surface_poll: ({ state }) => state.job_busy ? run(state, (w) => jobStep(w, w.t_ms)) : { scalar: OK },
    // No job or ATI pending and, with a surface, applied and tuned (NONE:
    // ready once the baseline job is done).
    tile_sense_cap_is_surface_ready: ({ state }) => ({
      scalar: !state.job_busy && !state.ati_busy && (!preset(state).has_srf || state.srf_applied && state.srf_tuned) ? 1 : 0
    }),
    tile_sense_cap_ati_start: ({ state, args }) => {
      let t = arg(args, 0);
      if (t < 1 || t > 3) return { scalar: ERR_ARG };
      if (state.job_busy || state.ati_busy) return { scalar: ERR_STATE };
      if (!preset(state).alp) t &= ~ATI_ALP;
      if (t & ATI_TRACKPAD && !state.srf_applied) t &= ~ATI_TRACKPAD;
      if (!t) return { scalar: ERR_STATE };
      return {
        scalar: OK,
        nextState: {
          ati_busy: 1,
          ati_targets: t,
          ati_polls: 0,
          ati_t0: state.t_ms,
          ...t & ATI_TRACKPAD ? { srf_tuned: 0 } : {}
        }
      };
    },
    tile_sense_cap_ati_poll: ({ state }) => state.ati_busy ? run(state, (w) => atiPoll(w, w.t_ms)) : { scalar: OK },
    // Studio cannot supply cfg.mclr (the platform layer has no GPIO write).
    tile_sense_cap_hw_reset: () => ({ scalar: ERR_NO_CALLBACK }),
    tile_sense_cap_reset: ({ state }) => run(state, (w) => {
      chipReset(w);
      return OK;
    }),
    tile_sense_cap_get_version_major: () => ({ scalar: VERSION_MAJOR }),
    tile_sense_cap_get_version_minor: () => ({ scalar: VERSION_MINOR }),
    tile_sense_cap_get_settings_version: ({ state }) => ({ scalar: readReg(state, 116) }),
    // ── event processing ──
    tile_sense_cap_process: ({ state }) => run(state, (w) => processOn(w, w.t_ms)),
    tile_sense_cap_on_event: () => void 0,
    // fired from process(); nothing to hold
    // ── runtime data: the process() cache ──
    tile_sense_cap_get_touch_events: ({ state }) => ({
      scalar: state.ev_latch,
      nextState: { ev_latch: 0 }
    }),
    tile_sense_cap_was_tapped: ({ state }) => {
      const taps = state.ev_latch & (EV_TAP | EV_DOUBLE_TAP);
      return { scalar: taps ? 1 : 0, nextState: { ev_latch: state.ev_latch & ~taps } };
    },
    tile_sense_cap_get_zone: ({ state }) => ({
      scalar: firstFingerValid(state) ? zoneAt(state, state.blk_f1_x, state.blk_f1_y) : -1
    }),
    tile_sense_cap_zone_at: ({ state, args }) => ({
      scalar: zoneAt(state, u16(arg(args, 0)), u16(arg(args, 1)))
    }),
    tile_sense_cap_get_x_pct: ({ state }) => ({
      scalar: pct(state, state.blk_f1_x, effXres(state))
    }),
    tile_sense_cap_get_y_pct: ({ state }) => ({
      scalar: pct(state, state.blk_f1_y, effYres(state))
    }),
    // process() every 5 ms until a finger or the timeout (the world holds still).
    tile_sense_cap_wait_for_touch: ({ state, args }) => {
      const timeout = Math.max(0, arg(args, 0));
      return run(state, (w) => {
        const t0 = w.t_ms;
        for (let i = 0, now = t0; i <= 1e3; i++, now += 5) {
          processOn(w, now);
          if (cachedFingers(w) > 0) return 1;
          if (now - t0 >= timeout) return 0;
        }
        return 0;
      });
    },
    tile_sense_cap_get_info_flags: ({ state }) => ({ scalar: state.blk_info }),
    tile_sense_cap_get_gestures: ({ state }) => ({ scalar: state.blk_gestures }),
    tile_sense_cap_get_num_fingers: ({ state }) => ({ scalar: cachedFingers(state) }),
    tile_sense_cap_get_finger_x: ({ state, args }) => {
      const n = u8(arg(args, 0));
      return { scalar: n === 0 ? state.blk_f1_x : n === 1 ? state.blk_f2_x : 0 };
    },
    tile_sense_cap_get_finger_y: ({ state, args }) => {
      const n = u8(arg(args, 0));
      return { scalar: n === 0 ? state.blk_f1_y : n === 1 ? state.blk_f2_y : 0 };
    },
    tile_sense_cap_get_finger_strength: ({ state, args }) => {
      const n = u8(arg(args, 0));
      return { scalar: n === 0 ? state.blk_f1_strength : n === 1 ? state.blk_f2_strength : 0 };
    },
    tile_sense_cap_get_finger_area: ({ state, args }) => {
      const n = u8(arg(args, 0));
      return { scalar: n === 0 ? state.blk_f1_area : n === 1 ? state.blk_f2_area : 0 };
    },
    tile_sense_cap_get_relative_x: ({ state }) => ({ scalar: state.blk_rel_x }),
    tile_sense_cap_get_relative_y: ({ state }) => ({ scalar: state.blk_rel_y }),
    tile_sense_cap_is_alp_active: ({ state }) => ({
      scalar: state.blk_info & INFO_ALP_OUTPUT ? 1 : 0
    }),
    tile_sense_cap_get_mode: ({ state }) => ({ scalar: state.blk_info & INFO_MODE_MASK }),
    tile_sense_cap_has_ati_error: ({ state }) => ({
      scalar: state.blk_info & INFO_ATI_ERROR || alpInUse(state) && state.blk_info & INFO_ALP_ATI_ERROR ? 1 : 0
    }),
    // ── live reads ──
    tile_sense_cap_get_touch_status: ({ state }) => ({ scalar: touchStatus(state) }),
    tile_sense_cap_is_touched: ({ state, args }) => {
      const ch = u8(arg(args, 0));
      return { scalar: ch <= 31 && touchStatus(state) >> ch & 1 ? 1 : 0 };
    },
    tile_sense_cap_get_num_channels: ({ state }) => ({
      scalar: state.srf_applied ? nCh(state) : 0
    }),
    tile_sense_cap_get_channel_count: ({ state, args }) => ({
      scalar: channelCount(state, u8(arg(args, 0)))
    }),
    tile_sense_cap_get_channel_delta: ({ state, args }) => ({
      scalar: channelDelta(state, u8(arg(args, 0)))
    }),
    tile_sense_cap_read_channel_counts: ({ state, caps }) => {
      const n = u8(caps?.["n"] ?? 0);
      if (n === 0 || n > 32) return { scalar: 0 };
      return {
        scalar: n,
        out: { counts: Array.from({ length: n }, (_, ch) => channelCount(state, ch)) }
      };
    },
    tile_sense_cap_read_channel_deltas: ({ state, caps }) => {
      const n = u8(caps?.["n"] ?? 0);
      if (n === 0 || n > 32) return { scalar: 0 };
      return {
        scalar: n,
        out: { deltas: Array.from({ length: n }, (_, ch) => channelDelta(state, ch)) }
      };
    },
    tile_sense_cap_get_alp_count: ({ state }) => ({ scalar: readReg(state, 35) }),
    tile_sense_cap_get_alp_lta: ({ state }) => ({ scalar: readReg(state, 36) }),
    // ── configuration ──
    tile_sense_cap_set_sensitivity: ({ state, args }) => {
      const level = arg(args, 0);
      if (level < 0 || level > 4) return { scalar: 0 };
      return commit(state, { sens: level, ...sensitivityMults(level) });
    },
    tile_sense_cap_set_touch_multipliers: ({ state, args }) => {
      const set = u8(arg(args, 0));
      const clr = u8(arg(args, 1));
      if (set < 1) return { scalar: 0 };
      return commit(state, { mult_set: set, mult_clear: Math.min(clr, set) });
    },
    tile_sense_cap_set_power_profile: ({ state, args }) => {
      const p = arg(args, 0);
      const prof = PROFILES[p];
      if (p < 0 || !prof) return { scalar: 0 };
      const next = { profile: p };
      RATE_FIELDS.forEach((f, i) => next[f] = prof[0][i]);
      TMO_FIELDS.forEach((f, i) => next[f] = prof[1][i]);
      return commit(state, next);
    },
    tile_sense_cap_set_power_mode: ({ state, args }) => {
      const m = arg(args, 0);
      if (m < POWER_AUTO || m > POWER_FORCE_LP2) return { scalar: 0 };
      return commit(state, { power_mode: m });
    },
    tile_sense_cap_set_active_rate: ({ state, args }) => commit(state, { rate_active: Math.min(1e3, Math.max(10, u16(arg(args, 0)))) }),
    tile_sense_cap_set_idle_timeout: ({ state, args }) => commit(state, { tmo_idle: Math.min(3600, u16(arg(args, 0))) }),
    tile_sense_cap_set_report_rate: ({ state, args }) => {
      const mode = arg(args, 0);
      const ms = u16(arg(args, 1));
      if (mode < 0 || mode > MODE_LP2 || ms === 0) return { scalar: 0 };
      return commit(state, { [RATE_FIELDS[mode]]: ms });
    },
    tile_sense_cap_set_mode_timeout: ({ state, args }) => {
      const mode = arg(args, 0);
      if (mode < 0 || mode > MODE_LP1) return { scalar: 0 };
      return commit(state, { [TMO_FIELDS[mode]]: u16(arg(args, 1)) });
    },
    tile_sense_cap_set_alp_wake: ({ state, args }) => {
      const en = u8(arg(args, 0)) ? 1 : 0;
      if (en && !preset(state).alp) return { scalar: 0 };
      return commit(state, { alp_wake: en });
    },
    tile_sense_cap_set_alp_threshold: ({ state, args }) => commit(state, { alp_threshold: Math.min(1e3, Math.max(1, u16(arg(args, 0)))) }),
    tile_sense_cap_set_flip_x: ({ state, args }) => commit(state, { user_flip_x: u8(arg(args, 0)) ? 1 : 0 }),
    tile_sense_cap_set_flip_y: ({ state, args }) => commit(state, { user_flip_y: u8(arg(args, 0)) ? 1 : 0 }),
    tile_sense_cap_set_switch_xy: ({ state, args }) => commit(state, { user_switch: u8(arg(args, 0)) ? 1 : 0 }),
    tile_sense_cap_set_resolution: ({ state, args }) => commit(state, { user_xres: u16(arg(args, 0)), user_yres: u16(arg(args, 1)) }),
    tile_sense_cap_set_tap_time: ({ state, args }) => commit(state, { tap_ms: Math.min(2e3, Math.max(50, u16(arg(args, 0)))) }),
    tile_sense_cap_set_hold_time: ({ state, args }) => commit(state, { hold_ms: Math.min(5e3, u16(arg(args, 0))) }),
    tile_sense_cap_set_swipe_time: ({ state, args }) => commit(state, { swipe_ms: Math.min(2e3, Math.max(50, u16(arg(args, 0)))) }),
    tile_sense_cap_set_swipe_distance: ({ state, args }) => commit(state, { swipe_dist_user: u16(arg(args, 0)) }),
    tile_sense_cap_set_swipe_angle: ({ state, args }) => {
      const deg = u8(arg(args, 0));
      if (deg < 1 || deg > 75) return { scalar: 0 };
      return commit(state, { swipe_angle_deg: deg });
    },
    tile_sense_cap_set_gesture_source: ({ state, args }) => {
      const src = arg(args, 0);
      if (src < GESTURES_DRIVER || src > GESTURES_CHIP) return { scalar: 0 };
      return commit(state, { gest_src: src, prev_gestures: 0 });
    },
    tile_sense_cap_enable_gestures: ({ state, args }) => commit(state, { gest_mask: u16(arg(args, 0)) & G_ALL }),
    tile_sense_cap_set_max_touches: ({ state, args }) => {
      const n = u8(arg(args, 0));
      if (n < 1 || n > 2) return { scalar: 0 };
      return commit(state, { max_touches_user: n });
    },
    tile_sense_cap_set_reference_update: ({ state, args }) => {
      const sec = u16(arg(args, 0));
      if (sec > 60) return { scalar: 0 };
      return commit(state, { ref_update: sec });
    },
    tile_sense_cap_set_i2c_timeout: ({ state, args }) => commit(state, { i2c_timeout: Math.min(1e3, Math.max(2, u16(arg(args, 0)))) }),
    tile_sense_cap_set_watchdog: ({ state, args }) => commit(state, { wdt: u8(arg(args, 0)) ? 1 : 0 }),
    // References re-seed onto the current counts: nothing held to change.
    tile_sense_cap_reseed: ({ state }) => ({ scalar: state.job_busy || state.ati_busy ? 0 : 1 }),
    // ── escape hatch ──
    tile_sense_cap_read_reg: ({ state, args }) => ({ scalar: readReg(state, u8(arg(args, 0))) }),
    tile_sense_cap_write_reg: ({ state, args }) => {
      const reg = u8(arg(args, 0));
      const v = u16(arg(args, 1));
      if (writeForbidden(reg, v)) return { scalar: ERR_FORBIDDEN };
      if (state.job_busy || state.ati_busy) return { scalar: ERR_STATE };
      return { scalar: OK, nextState: writeReg(state, reg, v) };
    }
  },
  provenance: {
    // canonical: the datasheet fixes the value, bit or rule
    tile_sense_cap_find: "canonical",
    // product 763 at 0x56 (§11.2, Table A.1)
    tile_sense_cap_get_version_major: "canonical",
    // Table A.1: 1
    tile_sense_cap_get_version_minor: "canonical",
    // Table A.1: 3
    tile_sense_cap_get_touch_status: "canonical",
    // bit n = channel n (Table A.4, §5.1.1); threshold ref*(1+set/128) (§5.5.1)
    tile_sense_cap_is_touched: "canonical",
    tile_sense_cap_get_finger_strength: "canonical",
    // sum of the finger's deltas (§7.2.4)
    tile_sense_cap_get_finger_area: "canonical",
    // channels associated with the finger (§7.2.5)
    tile_sense_cap_get_num_fingers: "canonical",
    // Info Flags [9:8]
    tile_sense_cap_get_gestures: "canonical",
    // Table A.3 bits, chip engine only, one report
    tile_sense_cap_set_touch_multipliers: "canonical",
    // 0x53 high clear / low set
    tile_sense_cap_set_resolution: "canonical",
    // 0x62/0x63; 0 = 256 per pitch (§7.4)
    tile_sense_cap_set_flip_x: "canonical",
    // §7.7
    tile_sense_cap_set_flip_y: "canonical",
    tile_sense_cap_set_switch_xy: "canonical",
    tile_sense_cap_set_swipe_angle: "canonical",
    // 64*tan(deg) (0x87, §8.3)
    tile_sense_cap_write_reg: "inferred",
    // guards per the driver; modeled registers map to state, the rest held sparse
    // inferred: follows the driver; magnitudes / timing modeled
    tile_sense_cap_set_layout: "inferred",
    // preset table from tile_sense_cap_layout_preset
    tile_sense_cap_surface_poll: "inferred",
    // job timing collapsed: a few polls + ~1 s ATI
    tile_sense_cap_is_surface_ready: "inferred",
    tile_sense_cap_ati_start: "inferred",
    tile_sense_cap_ati_poll: "inferred",
    // always converges
    tile_sense_cap_hw_reset: "inferred",
    // no cfg.mclr in Studio: ERR_NO_CALLBACK
    tile_sense_cap_reset: "inferred",
    tile_sense_cap_process: "inferred",
    // caches 0x10-0x1B, runs the recognizers; BUSY only while a job / ATI runs
    tile_sense_cap_on_event: "inferred",
    tile_sense_cap_get_touch_events: "inferred",
    // driver recognizers on the twin clock
    tile_sense_cap_was_tapped: "inferred",
    tile_sense_cap_get_zone: "inferred",
    // driver zone_at over the cached XY
    tile_sense_cap_zone_at: "inferred",
    tile_sense_cap_get_x_pct: "inferred",
    // truncating, per the driver
    tile_sense_cap_get_y_pct: "inferred",
    tile_sense_cap_wait_for_touch: "inferred",
    tile_sense_cap_get_info_flags: "inferred",
    // bit layout canonical; which bits rise is modeled
    tile_sense_cap_get_mode: "inferred",
    // §6 state diagram, timeouts modeled
    tile_sense_cap_is_alp_active: "inferred",
    // LP1/LP2 + ALP wake only (§5.2)
    tile_sense_cap_has_ati_error: "inferred",
    // the twin's ATI never fails
    tile_sense_cap_get_finger_x: "inferred",
    // interpolated position; empty slot 0xFFFF (hardware)
    tile_sense_cap_get_finger_y: "inferred",
    tile_sense_cap_get_relative_x: "inferred",
    // change since the previous read
    tile_sense_cap_get_relative_y: "inferred",
    tile_sense_cap_get_num_channels: "inferred",
    tile_sense_cap_get_channel_count: "inferred",
    // ATI target 900 + a bench-scaled finger delta
    tile_sense_cap_get_channel_delta: "inferred",
    tile_sense_cap_read_channel_counts: "inferred",
    tile_sense_cap_read_channel_deltas: "inferred",
    tile_sense_cap_get_alp_lta: "inferred",
    // ALP target 200 per prox block
    tile_sense_cap_set_sensitivity: "inferred",
    // driver's x2 / x1.5 / x1 / x0.75 / x0.5
    tile_sense_cap_set_power_profile: "inferred",
    // driver PROFILES table
    tile_sense_cap_set_power_mode: "inferred",
    tile_sense_cap_set_active_rate: "inferred",
    tile_sense_cap_set_idle_timeout: "inferred",
    tile_sense_cap_set_report_rate: "inferred",
    tile_sense_cap_set_mode_timeout: "inferred",
    tile_sense_cap_set_alp_wake: "inferred",
    tile_sense_cap_set_alp_threshold: "inferred",
    tile_sense_cap_set_tap_time: "inferred",
    tile_sense_cap_set_hold_time: "inferred",
    tile_sense_cap_set_swipe_time: "inferred",
    tile_sense_cap_set_swipe_distance: "inferred",
    tile_sense_cap_set_gesture_source: "inferred",
    tile_sense_cap_enable_gestures: "inferred",
    tile_sense_cap_set_max_touches: "inferred",
    tile_sense_cap_set_reference_update: "inferred",
    tile_sense_cap_set_i2c_timeout: "inferred",
    tile_sense_cap_set_watchdog: "inferred",
    tile_sense_cap_reseed: "inferred",
    tile_sense_cap_read_reg: "inferred",
    // the driver's register image; unmodeled addresses read 0
    // hallucinated: placeholders
    tile_sense_cap_get_alp_count: "hallucinated",
    // hand / finger ALP deltas are guesses
    tile_sense_cap_get_settings_version: "hallucinated",
    // a part with no GUI image: 0
    power: "inferred"
    // §3.4 Table 3.4 is a 30-channel pad; the tile runs <= 6
  },
  // Charging mode per the §6 state diagram (automatic control), or held by
  // set_power_mode / an ALP ATI (manual control, §6.3). Modeled: Active →
  // Idle-Touch (finger still) or Idle (no finger) after the Active timeout;
  // Idle-Touch → Active on movement, → Idle on release; Idle → Active on a
  // finger, → LP1 after the Idle timeout (0 = never unless ALP wake is on);
  // LP1 → LP2 after the LP1 timeout; LP1/LP2 → Active one LP report period
  // after the ALP output rises. Not modeled: the Idle-Touch timeout's reseed.
  deriveState(state, { t }) {
    const u = { t_ms: t };
    const finger = liveFinger(state).present;
    const moved = finger && (state.finger_x_pm !== state.tick_fx || state.finger_y_pm !== state.tick_fy);
    if (state.finger_x_pm !== state.tick_fx) u.tick_fx = state.finger_x_pm;
    if (state.finger_y_pm !== state.tick_fy) u.tick_fy = state.finger_y_pm;
    const activity = moved || (finger ? 1 : 0) !== state.tick_finger;
    if ((finger ? 1 : 0) !== state.tick_finger) u.tick_finger = finger ? 1 : 0;
    const lastAct = activity ? t : state.last_act_t;
    if (activity) u.last_act_t = t;
    const alp = alpOutput(state);
    if (alp && state.alp_seen_t < 0) u.alp_seen_t = t;
    if (!alp && state.alp_seen_t >= 0) u.alp_seen_t = -1;
    let mode = state.mode;
    const alpAti = state.ati_busy && state.ati_targets & ATI_ALP && (!(state.ati_targets & ATI_TRACKPAD) || t - state.ati_t0 >= ATI_TP_MS);
    if (alpAti || state.power_mode === POWER_FORCE_LP1) mode = MODE_LP1;
    else if (state.power_mode === POWER_FORCE_ACTIVE) mode = MODE_ACTIVE;
    else if (state.power_mode === POWER_FORCE_LP2) mode = MODE_LP2;
    else {
      const since = (sec, from) => sec > 0 && t - from >= sec * 1e3;
      switch (state.mode) {
        case MODE_ACTIVE:
          if (since(state.tmo_active, lastAct)) mode = finger ? MODE_IDLE_TOUCH : MODE_IDLE;
          break;
        case MODE_IDLE_TOUCH:
          if (!finger) mode = MODE_IDLE;
          else if (moved) mode = MODE_ACTIVE;
          break;
        case MODE_IDLE:
          if (finger) mode = MODE_ACTIVE;
          else if (since(effTimeout(state, MODE_IDLE), state.mode_t)) mode = MODE_LP1;
          break;
        default: {
          const period = state.mode === MODE_LP1 ? state.rate_lp1 : state.rate_lp2;
          if (alp && state.alp_seen_t >= 0 && t - state.alp_seen_t >= period) mode = MODE_ACTIVE;
          else if (state.mode === MODE_LP1 && since(state.tmo_lp1, state.mode_t)) mode = MODE_LP2;
        }
      }
    }
    if (mode !== state.mode) {
      u.mode = mode;
      u.mode_t = t;
      if (mode === MODE_ACTIVE) u.last_act_t = t;
    }
    return u;
  },
  // Pad 3 is the chip's MCLR reset INPUT (4.7k pull-up to V+ on the tile);
  // RDY is not connected on rev a. The electrode pads are analog sense lines.
  // The tile drives nothing.
  padOutputs() {
    return {};
  },
  // Electrical: a pure load on V+ (pad 10) / GND (pad 1). Draw follows the
  // live charging mode, from datasheet §3.4 Table 3.4.
  power(state, ctx) {
    const ua = MODE_UA[state.mode] ?? MODE_UA[MODE_ACTIVE];
    return {
      draw_ua: ua,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: ctx?.padVoltage?.["10"] ?? 3300,
          i_ua: ua,
          pads: ["10"],
          note: `IQS7211A ${MODE_NAME[state.mode] ?? "Active"} (Table 3.4, 30-ch reference pad${state.mode === MODE_IDLE_TOUCH ? "; Idle figure, no Idle-Touch row" : ""})`
        }
      ]
    };
  }
};
var sense_cap_default = sim;
export {
  sense_cap_default as default
};
