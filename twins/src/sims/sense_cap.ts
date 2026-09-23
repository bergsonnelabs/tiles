// Digital twin for Sense.CAP — Azoteq IQS7211A mutual-capacitance trackpad.
//
// The tile carries the WLCSP18 controller only; the electrode surface is
// external, attached via tile pads. The r0 surface is a 2x3 matrix: two Rx
// strips (rows, tile pads 2/9 = RX3/RX6) across three Tx blocks (columns,
// tile pads 6/7/8 = TX11/TX9/TX8). The driver programs that geometry with
// setup_2x3() / configure_surface(); before that runs, the chip has no
// meaningful trackpad output — mirrored here by gating XY/touch reads on
// `configured`.
//
// Physical world (controls): one finger at an XY position in the configured
// output resolution (X 0-511 across the three Tx columns, Y 0-255 across the
// two Rx rows), plus an ALP approach toggle and a power-mode selector.
// Channel numbering follows the datasheet rule (§5.1.1, along the Rxs first
// then per Tx): column c / row r -> channel c*2 + r.
//
// Pad map (from the r0 PCB netlist): GND (1), RX3 (2), /RDY (3, 100k pull-up
// on tile), SCL (4), SDA (5), TX11 (6), TX9/TX10 (7), TX8 (8), RX6 (9),
// V+ (10). The sense lines are analog electrodes, not driven outputs; RDY is
// the one tile-driven digital output.
import type { TileSim } from '../tileSim';

// Geometry of the r0 surface (driver sense_cap_surface_2x3).
const COLS = 3; // Tx blocks, X axis (switch_xy)
const ROWS = 2; // Rx strips, Y axis
const NUM_CH = COLS * ROWS;
const X_RES = 512; // (COLS-1) * 256 points between electrodes
const Y_RES = 256; // (ROWS-1) * 256

// Modeled counts: ATI settles every channel at the surface's target; a touch
// raises the pressed channel's count past the touch threshold (reference *
// multiplier/128, §5.5.1). The driver's r0 surface (sense_cap_surface_2x3,
// tile_sense_cap.c:521) runs ATI target 900 and set/clear multipliers 8/5
// (~56/35 counts); a finger measured ~100-190 delta counts on hardware.
const ATI_TARGET = 900;
const TOUCH_DELTA = 150; // mid of the measured 100-190 count finger signal
const NEIGHBOR_DELTA = 30; // partial coupling; below even the lightest set threshold (35)
const ALP_LTA = 200; // surface alp_ati_target
const ALP_DELTA = 40;

// set_sensitivity(1..5) → (set, clear) multiplier pairs (tile_sense_cap.c:888).
const SENSITIVITY_PAIRS: readonly (readonly [number, number])[] = [
  [48, 28],
  [28, 16],
  [16, 10],
  [10, 6],
  [5, 3],
];

// Charging modes (sense_cap_mode_t / info-flags[2:0]).
const MODE_ACTIVE = 0;
const MODE_IDLE_TOUCH = 1;
const MODE_IDLE = 2;
const MODE_LP1 = 3;
const MODE_LP2 = 4;

// Info-flag bits the driver exposes (IQS7211A_INFO_*).
const INFO_RE_ATI_OCCURRED = 1 << 4;
const INFO_NUM_FINGERS_SHIFT = 8;

// Supply current per mode, µA — datasheet §2.3 bench figures for a 5x6
// trackpad at ATI 300 / 1.4 MHz; the 2x3 surface runs proportionally lower.
const I_ACTIVE_UA = 1150;
const I_IDLE_UA = 160;
const I_LP1_UA = 9;
const I_LP2_UA = 6;

interface State {
  // ── physical world ──
  touching: number; // finger down on the surface
  finger_x: number; // 0..X_RES-1, along the three Tx columns
  finger_y: number; // 0..Y_RES-1, along the two Rx rows
  alp_approach: number; // hand near the surface (ALP wake channel)
  gesture_code: number; // 0 none, 1..6 -> SENSE_CAP_GESTURE_* bit index

  // ── operation ──
  mode: number; // charging mode 0..4
  configured: number; // 1 once a surface config has run

  // ── configuration (tracked) ──
  x_res: number;
  y_res: number;
  max_touches: number;
  gestures_mask: number;
  event_mode: number;
  touch_set_mult: number;
  touch_clear_mult: number;
  sensitivity: number;
  alp_threshold: number;
  active_rate_ms: number;

  [field: string]: number;
}

const pick = (args: number[], i: number, cur: number) =>
  args.length > i && Number.isFinite(args[i]) ? args[i] : cur;
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

// Which channel the finger sits on, and its neighbors within the column/row.
function fingerChannel(s: State): number {
  const col = clamp(Math.floor((s.finger_x / s.x_res) * COLS), 0, COLS - 1);
  const row = clamp(Math.floor((s.finger_y / s.y_res) * ROWS), 0, ROWS - 1);
  return col * ROWS + row;
}

function deltaOf(s: State, ch: number): number {
  if (!s.configured || !s.touching || ch >= NUM_CH) return 0;
  const pressed = fingerChannel(s);
  if (ch === pressed) return TOUCH_DELTA;
  // same column (other row) or same row (adjacent column) couples weakly
  const col = Math.floor(ch / ROWS);
  const pcol = Math.floor(pressed / ROWS);
  const sameCol = col === pcol;
  const sameRow = ch % ROWS === pressed % ROWS && Math.abs(col - pcol) === 1;
  return sameCol || sameRow ? NEIGHBOR_DELTA : 0;
}

const countOf = (s: State, ch: number): number =>
  !s.configured || ch >= NUM_CH ? 0 : ATI_TARGET + deltaOf(s, ch);

// Touch status: a channel reports touch when count > ref*(1 + set_mult/128).
function touchBit(s: State, ch: number): number {
  const threshold = ATI_TARGET * (s.touch_set_mult / 128);
  return deltaOf(s, ch) > threshold ? 1 : 0;
}

function touchStatus(s: State): number {
  let st = 0;
  for (let ch = 0; ch < NUM_CH; ch++) if (touchBit(s, ch)) st |= 1 << ch;
  return st;
}

const numFingers = (s: State) => (s.configured && s.touching ? 1 : 0);

// Info-flags word: charging mode [2:0] + finger count [9:8].
const infoFlags = (s: State) =>
  (s.mode & 7) | (numFingers(s) << INFO_NUM_FINGERS_SHIFT) | INFO_RE_ATI_OCCURRED;

const gestureWord = (s: State) =>
  s.gesture_code > 0 && (s.gestures_mask & (1 << (s.gesture_code - 1))) !== 0
    ? 1 << (s.gesture_code - 1)
    : 0;

function modeCurrentUa(mode: number): number {
  switch (mode) {
    case MODE_LP1:
      return I_LP1_UA;
    case MODE_LP2:
      return I_LP2_UA;
    case MODE_IDLE_TOUCH:
    case MODE_IDLE:
      return I_IDLE_UA;
    default:
      return I_ACTIVE_UA;
  }
}

const sim: TileSim<State> = {
  tile: 'Sense.CAP',

  defaultState: {
    touching: 0,
    finger_x: 256,
    finger_y: 128,
    alp_approach: 0,
    gesture_code: 0,

    mode: MODE_ACTIVE,
    configured: 0,

    x_res: X_RES,
    y_res: Y_RES,
    max_touches: 2,
    gestures_mask: 0,
    event_mode: 0,
    touch_set_mult: 8,
    touch_clear_mult: 5,
    sensitivity: 3,
    alp_threshold: 8,
    active_rate_ms: 10,
  },

  controls: [
    { type: 'toggle', field: 'touching', label: 'Finger on surface' },
    {
      type: 'slider',
      field: 'finger_x',
      label: 'Finger X (3 Tx columns)',
      min: 0,
      max: X_RES - 1,
      step: 1,
    },
    {
      type: 'slider',
      field: 'finger_y',
      label: 'Finger Y (2 Rx rows)',
      min: 0,
      max: Y_RES - 1,
      step: 1,
    },
    { type: 'toggle', field: 'alp_approach', label: 'Approach (ALP wake)' },
    {
      type: 'slider',
      field: 'gesture_code',
      label: 'Gesture (0 none, 1 tap, 2 hold, 3..6 swipes)',
      min: 0,
      max: 6,
      step: 1,
    },
    {
      type: 'slider',
      field: 'mode',
      label: 'Mode (0 Act/1 IdleT/2 Idle/3 LP1/4 LP2)',
      min: 0,
      max: 4,
      step: 1,
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_sense_cap_find: () => ({ scalar: 1 }), // 1 if a device ACKs at 0x56
    tile_sense_cap_init: () => ({
      scalar: 0,
      nextState: { mode: MODE_ACTIVE, configured: 0 },
    }),
    tile_sense_cap_reset: () => ({ nextState: { configured: 0, mode: MODE_ACTIVE } }),
    tile_sense_cap_ack_reset: () => ({ nextState: {} }),
    tile_sense_cap_sleep: () => ({ nextState: { mode: MODE_LP2 } }),
    tile_sense_cap_wake: () => ({ nextState: { mode: MODE_ACTIVE } }),

    // ── surface configuration ──
    tile_sense_cap_configure_surface: () => ({
      scalar: 1,
      nextState: { configured: 1 },
    }),
    tile_sense_cap_setup_2x3: () => ({
      scalar: 1,
      nextState: {
        configured: 1,
        x_res: X_RES,
        y_res: Y_RES,
        touch_set_mult: 8,
        touch_clear_mult: 5,
      },
    }),

    // ── event processing ──
    tile_sense_cap_process: () => ({ nextState: {} }),
    tile_sense_cap_on_event: () => ({ nextState: {} }),

    // ── identification ──
    tile_sense_cap_get_version_major: () => ({ scalar: 1 }),
    tile_sense_cap_get_version_minor: () => ({ scalar: 0 }),
    tile_sense_cap_get_settings_version: () => ({ scalar: 0 }),

    // ── trackpad data ──
    tile_sense_cap_get_info_flags: ({ state }) => ({ scalar: infoFlags(state) }),
    tile_sense_cap_get_gestures: ({ state }) => ({ scalar: gestureWord(state) }),
    tile_sense_cap_get_num_fingers: ({ state }) => ({ scalar: numFingers(state) }),
    tile_sense_cap_get_finger_x: ({ state, args }) => ({
      scalar: pick(args, 0, 0) === 0 && numFingers(state) ? state.finger_x : 0,
    }),
    tile_sense_cap_get_finger_y: ({ state, args }) => ({
      scalar: pick(args, 0, 0) === 0 && numFingers(state) ? state.finger_y : 0,
    }),
    tile_sense_cap_get_finger_strength: ({ state, args }) => ({
      scalar: pick(args, 0, 0) === 0 && numFingers(state) ? TOUCH_DELTA + 2 * NEIGHBOR_DELTA : 0,
    }),
    tile_sense_cap_get_finger_area: ({ state, args }) => ({
      scalar: pick(args, 0, 0) === 0 && numFingers(state) ? 2 : 0,
    }),

    // ── channels ──
    tile_sense_cap_get_touch_status: ({ state }) => ({ scalar: touchStatus(state) }),
    tile_sense_cap_is_touched: ({ state, args }) => ({
      scalar: touchBit(state, pick(args, 0, 0)),
    }),
    tile_sense_cap_get_num_channels: ({ state }) => ({
      scalar: state.configured ? NUM_CH : 0,
    }),
    tile_sense_cap_get_channel_count: ({ state, args }) => ({
      scalar: countOf(state, pick(args, 0, 0)),
    }),
    tile_sense_cap_get_channel_delta: ({ state, args }) => ({
      scalar: deltaOf(state, pick(args, 0, 0)),
    }),

    // ── touch events / zones (v0.3 high level) ──
    tile_sense_cap_on_touch: () => ({ nextState: {} }),
    tile_sense_cap_on_tap: () => ({ nextState: {} }),
    tile_sense_cap_on_long_press: () => ({ nextState: {} }),
    tile_sense_cap_on_swipe: () => ({ nextState: {} }),
    tile_sense_cap_on_drag: () => ({ nextState: {} }),
    tile_sense_cap_on_pinch: () => ({ nextState: {} }),
    tile_sense_cap_next_touch_event: () => ({ scalar: 0 }), // queue not modeled
    tile_sense_cap_get_touch_events: ({ state }) => {
      // approximate: touching -> DOWN bit, gesture control -> its event bit
      let ev = 0;
      if (state.touching) ev |= 1 << 0;
      if (state.gesture_code === 1) ev |= 1 << 2; // tap
      if (state.gesture_code === 2) ev |= 1 << 4; // hold
      // gesture codes 3..6 = -X, +X, +Y, -Y; events: LEFT(-X) 5, RIGHT(+X) 6,
      // UP(-Y) 7, DOWN(+Y) 8
      const swipeBit: Record<number, number> = { 3: 5, 4: 6, 5: 8, 6: 7 };
      if (swipeBit[state.gesture_code] !== undefined) ev |= 1 << swipeBit[state.gesture_code];
      return { scalar: ev };
    },
    tile_sense_cap_was_tapped: ({ state }) => ({
      scalar: state.gesture_code === 1 ? 1 : 0,
    }),
    tile_sense_cap_get_zone: ({ state }) => ({
      scalar: state.configured && state.touching ? fingerChannel(state) : -1,
    }),
    tile_sense_cap_zone_at: ({ state, args }) => {
      if (!state.configured) return { scalar: -1 };
      const zx = pick(args, 0, 0);
      const zy = pick(args, 1, 0);
      const col = Math.min(COLS - 1, Math.max(0, Math.floor((zx / state.x_res) * COLS)));
      const row = Math.min(ROWS - 1, Math.max(0, Math.floor((zy / state.y_res) * ROWS)));
      return { scalar: col * ROWS + row };
    },
    tile_sense_cap_is_zone_touched: ({ state, args }) => ({
      scalar: touchBit(state, pick(args, 0, 0)),
    }),
    // void get_position_pct(tile, int32_t *x_pct, int32_t *y_pct) — out-params
    tile_sense_cap_get_position_pct: ({ state }) => ({
      outScalars:
        state.configured && state.touching
          ? {
              x_pct: Math.round((state.finger_x * 100) / (state.x_res - 1)),
              y_pct: Math.round((state.finger_y * 100) / (state.y_res - 1)),
            }
          : { x_pct: -1, y_pct: -1 },
    }),
    tile_sense_cap_wait_for_touch: ({ state }) => ({
      scalar: state.configured && state.touching ? 1 : 0,
    }),
    tile_sense_cap_set_sensitivity: ({ args }) => {
      const level = clamp(pick(args, 0, 3), 1, 5);
      const [set, clr] = SENSITIVITY_PAIRS[level - 1];
      return {
        nextState: { sensitivity: level, touch_set_mult: set, touch_clear_mult: clr },
      };
    },

    // ── ALP ──
    tile_sense_cap_is_alp_active: ({ state }) => ({
      scalar: state.alp_approach || state.touching ? 1 : 0,
    }),
    tile_sense_cap_get_alp_count: ({ state }) => ({
      scalar: ALP_LTA + (state.alp_approach || state.touching ? ALP_DELTA : 0),
    }),
    tile_sense_cap_get_alp_lta: () => ({ scalar: ALP_LTA }),

    // ── status ──
    tile_sense_cap_get_mode: ({ state }) => ({ scalar: state.mode }),
    tile_sense_cap_has_ati_error: () => ({ scalar: 0 }),

    // ── configuration ──
    tile_sense_cap_enable_gestures: ({ args }) => ({
      nextState: { gestures_mask: pick(args, 0, 0) & 0x3f },
    }),
    tile_sense_cap_set_tap_timing: () => ({ nextState: {} }),
    tile_sense_cap_set_swipe_timing: () => ({ nextState: {} }),
    tile_sense_cap_set_report_rate: ({ state, args }) => ({
      nextState: {
        active_rate_ms:
          pick(args, 0, MODE_ACTIVE) === MODE_ACTIVE
            ? clamp(pick(args, 1, 10), 0, 65535)
            : state.active_rate_ms,
      },
    }),
    tile_sense_cap_set_mode_timeout: () => ({ nextState: {} }),
    tile_sense_cap_set_max_touches: ({ args }) => ({
      nextState: { max_touches: clamp(pick(args, 0, 2), 1, 2) },
    }),
    tile_sense_cap_set_resolution: ({ state, args }) => ({
      nextState: {
        x_res: clamp(pick(args, 0, state.x_res), 1, 65535),
        y_res: clamp(pick(args, 1, state.y_res), 1, 65535),
      },
    }),
    tile_sense_cap_set_touch_multipliers: ({ state, args }) => ({
      nextState: {
        touch_set_mult: clamp(pick(args, 0, state.touch_set_mult), 0, 255),
        touch_clear_mult: clamp(pick(args, 1, state.touch_clear_mult), 0, 255),
      },
    }),
    tile_sense_cap_set_alp_threshold: ({ state, args }) => ({
      nextState: { alp_threshold: clamp(pick(args, 0, state.alp_threshold), 0, 65535) },
    }),
    tile_sense_cap_set_event_mode: ({ args }) => ({
      nextState: { event_mode: pick(args, 0, 0) ? 1 : 0 },
    }),
    tile_sense_cap_set_watchdog: () => ({ nextState: {} }),

    // ── ATI ──
    tile_sense_cap_re_ati: () => ({ scalar: 1 }),
    tile_sense_cap_reseed: () => ({ nextState: {} }),

    // ── escape hatch ──
    tile_sense_cap_read_reg: () => ({ scalar: 0 }),
    tile_sense_cap_write_reg: () => ({ nextState: {} }),
  },

  provenance: {
    // canonical — datasheet-verified values and layouts
    tile_sense_cap_find: 'canonical', // I2C 0x56, fixed in silicon
    tile_sense_cap_get_info_flags: 'canonical', // mode [2:0], fingers [9:8]
    tile_sense_cap_get_mode: 'canonical',
    tile_sense_cap_get_num_fingers: 'canonical',
    tile_sense_cap_get_touch_status: 'canonical', // one bit per channel, §5.1.1 numbering
    tile_sense_cap_is_touched: 'canonical', // threshold = ref*(1+mult/128), §5.5.1
    tile_sense_cap_set_touch_multipliers: 'canonical', // 0x53 set/clear bytes
    tile_sense_cap_set_resolution: 'canonical', // 0x62/0x63
    tile_sense_cap_set_alp_threshold: 'canonical', // 0x54 delta threshold
    tile_sense_cap_setup_2x3: 'canonical', // r0 geometry: RX3/RX6 x TX11/TX9/TX8
    tile_sense_cap_get_num_channels: 'canonical',
    // inferred — behavior follows the driver, magnitudes modeled
    tile_sense_cap_get_channel_count: 'inferred', // counts settle at ATI target
    tile_sense_cap_get_channel_delta: 'inferred', // touch delta magnitude modeled
    tile_sense_cap_get_finger_x: 'inferred',
    tile_sense_cap_get_finger_y: 'inferred',
    tile_sense_cap_get_finger_strength: 'inferred',
    tile_sense_cap_get_gestures: 'inferred', // control-driven, mask per driver bits
    tile_sense_cap_is_alp_active: 'inferred',
    tile_sense_cap_get_alp_count: 'inferred', // combined ~ sum of block targets
    tile_sense_cap_get_alp_lta: 'inferred',
    tile_sense_cap_re_ati: 'inferred',
    tile_sense_cap_get_version_major: 'inferred', // app version varies by chip firmware
    tile_sense_cap_get_version_minor: 'inferred',
    tile_sense_cap_get_zone: 'canonical', // §5.1.1 channel math over finger XY
    tile_sense_cap_zone_at: 'canonical',
    tile_sense_cap_is_zone_touched: 'canonical',
    tile_sense_cap_get_position_pct: 'canonical',
    tile_sense_cap_wait_for_touch: 'inferred',
    tile_sense_cap_get_touch_events: 'inferred', // event synthesis approximated
    tile_sense_cap_was_tapped: 'inferred',
    tile_sense_cap_next_touch_event: 'hallucinated', // queue not modeled
    // hallucinated — stubs with no modeled behavior
    tile_sense_cap_get_finger_area: 'hallucinated',
    tile_sense_cap_get_settings_version: 'hallucinated',
    tile_sense_cap_read_reg: 'hallucinated',
    tile_sense_cap_write_reg: 'hallucinated',
    power: 'canonical', // datasheet §2.3 bench table (5x6 pad; 2x3 runs lower)
  },

  // RDY (pad 3): open-drain, asserted low each communication window — in
  // streaming mode that's every report cycle while awake. Shown 1 = asserted.
  // The Rx/Tx electrode pads are analog sense lines, not driven outputs.
  padOutputs(state) {
    const streaming = state.mode <= MODE_IDLE && state.configured === 1;
    const lpEvent = state.mode >= MODE_LP1 && (state.alp_approach || state.touching);
    return { '3': streaming || lpEvent ? 1 : 0 };
  },

  // Electrical: a pure load on V+ (pad 10) / GND (pad 1); no rail sourced.
  // Bench figures from the datasheet's 5x6 reference pad — the 2x3 surface
  // draws proportionally less in the sensing modes.
  power(state) {
    const ua = modeCurrentUa(state.mode);
    return {
      draw_ua: ua,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: 3300,
          i_ua: ua,
          pads: ['10'],
          note: `IQS7211A (${['Active', 'Idle-Touch', 'Idle', 'LP1', 'LP2'][state.mode] ?? 'Active'})`,
        },
      ],
    };
  },
};

export default sim;
