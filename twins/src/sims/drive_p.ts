// Digital twin for Drive.P — Boréas BOS1921 piezoelectric haptic driver.
//
// A boost converter fed from V_DRIVE (pad 9) swings a differential output
// OUT+/OUT- (pads 7/8) up to 190 Vpp (±95 V; ±13.28 V in the low range), and
// the same pins sense the piezo's voltage for touch. Logic on V+ (pad 10).
//
// One module, one vocabulary: hostCalls answer the firmware (tile_drive_p.c)
// and write `mode`, `amplitude`, `freq_hz`, `sleeping`, …; power / padOutputs
// read those same fields, so the drive current follows the program.
//
// Behavior notes from the driver + datasheet:
//   - set_mode() is READY-gated and there is no wake(): after sleep() only a
//     fault recovery (check_and_recover) re-readies the driver
//     (tile_drive_p.c:253-258, 354-360, 376-381).
//   - When the FIFO drains, the last sample stays on the output (BOS1921
//     §6.x "the last FIFO entry will remain on the device output"), so a click
//     leaves a small DC hold; with COMM.TOUT set the chip sleeps instead.
//   - read() returns the COMM.RDADDR register: CHIP_ID after init, IC_STATUS
//     after idle/play set_mode, SENSE_VAL after sense set_mode.
import type { TileSim } from '../tileSim';

// drive_p_mode_t (tile_drive_p.h:189-194)
const MODE_IDLE = 0;
const MODE_SENSE_FINE = 1;
const MODE_SENSE_COARSE = 2;
const MODE_PLAY_DIRECT = 3;
const MODE_PLAY_FIFO = 4;
const MODE_PLAY_RAM_SYNTH = 5;

// COMM.RDADDR targets (tile_drive_p.h:139-144)
const REG_IC_STATUS = 0x10;
const REG_SENSE_VAL = 0x18;
const REG_CHIP_ID = 0x1e;
const CHIP_ID = 0x0781;

// IC_STATUS (datasheet Table 44)
const STATE_IDLE = 0x0000;
const STATE_RUNNING = 0x0200;
const STATE_ERROR = 0x0300;
const UVLO_BIT = 0x0008;
const SC_BIT = 0x0004;
const PLAYST_BIT = 0x0001; // FIFO empty

const SAMPLE_FS = 2047; // 12-bit signed full-scale code
// REFERENCE bound for the rated output (BOS1921 §6.10.1): ±1743 = ±95 V /
// ±13.28 V. The driver scales 100 % intensity to it and clamps buffers to it.
const REF_MAX = 1743;
const UVLO_MV = 3000; // V_DRIVE minimum (Drive-P-a.json power[] min 3.0 V)
const PRESS_MV = 3000; // what the "press" toggle applies to the sense pins

// Power model — V_DRIVE draw, anchored on BOS1921 Table 7 (IBUS,AVG, Vbus 3.6 V).
// A piezo is a capacitor: reactive C·V²·f plus a static cost to hold the HV rail.
//     I = IDLE + K_STATIC·Vpk + K_DYN·C·Vpp²·f / Vbus
// Anchors: sleep 0.6 µA / 2.4 µA retained · idle 530 µA · DC 95 V 3.7 mA ·
// 190 Vpp/300 Hz/100 nF 90 mA. Check point 190 Vpp/200 Hz/10 nF predicts
// ~9.5 mA vs 14.5 mA spec, so the layer stays "inferred".
const IQ_SLEEP_RET_UA = 2.4;
const IQ_SLEEP_NORET_UA = 0.6;
const IDLE_UA = 530;
const VPLUS_ACTIVE_UA = 50; // V+ logic/IO (small; datasheet lumps Iq under VBUS)
const K_STATIC_UA_PER_V = 33.4;
const K_DYN = 0.287;

interface State {
  // ── physical inputs (controls) ──
  sense_mv: number; // voltage the piezo presents to the sense channel
  touch_detected: number; // a firm press: drives the sense pins to PRESS_MV
  fault_inject: number; // an output short (IC_STATUS SC → STATE ERROR)
  supply_mv: number; // V_DRIVE
  load_nf: number; // external piezo capacitance

  // ── driver / chip state ──
  mode: number; // drive_p_mode_t
  sleeping: number; // tile->state == SLEEPING
  auto_slept: number; // chip asleep via COMM.TOUT after the FIFO drained
  return_reg: number; // COMM.RDADDR
  output_range: number; // 0 = ±95 V, 1 = ±13.28 V (GAIND)
  sense_gain: number; // 1 = fine 7.6 mV, 0 = coarse 54.5 mV (GAINS)
  sleep_retention: number; // the driver's `retain` arg: 1 = retain (RET=0)
  auto_sleep: number; // COMM.TOUT
  upi: number; // PARCAP.UPI

  // ── output ──
  amplitude: number; // |sample| on the output, 0..2047
  freq_hz: number; // tone frequency of the current waveform (0 = DC)
  hold_amplitude: number; // |last sample| left on the output when playback ends
  currently_playing: string;
  play_ms: number; // requested duration, stamped by deriveState
  playing_until_ms: number; // deadline on deriveState's clock; 0 = none
  last_fifo_sample: number;
  last_samples_count: number;
  last_wfs_count: number;
  read_counter: number;
}

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const isPlayMode = (m: number) =>
  m === MODE_PLAY_DIRECT || m === MODE_PLAY_FIFO || m === MODE_PLAY_RAM_SYNTH;
const powered = (s: State) => !s.sleeping && !s.auto_slept && s.supply_mv >= UVLO_MV;

// Q12 sine as the driver's quarter-wave LUT (tile_drive_p.c:481-506).
function sineQ12(phase: number): number {
  const q = (phase >> 14) & 3;
  const idx = (phase >> 8) & 0x3f;
  const lut = (i: number) => Math.round(Math.sin((Math.PI / 2) * (i / 64)) * 2047);
  if (q === 0) return lut(idx);
  if (q === 1) return lut(63 - idx);
  if (q === 2) return -lut(idx);
  return -lut(63 - idx);
}
// scale_intensity(): 100 % of a ±2046 sine peaks at REF_MAX.
const scale = (sample: number, pct: number) =>
  Math.trunc((sample * Math.min(100, pct) * REF_MAX) / (100 * SAMPLE_FS));
const pctArg = (v: number | undefined) => Math.min(100, (v ?? 0) & 0xff);

// Drive envelope magnitude 0..1 on OUT±.
function driveStrength(s: State): number {
  if (!powered(s) || !isPlayMode(s.mode) || s.fault_inject) return 0;
  return clamp(s.amplitude / SAMPLE_FS, 0, 1);
}

// Vpk at code 2047: 3.6 V × FBratio (31 high, 4.33 low), §6.10.1.
const outFsMv = (s: State) => (s.output_range === 1 ? 15_588 : 111_600);

// Applied piezo mV → signed 12-bit SENSE_VAL at the active LSB.
function senseRaw(s: State): number {
  const mv = s.touch_detected ? Math.max(Math.abs(s.sense_mv), PRESS_MV) : s.sense_mv;
  const lsbTenths = s.sense_gain === 1 ? 76 : 545;
  return clamp(Math.round((mv * 10) / lsbTenths), -2048, 2047);
}

function statusWord(s: State): number {
  if (s.supply_mv < UVLO_MV) return STATE_ERROR | UVLO_BIT;
  if (s.fault_inject) return STATE_ERROR | SC_BIT;
  const running = powered(s) && isPlayMode(s.mode);
  const fifoEmpty = s.playing_until_ms === 0 && s.play_ms === 0;
  return (running ? STATE_RUNNING : STATE_IDLE) | (fifoEmpty ? PLAYST_BIT : 0);
}

function readReturn(s: State): number {
  if (s.return_reg === REG_SENSE_VAL) return senseRaw(s) & 0x0fff;
  if (s.return_reg === REG_CHIP_ID) return CHIP_ID;
  return statusWord(s);
}

// set_mode() as the driver does it (tile_drive_p.c:253-308). READY-gated.
function setMode(s: State, mode: number): Partial<State> {
  if (s.sleeping) return {};
  const m = mode >= MODE_IDLE && mode <= MODE_PLAY_RAM_SYNTH ? mode : MODE_IDLE;
  const sense = m === MODE_SENSE_FINE || m === MODE_SENSE_COARSE;
  return {
    mode: m,
    auto_slept: 0,
    return_reg: sense ? REG_SENSE_VAL : REG_IC_STATUS,
    ...(m === MODE_SENSE_FINE ? { sense_gain: 1 } : {}),
    ...(m === MODE_SENSE_COARSE ? { sense_gain: 0 } : {}),
    ...(isPlayMode(m)
      ? {}
      : { amplitude: 0, currently_playing: '', play_ms: 0, playing_until_ms: 0 }),
  };
}

// Start a timed FIFO waveform (the helpers all set PLAY_FIFO first).
function play(
  s: State,
  label: string,
  peak: number,
  freq: number,
  ms: number,
  hold: number,
): Partial<State> {
  if (s.sleeping) return {};
  return {
    ...setMode(s, MODE_PLAY_FIFO),
    amplitude: clamp(Math.abs(peak), 0, SAMPLE_FS),
    freq_hz: freq,
    hold_amplitude: clamp(Math.abs(hold), 0, SAMPLE_FS),
    currently_playing: label,
    play_ms: Math.max(1, ms),
    playing_until_ms: 0,
  };
}

// Half-sine click, 16 samples ≈ 2 ms (tile_drive_p.c:525-535): peak at i=8,
// last sample (i=15) stays on the output. A lone click is one charge of the
// piezo, not a tone, so its `freq_hz` is 0 (HV-hold only); a pulse train counts
// its repetition rate.
const clickPeak = (pct: number) => scale(sineQ12(8 * 2048), pct);
const clickHold = (pct: number) => scale(sineQ12(15 * 2048), pct);

const sim: TileSim<State> = {
  tile: 'Drive.P',

  // After tile_drive_p_init(): soft reset (RDADDR = CHIP_ID, IDLE), GAINS=1,
  // GAIND=0, RET=0 (retain), TOUT=0, UPI=0 (tile_drive_p.c:148-183).
  defaultState: {
    sense_mv: 0,
    touch_detected: 0,
    fault_inject: 0,
    supply_mv: 3700,
    load_nf: 260, // init tunes PARCAP for a 260 nF piezo (tile_drive_p.c:174)

    mode: MODE_IDLE,
    sleeping: 0,
    auto_slept: 0,
    return_reg: REG_CHIP_ID,
    output_range: 0,
    sense_gain: 1,
    sleep_retention: 1,
    auto_sleep: 0,
    upi: 0,

    amplitude: 0,
    freq_hz: 0,
    hold_amplitude: 0,
    currently_playing: '',
    play_ms: 0,
    playing_until_ms: 0,
    last_fifo_sample: 0,
    last_samples_count: 0,
    last_wfs_count: 0,
    read_counter: 0,
  },

  controls: [
    {
      type: 'slider',
      field: 'sense_mv',
      label: 'Sense voltage',
      min: -3300,
      max: 3300,
      step: 10,
      unit: 'mV',
      description:
        'Voltage the piezo presents to the sense channel. read_sense() returns it as a signed 12-bit value; is_touched() compares |mV| to its threshold.',
    },
    {
      type: 'toggle',
      field: 'touch_detected',
      label: 'Press',
      description: 'A firm press on the piezo: drives the sense pins to at least 3 V.',
    },
    {
      type: 'toggle',
      field: 'fault_inject',
      label: 'Fault: output short',
      description:
        'IC_STATUS reports SC with STATE = ERROR and the output stops; check_and_recover() runs a reset + re-enters the mode.',
    },
    {
      type: 'slider',
      field: 'supply_mv',
      label: 'V_DRIVE supply',
      min: 2500,
      max: 5500,
      step: 50,
      unit: 'mV',
      description: 'Boost supply. Below 3.0 V the chip reports UVLO and does not drive.',
    },
    {
      type: 'slider',
      field: 'load_nf',
      label: 'Piezo load',
      min: 1,
      max: 470,
      step: 1,
      unit: 'nF',
      description: 'Capacitance of the external piezo; the drive current scales with it.',
    },
  ],

  // Pressing the piezo: it generates a voltage the sense channel reads.
  stimuli: [
    {
      id: 'touch',
      label: 'Touch',
      controls: [
        { kind: 'toggle', id: 'press', label: 'Press', fields: ['touch_detected'] },
        {
          kind: 'slider',
          id: 'sense_mv',
          label: 'piezo voltage',
          field: 'sense_mv',
          min: -3300,
          max: 3300,
          step: 10,
          unit: 'mV',
        },
      ],
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_drive_p_find: () => ({ scalar: 1 }),
    tile_drive_p_init: () => ({
      nextState: {
        mode: MODE_IDLE,
        sleeping: 0,
        auto_slept: 0,
        return_reg: REG_CHIP_ID,
        output_range: 0,
        sense_gain: 1,
        sleep_retention: 1,
        auto_sleep: 0,
        upi: 0,
        amplitude: 0,
        currently_playing: '',
        play_ms: 0,
        playing_until_ms: 0,
      },
    }),
    // CONFIG DS=1 (tile_drive_p.c:354-360). Not gated.
    tile_drive_p_sleep: () => ({
      nextState: {
        sleeping: 1,
        mode: MODE_IDLE,
        amplitude: 0,
        currently_playing: '',
        play_ms: 0,
        playing_until_ms: 0,
      },
    }),
    // Error STATE or any fault bit → reset + READY + set_mode(restore)
    // (tile_drive_p.c:362-384). The reset restores the shadow defaults.
    tile_drive_p_check_and_recover: ({ state, args }) => {
      const status = statusWord(state);
      const needs = (status & 0x0300) === STATE_ERROR || (status & 0x00fc) !== 0;
      if (!needs) return { scalar: 0 };
      const reset: State = {
        ...state,
        sleeping: 0,
        output_range: 0,
        sense_gain: 1,
        sleep_retention: 1,
        auto_sleep: 0,
        upi: 0,
      };
      return {
        scalar: 1,
        nextState: {
          sleeping: 0,
          output_range: 0,
          sense_gain: 1,
          sleep_retention: 1,
          auto_sleep: 0,
          upi: 0,
          amplitude: 0,
          ...setMode(reset, (args[0] ?? 0) & 0xff),
        },
      };
    },

    // ── mode + raw access ──
    tile_drive_p_set_mode: ({ state, args }) => ({
      nextState: setMode(state, (args[0] ?? 0) & 0xff),
    }),
    tile_drive_p_read: ({ state }) => ({
      scalar: readReturn(state),
      nextState: { read_counter: state.read_counter + 1 },
    }),
    tile_drive_p_read_sense: ({ state }) => ({
      scalar: senseRaw(state),
      nextState: { return_reg: REG_SENSE_VAL, read_counter: state.read_counter + 1 },
    }),
    tile_drive_p_read_status: ({ state }) => ({
      scalar: statusWord(state),
      nextState: { return_reg: REG_IC_STATUS },
    }),
    // REFERENCE write: in a play mode the sample goes out (and stays there).
    tile_drive_p_write_fifo: ({ args }) => {
      const raw = (args[0] ?? 0) & 0xffff;
      const sample = raw > 0x7fff ? raw - 0x10000 : raw; // int16
      return {
        nextState: {
          last_fifo_sample: sample,
          amplitude: clamp(Math.abs(sample), 0, SAMPLE_FS),
          auto_slept: 0,
        },
      };
    },
    // Up to 8 WFS words to REFERENCE (tile_drive_p.c:342-352); RAM synth not modeled.
    tile_drive_p_wfs_write: ({ bufferIn }) => ({
      nextState: { last_wfs_count: Math.min(8, (bufferIn?.words ?? []).length) },
    }),

    // ── config setters ──
    // GAIND / GAINS land with an IDLE CONFIG write when READY, which clears OE
    // (tile_drive_p.c:390-419) — a play mode stops.
    tile_drive_p_set_output_range: ({ state, args }) => ({
      nextState: {
        output_range: args[0] === 1 ? 1 : 0,
        ...(state.sleeping ? {} : { ...setMode(state, MODE_IDLE), return_reg: state.return_reg }),
      },
    }),
    tile_drive_p_set_sense_gain: ({ state, args }) => ({
      nextState: {
        sense_gain: args[0] === 1 ? 1 : 0,
        ...(state.sleeping ? {} : { ...setMode(state, MODE_IDLE), return_reg: state.return_reg }),
      },
    }),
    tile_drive_p_set_sleep_retention: ({ args }) => ({
      nextState: { sleep_retention: args[0] ? 1 : 0 },
    }),
    // COMM rewrite points RDADDR at IC_STATUS (tile_drive_p.c:438-456).
    tile_drive_p_set_auto_sleep: ({ args }) => ({
      nextState: { auto_sleep: args[0] ? 1 : 0, return_reg: REG_IC_STATUS },
    }),
    tile_drive_p_set_upi: ({ args }) => ({ nextState: { upi: args[0] ? 1 : 0 } }),

    // ── playback helpers ──
    tile_drive_p_play_click: ({ state, args }) => {
      const pct = pctArg(args[0]);
      return {
        nextState: play(state, `click @ ${pct}%`, clickPeak(pct), 0, 2, clickHold(pct)),
      };
    },
    // FIFO sine at 8 ksps for `ms` (tile_drive_p.c:537-566); no-op on 0 Hz / 0 ms.
    tile_drive_p_play_sine: ({ state, args }) => {
      const freq = (args[0] ?? 0) & 0xffff;
      const pct = pctArg(args[1]);
      const ms = (args[2] ?? 0) & 0xffff;
      if (freq === 0 || ms === 0) return {};
      const step = Math.floor((freq * 65536) / 8000);
      const last = scale(sineQ12(((ms * 8 - 1) * step) & 0xffff), pct);
      return {
        nextState: play(
          state,
          `sine ${freq} Hz @ ${pct}% / ${ms} ms`,
          scale(2046, pct),
          freq,
          ms,
          last,
        ),
      };
    },
    // play_sine at 150 Hz (tile_drive_p.c:568-572).
    tile_drive_p_play_buzz: ({ state, args }) => {
      const pct = pctArg(args[0]);
      const ms = (args[1] ?? 0) & 0xffff;
      if (ms === 0) return {};
      const step = Math.floor((150 * 65536) / 8000);
      const last = scale(sineQ12(((ms * 8 - 1) * step) & 0xffff), pct);
      return {
        nextState: play(state, `buzz @ ${pct}% / ${ms} ms`, scale(2046, pct), 150, ms, last),
      };
    },
    tile_drive_p_play_pulse_train: ({ state, args }) => {
      const pct = pctArg(args[0]);
      const count = (args[1] ?? 0) & 0xff;
      const gap = (args[2] ?? 0) & 0xffff;
      if (count === 0) return {};
      return {
        nextState: play(
          state,
          `pulse train ${count}× @ ${pct}%, ${gap} ms gap`,
          clickPeak(pct),
          count > 1 ? Math.round(1000 / (2 + gap)) : 0,
          count * 2 + (count - 1) * gap,
          clickHold(pct),
        ),
      };
    },
    // set_mode(SENSE_FINE) + read_sense, |raw·7.6 mV| ≥ threshold
    // (tile_drive_p.c:585-596).
    tile_drive_p_is_touched: ({ state, args }) => {
      const threshold = (args[0] ?? 0) & 0xffff;
      const next = { ...state, ...setMode(state, MODE_SENSE_FINE) };
      const mv = Math.abs(Math.trunc((senseRaw(next) * 76) / 10));
      return {
        scalar: mv >= threshold ? 1 : 0,
        nextState: {
          ...setMode(state, MODE_SENSE_FINE),
          return_reg: REG_SENSE_VAL,
          read_counter: state.read_counter + 1,
        },
      };
    },
    // Polls is_touched every 1 ms up to timeout_ms, then clicks
    // (tile_drive_p.c:598-612). Collapsed to one look at the present state.
    tile_drive_p_play_on_touch: ({ state, args }) => {
      const pct = pctArg(args[0]);
      const threshold = (args[1] ?? 0) & 0xffff;
      const timeout = args[2] ?? 0;
      if (timeout <= 0) return { scalar: 0 };
      const sensing = { ...state, ...setMode(state, MODE_SENSE_FINE) };
      const mv = Math.abs(Math.trunc((senseRaw(sensing) * 76) / 10));
      if (mv < threshold) {
        return {
          scalar: 0,
          nextState: { ...setMode(state, MODE_SENSE_FINE), return_reg: REG_SENSE_VAL },
        };
      }
      return {
        scalar: 1,
        nextState: play(
          sensing,
          `click @ ${pct}% (on touch)`,
          clickPeak(pct),
          0,
          2,
          clickHold(pct),
        ),
      };
    },
    // FIFO playback of the caller's samples at 8 ksps, each clamped to
    // ±REF_MAX; the last one holds on the output (tile_drive_p.c).
    tile_drive_p_play_samples: ({ state, bufferIn }) => {
      const samples = (bufferIn?.samples ?? []).map((v) => clamp(Math.trunc(v), -REF_MAX, REF_MAX));
      const count = samples.length;
      if (count === 0) return {};
      const peak = samples.reduce((m, v) => Math.max(m, Math.abs(v)), 0);
      return {
        nextState: {
          ...play(state, `samples ×${count}`, peak, 0, Math.ceil(count / 8), samples[count - 1]),
          last_samples_count: count,
        },
      };
    },
    // Fine sense mode, then `count` SENSE_VAL reads of the present piezo voltage.
    tile_drive_p_read_sense_samples: ({ state, caps }) => {
      const n = Math.max(0, caps?.buf ?? 0);
      const sensing = { ...state, ...setMode(state, MODE_SENSE_FINE) };
      return {
        out: { buf: new Array<number>(n).fill(senseRaw(sensing)) },
        nextState: {
          ...setMode(state, MODE_SENSE_FINE),
          return_reg: REG_SENSE_VAL,
          read_counter: state.read_counter + n,
        },
      };
    },
  },

  provenance: {
    tile_drive_p_find: 'canonical',
    tile_drive_p_read_sense: 'canonical', // signed 12-bit, 7.6 / 54.5 mV LSB
    tile_drive_p_read_status: 'canonical', // IC_STATUS STATE / fault / PLAYST
    tile_drive_p_read: 'canonical', // COMM.RDADDR return register
    tile_drive_p_set_mode: 'canonical', // CONFIG per mode, RDADDR, GAINS
    tile_drive_p_set_output_range: 'canonical', // GAIND, clears OE
    tile_drive_p_set_sense_gain: 'canonical', // GAINS, clears OE
    tile_drive_p_write_fifo: 'canonical', // REFERENCE; last sample holds
    tile_drive_p_is_touched: 'canonical', // 7.6 mV/LSB threshold compare
    tile_drive_p_play_click: 'canonical', // driver's 16-sample half-sine, 100 % = 1743
    tile_drive_p_play_sine: 'canonical', // driver's Q12 phase accumulator
    tile_drive_p_play_buzz: 'canonical',
    tile_drive_p_play_pulse_train: 'inferred',
    tile_drive_p_play_on_touch: 'inferred', // polling loop collapsed to one look
    tile_drive_p_check_and_recover: 'inferred',
    tile_drive_p_sleep: 'inferred',
    tile_drive_p_set_sleep_retention: 'inferred',
    tile_drive_p_set_auto_sleep: 'inferred',
    tile_drive_p_set_upi: 'inferred',
    tile_drive_p_wfs_write: 'hallucinated', // RAM synthesis not modeled
    tile_drive_p_play_samples: 'inferred', // envelope = peak |sample|, clamped ±1743
    tile_drive_p_read_sense_samples: 'inferred', // a constant press level, no waveform
    power: 'inferred', // CV²f + HV-hold fit to datasheet Table 7; loss factor inferred
  },

  // Playback end: the FIFO drains and the last sample holds on the output
  // (freq 0 = DC), or with COMM.TOUT the chip sleeps.
  deriveState: (state, { t }) => {
    if (state.play_ms > 0 && state.playing_until_ms === 0) {
      return { playing_until_ms: t + state.play_ms, play_ms: 0 };
    }
    if (state.playing_until_ms !== 0 && t >= state.playing_until_ms) {
      const tout =
        state.auto_sleep && (state.mode === MODE_PLAY_FIFO || state.mode === MODE_PLAY_DIRECT);
      return {
        playing_until_ms: 0,
        currently_playing: '',
        freq_hz: 0,
        amplitude: tout ? 0 : state.hold_amplitude,
        auto_slept: tout ? 1 : 0,
      };
    }
    return {};
  },

  padOutputs(state) {
    const s = driveStrength(state);
    return { '7': s, '8': s };
  },

  // V+ logic (pad 10) + V_DRIVE boost (pad 9); boosted differential drive on
  // OUT± (pads 7/8). V_DRIVE draw responds to amplitude, tone frequency, output
  // range, piezo load and supply voltage.
  power(state, ctx) {
    const supplyMv = ctx?.padVoltage['9'] ?? state.supply_mv;
    const vPlus = ctx?.padVoltage['10'] ?? 3300;
    const s = driveStrength(state);
    let driveUa: number;
    if (state.sleeping || state.auto_slept) {
      driveUa = state.sleep_retention ? IQ_SLEEP_RET_UA : IQ_SLEEP_NORET_UA;
    } else if (supplyMv < UVLO_MV) {
      driveUa = 0;
    } else if (s <= 0) {
      driveUa = IDLE_UA;
    } else {
      const vpk = (s * outFsMv(state)) / 1000;
      const vpp = 2 * vpk;
      const c = Math.max(1, state.load_nf) * 1e-9;
      const staticUa = K_STATIC_UA_PER_V * vpk;
      const dynUa = ((K_DYN * c * vpp * vpp * state.freq_hz) / (supplyMv / 1000)) * 1e6;
      driveUa = IDLE_UA + staticUa + dynUa;
    }
    const vplusUa = powered(state) ? VPLUS_ACTIVE_UA : 0;
    return {
      draw_ua: Math.round(driveUa + vplusUa),
      rails: [
        { name: 'V+', role: 'supply', v_mv: vPlus, i_ua: vplusUa, pads: ['10'], note: 'logic/IO' },
        {
          name: 'V_DRIVE',
          role: 'supply',
          v_mv: supplyMv,
          i_ua: Math.round(driveUa),
          pads: ['9'],
          note:
            state.sleeping || state.auto_slept
              ? 'boost supply — sleep'
              : s > 0
                ? 'boost supply + piezo drive'
                : 'boost supply — idle',
        },
        {
          name: 'OUT±',
          role: 'output',
          v_mv: Math.round(s * outFsMv(state)),
          pads: ['7', '8'],
          note: 'boosted differential piezo drive (±95 V at code 1743)',
        },
      ],
    };
  },
};

export default sim;
