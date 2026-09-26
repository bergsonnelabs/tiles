// Digital twin for Drive.A.2 — TI DAC63202W smart DAC + 2× TPA2028D1 Class-D amps.
//
// One V+ rail (pad 10, 2.5-5.5 V) feeds all three ICs:
//   - DAC63202W @ 0x49: two 12-bit channels with an on-chip function generator;
//     CH0 → amp0 → OUT.0± (pads 7/6), CH1 → amp1 → OUT.1± (pads 9/8).
//   - 2× TPA2028D1, both at 0x58: every amp write (gain, enable, mute, AGC)
//     lands on BOTH amps — there is one set of `amp_*` fields.
//
// One module, one vocabulary: hostCalls answer the firmware (tile_drive_a_2.c)
// and write `chN_code`, `chN_wave*`, `amp_gain_db`, `amp_enabled`, …; power and
// padOutputs read those same fields, so the speaker drive follows the program.
//
// Signal chain: the amp input sits at VDD/2, so DAC mid-scale (2048) is silence
// (tile_drive_a_2_init). The differential input is (code − 2048)·VREF/4096,
// amplified by the fixed gain, capped by the TPA2028D1 output limiter (on by
// default since driver 3.3.0) and clipped at the V+ rail. The tile ties VREF to
// VDD, so the ACTUAL full scale is V+. set_mv() uses the V+ the driver was told
// (`vdd_mv`, set_supply_mv(), default 3.3 V) and each channel's own gain; the
// output uses the real rail (`vplus_mv`), so a wrong set_supply_mv() shows.
import type { TileSim } from '../tileSim';

const DAC_MID = 2048;
const DAC_MAX = 4095;
const WAVE_SINE = 4;
const WAVE_OFF = 7; // DRIVE_A_2_WAVE_OFF
const GEN_ID_STATUS = 0x06 << 2; // GENERAL-STATUS DEVICE-ID[7:2] = 0x06
// The sine generator plays 24 fixed codes, 0x19A..0xE66 (SLASF73A Table 6-10):
// ±1638 codes about mid-scale, whatever the margins are.
const SINE_PEAK_CODES = 0xe66 - DAC_MID;

// Driver's full_scale_mv(): V+ (as told) in the 1× gains, 1.21 V × 1.5/2/3/4
// with the internal reference (SLASF73A Eq 1-3, Table 6-26).
const INT_FS_MV = [0, 0, 1815, 2420, 3630, 4840];
const driverFullScale = (gain: number, vddMv: number) =>
  gain >= 2 && gain <= 5 ? INT_FS_MV[gain] : vddMv;
const VDD_MIN_MV = 2500;
const VDD_MAX_MV = 5500;
const VDD_DEFAULT_MV = 3300;

// TPA2028D1 output limiter (SLOS660C Table 11): level n = -6.5 + 0.5·n dBV.
const LIMITER_DEFAULT = 19; // DRIVE_A_2_LIMITER_DEFAULT: 3 dBV, 2.0 V peak, 0.25 W / 8 Ω
const limiterPeakMv = (level: number) =>
  Math.pow(10, (-6.5 + 0.5 * level) / 20) * Math.SQRT2 * 1000;
const COMP_1_1 = 0;
// Lowest valid fixed gain: 0 dB at compression 1:1, -28 dB otherwise (Table 10).
const minGain = (s: State) => (s.agc_compression === COMP_1_1 ? 0 : -28);

// Supply currents on V+ (datasheets).
const I_DAC_CH_UA = 150; // DAC63202W IDD per powered channel (VOUT, ext ref at VDD)
const I_DAC_INTREF_CH_UA = 12.5; // + per channel when the internal reference is on
const I_DAC_SLEEP_UA = 28; // DAC sleep, ext ref (max)
const R_SPEAKER_OHM = 8; // TPA2028D1 characterization load (speakers are external)
const AMP_PMAX_5V_W = 1.4; // 1 % THD, 5 V, 8 Ω
const AMP_EFF = 0.91; // 3.6 V, 8 Ω

// TPA2028D1 per-amp quiescent vs V+ (typ at 2.5 / 3.6 / 5.5 V).
function ampIq(vMv: number, on: boolean): number {
  const [a, b, c] = on ? [1500, 1700, 2000] : [35, 50, 75];
  if (vMv <= 3600) return a + ((b - a) * (Math.max(2500, vMv) - 2500)) / 1100;
  return b + ((c - b) * (Math.min(5500, vMv) - 3600)) / 1900;
}

interface State {
  // ── physical inputs ──
  vplus_mv: number; // V+ (the DAC's VDD reference too)
  amp_fault: number; // FUNC_CTRL FAULT: output short (cleared by a 0 write)
  amp_thermal: number; // FUNC_CTRL Thermal: die > 150 °C

  // ── DAC ──
  ch0_code: number; // DAC-0-DATA, 12-bit
  ch1_code: number;
  ch0_gain: number; // drive_a_2_gain_t (VOUT-GAIN)
  ch1_gain: number;
  vdd_mv: number; // driver shadow: the V+ it was told (set_supply_mv / cfg->vdd_mv)
  ch0_wave: number; // FUNC-CONFIG (7 = off)
  ch1_wave: number;
  ch0_wave_running: number; // START-FUNC issued and not stopped
  ch1_wave_running: number;
  ch0_sw_wave: number; // play_chirp driving the DAC from software
  ch1_sw_wave: number;
  ch0_freq: number; // tone pitch the generator plays / chirp end frequency (Hz)
  ch1_freq: number;
  ch0_phase: number;
  ch1_phase: number;
  ch0_slew: number;
  ch1_slew: number;
  ch0_step: number;
  ch1_step: number;
  ch0_margin_low: number;
  ch0_margin_high: number;
  ch1_margin_low: number;
  ch1_margin_high: number;

  // ── amps (shared 0x58) ──
  amp_gain_db: number; // AGC fixed gain, -28..+30
  amp_enabled: number; // FUNC_CTRL SWS = 0
  amp_shutdown: number; // driver shadow: app asked for SWS (amp_disable / mute); wake() keeps it
  amp_muted: number; // driver shadow `is_muted`
  amp_muted_gain_db: number; // driver shadow `muted_gain_db`
  agc_compression: number;
  agc_max_gain_db: number;
  agc_limiter_level: number;
  agc_limiter_enabled: number; // AGC_CTRL1 bit 7 = 0
  agc_attack: number;
  agc_release: number;
  agc_hold: number;
  agc_noise_gate: number;

  sleeping: number;

  // ── blocking helpers (play_tone / play_chirp / play_silence) ──
  currently_playing: string;
  play_channels: number; // bit 0 = CH0, bit 1 = CH1
  play_remute: number; // re-mute when the helper finishes
  play_ms: number; // requested duration, stamped by deriveState
  playing_until_ms: number; // deadline on deriveState's clock; 0 = none
}

type Ch = 0 | 1;
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

// drive_a_2_channel_t → channels (LEFT 0, RIGHT 1, BOTH 2; others none).
const chans = (ch: number): Ch[] => (ch === 2 ? [0, 1] : ch === 0 || ch === 1 ? [ch] : []);
const maskOf = (cs: Ch[]) => cs.reduce<number>((m, c) => m | (1 << c), 0);

function patch(cs: Ch[], fields: Record<string, number>): Partial<State> {
  const out: Record<string, number> = {};
  for (const c of cs) for (const [k, v] of Object.entries(fields)) out[`ch${c}_${k}`] = v;
  return out as Partial<State>;
}

// Actual DAC full scale (mV): VREF pin is tied to VDD on the tile.
const realVref = (s: State, gain: number) =>
  gain <= 1 ? s.vplus_mv : Math.min(s.vplus_mv, (1210 * [0, 0, 1.5, 2, 3, 4][gain]) | 0);

// Peak differential input in DAC codes around mid-scale.
function inputCodes(s: State, c: Ch): number {
  const g = (k: string) => s[`ch${c}_${k}` as keyof State] as number;
  if (g('sw_wave')) return 2046;
  if (g('wave_running') && g('wave') === WAVE_SINE) return SINE_PEAK_CODES;
  if (g('wave_running') && g('wave') !== WAVE_OFF) {
    const lo = g('margin_low');
    const hi = g('margin_high');
    return Math.abs((lo + hi) / 2 - DAC_MID) + (hi - lo) / 2;
  }
  return Math.abs(g('code') - DAC_MID);
}

// Speaker drive level 0..1 (peak / V+) on a channel.
function outLevel(s: State, c: Ch): number {
  if (s.sleeping || !s.amp_enabled || s.amp_fault || s.amp_thermal) return 0;
  const vref = realVref(s, c === 0 ? s.ch0_gain : s.ch1_gain);
  const vinMv = (inputCodes(s, c) * vref) / 4096;
  let voutMv = vinMv * Math.pow(10, s.amp_gain_db / 20);
  // The limiter's gain reduction settles the peak at its level (attack ramp not modeled).
  if (s.agc_limiter_enabled) voutMv = Math.min(voutMv, limiterPeakMv(s.agc_limiter_level));
  return s.vplus_mv > 0 ? clamp(voutMv / s.vplus_mv, 0, 1) : 0;
}

// Driver amp_enable()/amp_disable(): the request is recorded in amp_shutdown;
// while asleep amp_enable() only records it and wake() applies it.
const enableState = (s: State): Partial<State> => ({
  amp_shutdown: 0,
  amp_enabled: s.sleeping ? 0 : 1,
});
const disableState: Partial<State> = { amp_shutdown: 1, amp_enabled: 0 };
// Driver mute()/unmute(): amp_disable()/amp_enable() plus the gain shadow.
const muteState = (s: State): Partial<State> => ({
  ...disableState,
  ...(s.amp_muted ? {} : { amp_muted: 1, amp_muted_gain_db: s.amp_gain_db }),
});
const unmuteState = (s: State): Partial<State> => ({
  ...enableState(s),
  ...(s.amp_muted
    ? { amp_muted: 0, amp_gain_db: clamp(s.amp_muted_gain_db, minGain(s), 30) }
    : {}),
});

// Sine pitch per SLEW-RATE code 1..15 in 0.1 Hz: f = 1 / (24 × time_step)
// (SLASF73A Eq 8, Table 6-30), the driver's SINE_HZ_X10.
const SINE_HZ_X10 = [
  104167, 52083, 34722, 23148, 15409, 10293, 6862, 4573, 3048, 1742, 995, 569, 325, 163, 81,
];

// pick_wave_params() (tile_drive_a_2.c): the slew code whose sine pitch is
// nearest `f` on a log scale; the code step is 1 LSB (unused by the sine).
// Returns [slew, step, generated pitch in Hz].
function waveParams(f: number): [number, number, number] {
  const fx10 = f * 10;
  let k = 14;
  for (let i = 0; i < 15; i++) {
    if (fx10 >= SINE_HZ_X10[i]) {
      k = i > 0 && fx10 * fx10 > SINE_HZ_X10[i] * SINE_HZ_X10[i - 1] ? i - 1 : i;
      break;
    }
  }
  return [k + 1, 0, SINE_HZ_X10[k] / 10];
}

// Channel-scoped setters: channel > 1 is ignored by the driver.
const onCh = (args: number[], fn: (c: Ch) => Partial<State>): { nextState?: Partial<State> } => {
  const ch = args[0] ?? 0;
  return ch === 0 || ch === 1 ? { nextState: fn(ch) } : {};
};

const sim: TileSim<State> = {
  tile: 'Drive.A.2',

  // After tile_drive_a_2_init(cfg = NULL): gain 1× EXT on both channels, V+
  // taken as 3.3 V, both DACs at mid-scale, amp gain 6 dB, AGC compression 1:1
  // / max 30 dB, limiter ON at level 19 (AGC_CTRL1 0x33), attack / release /
  // hold at chip defaults, amps on (FUNC_CTRL 0xC2).
  defaultState: {
    vplus_mv: 3300, // the driver's VDD assumption (resolve_vref)
    amp_fault: 0,
    amp_thermal: 0,

    ch0_code: DAC_MID,
    ch1_code: DAC_MID,
    ch0_gain: 0,
    ch1_gain: 0,
    vdd_mv: VDD_DEFAULT_MV,
    ch0_wave: WAVE_OFF,
    ch1_wave: WAVE_OFF,
    ch0_wave_running: 0,
    ch1_wave_running: 0,
    ch0_sw_wave: 0,
    ch1_sw_wave: 0,
    ch0_freq: 0,
    ch1_freq: 0,
    ch0_phase: 0,
    ch1_phase: 0,
    ch0_slew: 0,
    ch1_slew: 0,
    ch0_step: 0,
    ch1_step: 0,
    ch0_margin_low: 0,
    ch0_margin_high: DAC_MAX,
    ch1_margin_low: 0,
    ch1_margin_high: DAC_MAX,

    amp_gain_db: 6,
    amp_enabled: 1,
    amp_shutdown: 0,
    amp_muted: 0,
    amp_muted_gain_db: 6,
    agc_compression: 0, // AGC_CTRL2 0xC0
    agc_max_gain_db: 30,
    agc_limiter_level: LIMITER_DEFAULT, // AGC_CTRL1 0x33 (limiter on, NG 4 mV)
    agc_limiter_enabled: 1,
    agc_attack: 0x05, // chip defaults, rewritten by init
    agc_release: 0x0b,
    agc_hold: 0x00,
    agc_noise_gate: 1,

    sleeping: 0,

    currently_playing: '',
    play_channels: 0,
    play_remute: 0,
    play_ms: 0,
    playing_until_ms: 0,
  },

  controls: [
    {
      type: 'slider',
      field: 'vplus_mv',
      label: 'V+ supply',
      min: 2500,
      max: 5500,
      step: 50,
      unit: 'mV',
      description:
        "Tile supply. It is also the DAC's reference, so it sets the real full scale; set_mv() uses the V+ the program gave set_supply_mv() (3.3 V by default).",
    },
    {
      type: 'toggle',
      field: 'amp_fault',
      label: 'Fault: speaker short',
      description:
        'TPA2028D1 FAULT: output stops. Cleared when the driver rewrites FUNC_CTRL (init / wake).',
    },
    {
      type: 'toggle',
      field: 'amp_thermal',
      label: 'Fault: over-temp',
      description: 'TPA2028D1 thermal shutdown (die above 150 °C): output stops.',
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_drive_a_2_find: () => ({ scalar: 1 }),
    tile_drive_a_2_init: () => ({
      nextState: {
        ch0_code: DAC_MID,
        ch1_code: DAC_MID,
        ch0_gain: 0,
        ch1_gain: 0,
        vdd_mv: VDD_DEFAULT_MV, // cfg->vdd_mv is behind a pointer the host call can't read
        amp_gain_db: 6,
        amp_enabled: 1,
        amp_shutdown: 0,
        amp_muted: 0,
        amp_fault: 0,
        amp_thermal: 0,
        agc_compression: 0,
        agc_max_gain_db: 30,
        agc_limiter_level: LIMITER_DEFAULT,
        agc_limiter_enabled: 1,
        agc_attack: 0x05,
        agc_release: 0x0b,
        agc_hold: 0x00,
        agc_noise_gate: 1,
        sleeping: 0,
      },
    }),
    // VOUT Hi-Z + amp SWS; the app's amp_shutdown request is kept for wake().
    tile_drive_a_2_sleep: () => ({
      nextState: {
        sleeping: 1,
        amp_enabled: 0,
        ch0_wave_running: 0,
        ch1_wave_running: 0,
        ch0_sw_wave: 0,
        ch1_sw_wave: 0,
      },
    }),
    // DACs back at mid-scale, then FUNC_CTRL = 0xC2, or 0xE2 when the amps were
    // muted / disabled before sleep (they stay off); FAULT/Thermal clear.
    tile_drive_a_2_wake: ({ state }) => ({
      nextState: {
        sleeping: 0,
        ch0_code: DAC_MID,
        ch1_code: DAC_MID,
        amp_enabled: state.amp_shutdown ? 0 : 1,
        amp_fault: 0,
        amp_thermal: 0,
      },
    }),

    // ── DAC output ──
    tile_drive_a_2_set: ({ args }) =>
      onCh(args, (c) => patch([c], { code: clamp((args[1] ?? 0) & 0xffff, 0, DAC_MAX) })),
    // mv clamped to V+, then code = round(mv·4096 / full scale), the channel's own
    // gain (SLASF73A Eq 1-3).
    tile_drive_a_2_set_mv: ({ state, args }) =>
      onCh(args, (c) => {
        const mv = Math.min((args[1] ?? 0) & 0xffff, state.vdd_mv);
        const fs = driverFullScale(c === 0 ? state.ch0_gain : state.ch1_gain, state.vdd_mv);
        const code = fs ? Math.floor((mv * 4096 + Math.floor(fs / 2)) / fs) : 0;
        return patch([c], { code: Math.min(DAC_MAX, code) });
      }),
    tile_drive_a_2_get: ({ state, args }) => {
      const ch = args[0] ?? 0;
      return { scalar: ch === 0 ? state.ch0_code : ch === 1 ? state.ch1_code : 0 };
    },
    tile_drive_a_2_set_gain: ({ args }) =>
      onCh(args, (c) => patch([c], { gain: (args[1] ?? 0) & 0x07 })),
    // Driver shadow only, clamped to the tile's 2.5-5.5 V; no bus traffic.
    tile_drive_a_2_set_supply_mv: ({ args }) => ({
      nextState: { vdd_mv: clamp((args[0] ?? 0) & 0xffff, VDD_MIN_MV, VDD_MAX_MV) },
    }),

    // ── function generator ──
    tile_drive_a_2_set_waveform: ({ args }) =>
      onCh(args, (c) => patch([c], { wave: (args[1] ?? WAVE_OFF) & 0x07 })),
    tile_drive_a_2_start_waveform: ({ args }) => onCh(args, (c) => patch([c], { wave_running: 1 })),
    // START-FUNC-X cleared, FUNC-CONFIG = OFF (tile_drive_a_2_stop_waveform).
    tile_drive_a_2_stop_waveform: ({ args }) =>
      onCh(args, (c) => patch([c], { wave: WAVE_OFF, wave_running: 0 })),
    tile_drive_a_2_set_slew_rate: ({ args }) =>
      onCh(args, (c) => patch([c], { slew: (args[1] ?? 0) & 0x0f })),
    tile_drive_a_2_set_code_step: ({ args }) =>
      onCh(args, (c) => patch([c], { step: (args[1] ?? 0) & 0x07 })),
    // Clamped and swapped so high > low (tile_drive_a_2.c:440-455).
    tile_drive_a_2_set_margins: ({ args }) =>
      onCh(args, (c) => {
        const a = Math.min(DAC_MAX, (args[1] ?? 0) & 0xffff);
        const b = Math.min(DAC_MAX, (args[2] ?? DAC_MAX) & 0xffff);
        return patch([c], { margin_low: Math.min(a, b), margin_high: Math.max(a, b) });
      }),
    tile_drive_a_2_set_phase: ({ args }) =>
      onCh(args, (c) => patch([c], { phase: (args[1] ?? 0) & 0x03 })),
    // Margins to full scale + wave/step/slew (tile_drive_a_2.c:467-486).
    tile_drive_a_2_set_waveform_params: ({ args }) =>
      onCh(args, (c) =>
        patch([c], {
          margin_low: 0,
          margin_high: DAC_MAX,
          wave: (args[1] ?? WAVE_OFF) & 0x07,
          step: (args[2] ?? 0) & 0x07,
          slew: (args[3] ?? 0) & 0x0f,
        }),
      ),

    // ── amps (both, shared 0x58) ──
    // Clamped to the gain range valid for the compression ratio (0..30 at 1:1).
    tile_drive_a_2_amp_set_gain: ({ state, args }) => {
      const raw = (args[0] ?? 0) & 0xff;
      return {
        nextState: { amp_gain_db: clamp(raw > 127 ? raw - 256 : raw, minGain(state), 30) },
      };
    },
    tile_drive_a_2_amp_get_gain: ({ state }) => ({ scalar: state.amp_gain_db }),
    // EN = 1, SWS = 0 (deferred to wake() while asleep); the mute shadow is untouched.
    tile_drive_a_2_amp_enable: ({ state }) => ({ nextState: enableState(state) }),
    tile_drive_a_2_amp_disable: () => ({ nextState: disableState }),
    // The config is a struct pointer the host call can't read; no state change.
    tile_drive_a_2_amp_set_agc: () => ({}),
    // Level clamped to 0..31; a disable is honoured only at compression 1:1.
    tile_drive_a_2_amp_set_limiter: ({ state, args }) => ({
      nextState: {
        agc_limiter_level: clamp((args[1] ?? LIMITER_DEFAULT) & 0xff, 0, 31),
        agc_limiter_enabled: (args[0] ?? 1) || state.agc_compression !== COMP_1_1 ? 1 : 0,
      },
    }),
    // FUNC_CTRL raw: [7]=1 [6]EN [5]SWS [3]FAULT [2]Thermal [1]=1 [0]NG_EN=0.
    tile_drive_a_2_amp_read_status: ({ state }) => ({
      scalar:
        0xc2 |
        (state.amp_enabled ? 0 : 0x20) |
        (state.amp_fault ? 0x08 : 0) |
        (state.amp_thermal ? 0x04 : 0),
    }),

    // ── DAC status / NVM / raw ──
    tile_drive_a_2_read_status: () => ({ scalar: GEN_ID_STATUS }),
    tile_drive_a_2_nvm_save: () => ({}),
    // Registers reload from NVM (factory: gain 1× EXT); the driver re-reads
    // both channels' gains.
    tile_drive_a_2_nvm_reload: () => ({
      nextState: { ch0_gain: 0, ch1_gain: 0 },
    }),
    tile_drive_a_2_read_reg: ({ state, args }) => {
      const reg = (args[0] ?? 0) & 0xff;
      const map: Record<number, number> = {
        0x22: GEN_ID_STATUS,
        0x1c: state.ch0_code << 4,
        0x19: state.ch1_code << 4,
        0x13: state.ch0_margin_high << 4,
        0x14: state.ch0_margin_low << 4,
        0x01: state.ch1_margin_high << 4,
        0x02: state.ch1_margin_low << 4,
      };
      return { scalar: map[reg] ?? 0 };
    },
    // Only the DAC data / margin registers feed the model.
    tile_drive_a_2_write_reg: ({ args }) => {
      const reg = (args[0] ?? 0) & 0xff;
      const v = ((args[1] ?? 0) >> 4) & 0x0fff;
      const field: Record<number, keyof State> = {
        0x1c: 'ch0_code',
        0x19: 'ch1_code',
        0x13: 'ch0_margin_high',
        0x14: 'ch0_margin_low',
        0x01: 'ch1_margin_high',
        0x02: 'ch1_margin_low',
      };
      const f = field[reg];
      return f ? { nextState: { [f]: v } as Partial<State> } : {};
    },

    // ── blocking helpers (played out on deriveState's clock) ──
    // stop_waveform + mid-scale, then delay (tile_drive_a_2.c:676-691).
    tile_drive_a_2_play_silence: ({ args }) => {
      const cs = chans(args[0] ?? 0);
      return {
        nextState: patch(cs, { wave: WAVE_OFF, wave_running: 0, sw_wave: 0, code: DAC_MID }),
      };
    },
    // Unmute if muted, sine on the generator at the nearest of its 15 pitches
    // for `ms`, then stop + mid-scale and re-mute (tile_drive_a_2.c).
    tile_drive_a_2_play_tone: ({ state, args }) => {
      const freq = (args[1] ?? 0) & 0xffff;
      const ms = (args[2] ?? 0) & 0xffff;
      if (freq === 0 || ms === 0) return {};
      const cs = chans(args[0] ?? 0);
      const [slew, step, pitch] = waveParams(freq);
      return {
        nextState: {
          ...(state.amp_muted ? unmuteState(state) : {}),
          ...patch(cs, {
            margin_low: 0,
            margin_high: DAC_MAX,
            wave: WAVE_SINE,
            step,
            slew,
            wave_running: 1,
            freq: pitch,
          }),
          currently_playing: `tone ${pitch} Hz (asked ${freq}) / ${ms} ms`,
          play_channels: maskOf(cs),
          play_remute: state.amp_muted,
          play_ms: ms,
          playing_until_ms: 0,
        },
      };
    },
    // Software sine sweep at 8 ksps, full swing, then mid-scale
    // (tile_drive_a_2.c:771-834).
    tile_drive_a_2_play_chirp: ({ state, args }) => {
      const start = (args[1] ?? 0) & 0xffff;
      const end = (args[2] ?? 0) & 0xffff;
      const ms = (args[3] ?? 0) & 0xffff;
      if (ms === 0 || (start === 0 && end === 0)) return {};
      const cs = chans(args[0] ?? 0);
      return {
        nextState: {
          ...(state.amp_muted ? unmuteState(state) : {}),
          ...patch(cs, { wave: WAVE_OFF, wave_running: 0, sw_wave: 1, freq: end }),
          currently_playing: `chirp ${start}→${end} Hz / ${ms} ms`,
          play_channels: maskOf(cs),
          play_remute: state.amp_muted,
          play_ms: ms,
          playing_until_ms: 0,
        },
      };
    },
    // pct·30/100 dB at compression 1:1, else -28 + pct·58/100 dB, on both amps;
    // the mute shadow follows.
    tile_drive_a_2_set_volume_pct: ({ state, args }) => {
      const pct = Math.min(100, (args[1] ?? 0) & 0xff);
      const db =
        state.agc_compression === COMP_1_1
          ? Math.trunc((pct * 30) / 100)
          : -28 + Math.trunc((pct * 58) / 100);
      return {
        nextState: { amp_gain_db: db, ...(state.amp_muted ? { amp_muted_gain_db: db } : {}) },
      };
    },
    tile_drive_a_2_mute: ({ state }) => ({ nextState: muteState(state) }),
    tile_drive_a_2_unmute: ({ state }) => ({ nextState: unmuteState(state) }),
  },

  provenance: {
    tile_drive_a_2_find: 'canonical',
    tile_drive_a_2_set: 'canonical', // DAC-X-DATA, 12-bit
    tile_drive_a_2_set_mv: 'canonical', // Eq 1-3, per-channel gain, V+ as told
    tile_drive_a_2_set_supply_mv: 'canonical', // driver shadow, clamped 2.5-5.5 V
    tile_drive_a_2_get: 'canonical',
    tile_drive_a_2_set_gain: 'canonical', // VOUT-GAIN table
    tile_drive_a_2_set_waveform: 'canonical', // FUNC-CONFIG codes
    tile_drive_a_2_stop_waveform: 'canonical',
    tile_drive_a_2_set_margins: 'canonical',
    tile_drive_a_2_set_waveform_params: 'canonical',
    tile_drive_a_2_amp_set_gain: 'canonical', // -28..+30 dB
    tile_drive_a_2_amp_get_gain: 'canonical',
    tile_drive_a_2_amp_enable: 'canonical', // FUNC_CTRL EN/SWS
    tile_drive_a_2_amp_disable: 'canonical',
    tile_drive_a_2_amp_set_limiter: 'canonical', // Table 11 levels; disable only at 1:1
    tile_drive_a_2_amp_read_status: 'canonical', // FUNC_CTRL bits
    tile_drive_a_2_read_status: 'canonical', // GENERAL-STATUS DEVICE-ID
    tile_drive_a_2_set_volume_pct: 'canonical',
    tile_drive_a_2_mute: 'canonical',
    tile_drive_a_2_unmute: 'canonical',
    tile_drive_a_2_sleep: 'inferred',
    tile_drive_a_2_wake: 'inferred',
    tile_drive_a_2_start_waveform: 'canonical', // START-FUNC-X, per channel; sine level from Table 6-10
    tile_drive_a_2_set_slew_rate: 'inferred', // stored; frequency not derived from it
    tile_drive_a_2_set_code_step: 'inferred',
    tile_drive_a_2_set_phase: 'inferred',
    tile_drive_a_2_play_tone: 'canonical', // pitch from Eq 8, level from Table 6-10
    tile_drive_a_2_play_silence: 'inferred',
    tile_drive_a_2_play_chirp: 'inferred',
    tile_drive_a_2_nvm_reload: 'inferred', // assumes factory NVM
    tile_drive_a_2_read_reg: 'inferred', // data/margin/status registers only
    tile_drive_a_2_write_reg: 'inferred',
    tile_drive_a_2_nvm_save: 'hallucinated', // no NVM model
    tile_drive_a_2_amp_set_agc: 'hallucinated', // struct arg unreadable → no-op
    power: 'inferred', // IC currents datasheet; speaker load assumed 8 Ω; limiter caps the peak
  },

  // A blocking helper ends: stop the generator / software sweep, park at
  // mid-scale, re-mute if the helper unmuted.
  deriveState: (state, { t }) => {
    if (state.play_ms > 0 && state.playing_until_ms === 0) {
      return { playing_until_ms: t + state.play_ms, play_ms: 0 };
    }
    if (state.playing_until_ms !== 0 && t >= state.playing_until_ms) {
      const cs = ([0, 1] as Ch[]).filter((c) => state.play_channels & (1 << c));
      return {
        ...patch(cs, { wave: WAVE_OFF, wave_running: 0, sw_wave: 0, code: DAC_MID }),
        ...(state.play_remute ? muteState(state) : {}),
        currently_playing: '',
        play_channels: 0,
        play_remute: 0,
        playing_until_ms: 0,
      };
    }
    return {};
  },

  // OUT.0+ = pad 7, OUT.0- = pad 6, OUT.1+ = pad 9, OUT.1- = pad 8.
  padOutputs(state) {
    const o0 = outLevel(state, 0);
    const o1 = outLevel(state, 1);
    return { '7': o0, '6': o0, '9': o1, '8': o1 };
  },

  // Single V+ rail (pad 10): DAC (150 µA/ch) + two amps (TPA2028D1 IDD, or the
  // software-shutdown current) + the audio power at ~91 % Class-D efficiency.
  power(state, ctx) {
    const vMv = ctx?.padVoltage['10'] ?? state.vplus_mv;
    const intRef = [state.ch0_gain, state.ch1_gain].filter((g) => g >= 2).length;
    const dacUa = state.sleeping ? I_DAC_SLEEP_UA : 2 * I_DAC_CH_UA + intRef * I_DAC_INTREF_CH_UA;
    const ampOn = !state.sleeping && state.amp_enabled === 1;
    const ampUa = 2 * ampIq(vMv, ampOn);
    const o0 = outLevel(state, 0);
    const o1 = outLevel(state, 1);
    const pmaxW = AMP_PMAX_5V_W * Math.pow(vMv / 5000, 2);
    const audioW = (o0 * o0 + o1 * o1) * pmaxW;
    const audioUa = vMv > 0 ? (audioW / (AMP_EFF * (vMv / 1000))) * 1e6 : 0;
    const drawUa = Math.round(dacUa + ampUa + audioUa);
    const outUa = (o: number) => Math.round(((o * vMv) / R_SPEAKER_OHM / Math.SQRT2) * 1000);
    return {
      draw_ua: drawUa,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: vMv,
          i_ua: drawUa,
          pads: ['10'],
          note: 'DAC + 2× amp + audio',
        },
        {
          name: 'OUT.0',
          role: 'output',
          v_mv: Math.round(o0 * vMv),
          i_ua: outUa(o0),
          pads: ['7', '6'],
          note: 'amp0 speaker (8 Ω assumed), peak',
        },
        {
          name: 'OUT.1',
          role: 'output',
          v_mv: Math.round(o1 * vMv),
          i_ua: outUa(o1),
          pads: ['9', '8'],
          note: 'amp1 speaker (8 Ω assumed), peak',
        },
      ],
    };
  },
};

export default sim;
