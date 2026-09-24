// src/sims/drive_a_2.ts
var DAC_MID = 2048;
var DAC_MAX = 4095;
var WAVE_SINE = 4;
var WAVE_OFF = 7;
var GEN_ID_STATUS = 6 << 2;
var DRIVER_VREF_MV = [3300, 3300, 1815, 2420, 3630, 4840];
var driverVref = (gain) => DRIVER_VREF_MV[gain] ?? 3300;
var I_DAC_CH_UA = 150;
var I_DAC_INTREF_CH_UA = 12.5;
var I_DAC_SLEEP_UA = 28;
var R_SPEAKER_OHM = 8;
var AMP_PMAX_5V_W = 1.4;
var AMP_EFF = 0.91;
function ampIq(vMv, on) {
  const [a, b, c] = on ? [1500, 1700, 2e3] : [35, 50, 75];
  if (vMv <= 3600) return a + (b - a) * (Math.max(2500, vMv) - 2500) / 1100;
  return b + (c - b) * (Math.min(5500, vMv) - 3600) / 1900;
}
var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
var chans = (ch) => ch === 2 ? [0, 1] : ch === 0 || ch === 1 ? [ch] : [];
var maskOf = (cs) => cs.reduce((m, c) => m | 1 << c, 0);
function patch(cs, fields) {
  const out = {};
  for (const c of cs) for (const [k, v] of Object.entries(fields)) out[`ch${c}_${k}`] = v;
  return out;
}
var realVref = (s, gain) => gain <= 1 ? s.vplus_mv : Math.min(s.vplus_mv, 1210 * [0, 0, 1.5, 2, 3, 4][gain] | 0);
function inputCodes(s, c) {
  const g = (k) => s[`ch${c}_${k}`];
  if (g("sw_wave")) return 2046;
  if (g("wave_running") && g("wave") !== WAVE_OFF) {
    const lo = g("margin_low");
    const hi = g("margin_high");
    return Math.abs((lo + hi) / 2 - DAC_MID) + (hi - lo) / 2;
  }
  return Math.abs(g("code") - DAC_MID);
}
function outLevel(s, c) {
  if (s.sleeping || !s.amp_enabled || s.amp_fault || s.amp_thermal) return 0;
  const vref = realVref(s, c === 0 ? s.ch0_gain : s.ch1_gain);
  const vinMv = inputCodes(s, c) * vref / 4096;
  const voutMv = vinMv * Math.pow(10, s.amp_gain_db / 20);
  return s.vplus_mv > 0 ? clamp(voutMv / s.vplus_mv, 0, 1) : 0;
}
var muteState = (s) => ({
  amp_enabled: 0,
  ...s.amp_muted ? {} : { amp_muted: 1, amp_muted_gain_db: s.amp_gain_db }
});
var unmuteState = (s) => ({
  amp_enabled: 1,
  ...s.amp_muted ? { amp_muted: 0, amp_gain_db: s.amp_muted_gain_db } : {}
});
function waveParams(f) {
  if (f >= 5e3) return [1, 7];
  if (f >= 2e3) return [1, 5];
  if (f >= 1e3) return [2, 5];
  if (f >= 500) return [4, 5];
  if (f >= 200) return [6, 5];
  if (f >= 100) return [8, 5];
  if (f >= 50) return [10, 5];
  if (f >= 10) return [13, 5];
  return [15, 0];
}
var onCh = (args, fn) => {
  const ch = args[0] ?? 0;
  return ch === 0 || ch === 1 ? { nextState: fn(ch) } : {};
};
var sim = {
  tile: "Drive.A.2",
  // After tile_drive_a_2_init(cfg = NULL) (tile_drive_a_2.c:178-277): gain
  // 1× EXT on both channels, both DACs at mid-scale, amp gain 6 dB, AGC
  // compression 1:1 / max 30 dB / limiter off, amps on (FUNC_CTRL 0xC2).
  defaultState: {
    vplus_mv: 3300,
    // the driver's VDD assumption (resolve_vref)
    amp_fault: 0,
    amp_thermal: 0,
    ch0_code: DAC_MID,
    ch1_code: DAC_MID,
    ch0_gain: 0,
    ch1_gain: 0,
    vref_mv: 3300,
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
    amp_muted: 0,
    amp_muted_gain_db: 6,
    agc_compression: 0,
    // AGC_CTRL2 0xC0
    agc_max_gain_db: 30,
    agc_limiter_level: 0,
    // AGC_CTRL1 0x80 (limiter off)
    agc_attack: 5,
    // chip defaults, not written by init
    agc_release: 11,
    agc_hold: 0,
    agc_noise_gate: 0,
    sleeping: 0,
    currently_playing: "",
    play_channels: 0,
    play_remute: 0,
    play_ms: 0,
    playing_until_ms: 0
  },
  controls: [
    {
      type: "slider",
      field: "vplus_mv",
      label: "V+ supply",
      min: 2500,
      max: 5500,
      step: 50,
      unit: "mV",
      description: "Tile supply. It is also the DAC's reference, so it sets the real full scale; set_mv() still assumes 3.3 V."
    },
    {
      type: "toggle",
      field: "amp_fault",
      label: "Fault: speaker short",
      description: "TPA2028D1 FAULT: output stops. Cleared when the driver rewrites FUNC_CTRL (init / wake)."
    },
    {
      type: "toggle",
      field: "amp_thermal",
      label: "Fault: over-temp",
      description: "TPA2028D1 thermal shutdown (die above 150 \xB0C): output stops."
    }
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
        vref_mv: 3300,
        amp_gain_db: 6,
        amp_enabled: 1,
        amp_muted: 0,
        amp_fault: 0,
        amp_thermal: 0,
        agc_compression: 0,
        agc_max_gain_db: 30,
        agc_limiter_level: 0,
        agc_noise_gate: 0,
        sleeping: 0
      }
    }),
    // VOUT Hi-Z + amp SWS (tile_drive_a_2.c:279-293).
    tile_drive_a_2_sleep: () => ({
      nextState: {
        sleeping: 1,
        amp_enabled: 0,
        ch0_wave_running: 0,
        ch1_wave_running: 0,
        ch0_sw_wave: 0,
        ch1_sw_wave: 0
      }
    }),
    // DACs back at mid-scale, then FUNC_CTRL = 0xC2 — the amps come on even if
    // mute() was in effect, and FAULT/Thermal clear (tile_drive_a_2.c:295-324).
    tile_drive_a_2_wake: () => ({
      nextState: {
        sleeping: 0,
        ch0_code: DAC_MID,
        ch1_code: DAC_MID,
        amp_enabled: 1,
        amp_fault: 0,
        amp_thermal: 0
      }
    }),
    // ── DAC output ──
    tile_drive_a_2_set: ({ args }) => onCh(args, (c) => patch([c], { code: clamp((args[1] ?? 0) & 65535, 0, DAC_MAX) })),
    // code = mv·4096 / vref_mv, the driver's single cached vref (tile_drive_a_2.c:343-351).
    tile_drive_a_2_set_mv: ({ state, args }) => onCh(args, (c) => {
      const mv = (args[1] ?? 0) & 65535;
      const code = state.vref_mv ? Math.floor(mv * 4096 / state.vref_mv) : 0;
      return patch([c], { code: Math.min(DAC_MAX, code) });
    }),
    tile_drive_a_2_get: ({ state, args }) => {
      const ch = args[0] ?? 0;
      return { scalar: ch === 0 ? state.ch0_code : ch === 1 ? state.ch1_code : 0 };
    },
    tile_drive_a_2_set_gain: ({ args }) => onCh(args, (c) => {
      const gain = (args[1] ?? 0) & 7;
      return { ...patch([c], { gain }), vref_mv: driverVref(gain) };
    }),
    // ── function generator ──
    tile_drive_a_2_set_waveform: ({ args }) => onCh(args, (c) => patch([c], { wave: (args[1] ?? WAVE_OFF) & 7 })),
    tile_drive_a_2_start_waveform: ({ args }) => onCh(args, (c) => patch([c], { wave_running: 1 })),
    // FUNC-CONFIG = OFF (tile_drive_a_2.c:411-418).
    tile_drive_a_2_stop_waveform: ({ args }) => onCh(args, (c) => patch([c], { wave: WAVE_OFF, wave_running: 0 })),
    tile_drive_a_2_set_slew_rate: ({ args }) => onCh(args, (c) => patch([c], { slew: (args[1] ?? 0) & 15 })),
    tile_drive_a_2_set_code_step: ({ args }) => onCh(args, (c) => patch([c], { step: (args[1] ?? 0) & 7 })),
    // Clamped and swapped so high > low (tile_drive_a_2.c:440-455).
    tile_drive_a_2_set_margins: ({ args }) => onCh(args, (c) => {
      const a = Math.min(DAC_MAX, (args[1] ?? 0) & 65535);
      const b = Math.min(DAC_MAX, (args[2] ?? DAC_MAX) & 65535);
      return patch([c], { margin_low: Math.min(a, b), margin_high: Math.max(a, b) });
    }),
    tile_drive_a_2_set_phase: ({ args }) => onCh(args, (c) => patch([c], { phase: (args[1] ?? 0) & 3 })),
    // Margins to full scale + wave/step/slew (tile_drive_a_2.c:467-486).
    tile_drive_a_2_set_waveform_params: ({ args }) => onCh(
      args,
      (c) => patch([c], {
        margin_low: 0,
        margin_high: DAC_MAX,
        wave: (args[1] ?? WAVE_OFF) & 7,
        step: (args[2] ?? 0) & 7,
        slew: (args[3] ?? 0) & 15
      })
    ),
    // ── amps (both, shared 0x58) ──
    tile_drive_a_2_amp_set_gain: ({ args }) => {
      const raw = (args[0] ?? 0) & 255;
      return { nextState: { amp_gain_db: clamp(raw > 127 ? raw - 256 : raw, -28, 30) } };
    },
    tile_drive_a_2_amp_get_gain: ({ state }) => ({ scalar: state.amp_gain_db }),
    // EN = 1, SWS = 0; the mute shadow is untouched (tile_drive_a_2.c:507-513).
    tile_drive_a_2_amp_enable: () => ({ nextState: { amp_enabled: 1 } }),
    tile_drive_a_2_amp_disable: () => ({ nextState: { amp_enabled: 0 } }),
    // The config is a struct pointer the host call can't read; no state change.
    tile_drive_a_2_amp_set_agc: () => ({}),
    // FUNC_CTRL raw: [7]=1 [6]EN [5]SWS [3]FAULT [2]Thermal [1]=1 [0]NG_EN=0.
    tile_drive_a_2_amp_read_status: ({ state }) => ({
      scalar: 194 | (state.amp_enabled ? 0 : 32) | (state.amp_fault ? 8 : 0) | (state.amp_thermal ? 4 : 0)
    }),
    // ── DAC status / NVM / raw ──
    tile_drive_a_2_read_status: () => ({ scalar: GEN_ID_STATUS }),
    tile_drive_a_2_nvm_save: () => ({}),
    // Registers reload from NVM (factory: gain 1× EXT); the driver re-derives
    // its gain shadow and takes vref from channel 0 (tile_drive_a_2.c:584-599).
    tile_drive_a_2_nvm_reload: () => ({
      nextState: { ch0_gain: 0, ch1_gain: 0, vref_mv: driverVref(0) }
    }),
    tile_drive_a_2_read_reg: ({ state, args }) => {
      const reg = (args[0] ?? 0) & 255;
      const map = {
        34: GEN_ID_STATUS,
        28: state.ch0_code << 4,
        25: state.ch1_code << 4,
        19: state.ch0_margin_high << 4,
        20: state.ch0_margin_low << 4,
        1: state.ch1_margin_high << 4,
        2: state.ch1_margin_low << 4
      };
      return { scalar: map[reg] ?? 0 };
    },
    // Only the DAC data / margin registers feed the model.
    tile_drive_a_2_write_reg: ({ args }) => {
      const reg = (args[0] ?? 0) & 255;
      const v = (args[1] ?? 0) >> 4 & 4095;
      const field = {
        28: "ch0_code",
        25: "ch1_code",
        19: "ch0_margin_high",
        20: "ch0_margin_low",
        1: "ch1_margin_high",
        2: "ch1_margin_low"
      };
      const f = field[reg];
      return f ? { nextState: { [f]: v } } : {};
    },
    // ── blocking helpers (played out on deriveState's clock) ──
    // stop_waveform + mid-scale, then delay (tile_drive_a_2.c:676-691).
    tile_drive_a_2_play_silence: ({ args }) => {
      const cs = chans(args[0] ?? 0);
      return {
        nextState: patch(cs, { wave: WAVE_OFF, wave_running: 0, sw_wave: 0, code: DAC_MID })
      };
    },
    // Unmute if muted, sine on the generator for `ms`, then stop + mid-scale and
    // re-mute (tile_drive_a_2.c:749-767).
    tile_drive_a_2_play_tone: ({ state, args }) => {
      const freq = (args[1] ?? 0) & 65535;
      const ms = (args[2] ?? 0) & 65535;
      if (freq === 0 || ms === 0) return {};
      const cs = chans(args[0] ?? 0);
      const [slew, step] = waveParams(freq);
      return {
        nextState: {
          ...state.amp_muted ? unmuteState(state) : {},
          ...patch(cs, {
            margin_low: 0,
            margin_high: DAC_MAX,
            wave: WAVE_SINE,
            step,
            slew,
            wave_running: 1,
            freq
          }),
          currently_playing: `tone ${freq} Hz / ${ms} ms`,
          play_channels: maskOf(cs),
          play_remute: state.amp_muted,
          play_ms: ms,
          playing_until_ms: 0
        }
      };
    },
    // Software sine sweep at 8 ksps, full swing, then mid-scale
    // (tile_drive_a_2.c:771-834).
    tile_drive_a_2_play_chirp: ({ state, args }) => {
      const start = (args[1] ?? 0) & 65535;
      const end = (args[2] ?? 0) & 65535;
      const ms = (args[3] ?? 0) & 65535;
      if (ms === 0 || start === 0 && end === 0) return {};
      const cs = chans(args[0] ?? 0);
      return {
        nextState: {
          ...state.amp_muted ? unmuteState(state) : {},
          ...patch(cs, { wave: WAVE_OFF, wave_running: 0, sw_wave: 1, freq: end }),
          currently_playing: `chirp ${start}\u2192${end} Hz / ${ms} ms`,
          play_channels: maskOf(cs),
          play_remute: state.amp_muted,
          play_ms: ms,
          playing_until_ms: 0
        }
      };
    },
    // -28 + pct·58/100 dB on both amps; the mute shadow follows (tile_drive_a_2.c:838-853).
    tile_drive_a_2_set_volume_pct: ({ state, args }) => {
      const pct = Math.min(100, (args[1] ?? 0) & 255);
      const db = -28 + Math.trunc(pct * 58 / 100);
      return {
        nextState: { amp_gain_db: db, ...state.amp_muted ? { amp_muted_gain_db: db } : {} }
      };
    },
    tile_drive_a_2_mute: ({ state }) => ({ nextState: muteState(state) }),
    tile_drive_a_2_unmute: ({ state }) => ({ nextState: unmuteState(state) })
  },
  provenance: {
    tile_drive_a_2_find: "canonical",
    tile_drive_a_2_set: "canonical",
    // DAC-X-DATA, 12-bit
    tile_drive_a_2_set_mv: "canonical",
    // driver math incl. its single cached vref
    tile_drive_a_2_get: "canonical",
    tile_drive_a_2_set_gain: "canonical",
    // VOUT-GAIN table
    tile_drive_a_2_set_waveform: "canonical",
    // FUNC-CONFIG codes
    tile_drive_a_2_stop_waveform: "canonical",
    tile_drive_a_2_set_margins: "canonical",
    tile_drive_a_2_set_waveform_params: "canonical",
    tile_drive_a_2_amp_set_gain: "canonical",
    // -28..+30 dB
    tile_drive_a_2_amp_get_gain: "canonical",
    tile_drive_a_2_amp_enable: "canonical",
    // FUNC_CTRL EN/SWS
    tile_drive_a_2_amp_disable: "canonical",
    tile_drive_a_2_amp_read_status: "canonical",
    // FUNC_CTRL bits
    tile_drive_a_2_read_status: "canonical",
    // GENERAL-STATUS DEVICE-ID
    tile_drive_a_2_set_volume_pct: "canonical",
    tile_drive_a_2_mute: "canonical",
    tile_drive_a_2_unmute: "canonical",
    tile_drive_a_2_sleep: "inferred",
    tile_drive_a_2_wake: "inferred",
    tile_drive_a_2_start_waveform: "inferred",
    // generator output level only, no waveform
    tile_drive_a_2_set_slew_rate: "inferred",
    // stored; frequency not derived from it
    tile_drive_a_2_set_code_step: "inferred",
    tile_drive_a_2_set_phase: "inferred",
    tile_drive_a_2_play_tone: "inferred",
    tile_drive_a_2_play_silence: "inferred",
    tile_drive_a_2_play_chirp: "inferred",
    tile_drive_a_2_nvm_reload: "inferred",
    // assumes factory NVM
    tile_drive_a_2_read_reg: "inferred",
    // data/margin/status registers only
    tile_drive_a_2_write_reg: "inferred",
    tile_drive_a_2_nvm_save: "hallucinated",
    // no NVM model
    tile_drive_a_2_amp_set_agc: "hallucinated",
    // struct arg unreadable → no-op
    power: "inferred"
    // IC currents datasheet; speaker load assumed 8 Ω
  },
  // A blocking helper ends: stop the generator / software sweep, park at
  // mid-scale, re-mute if the helper unmuted.
  deriveState: (state, { t }) => {
    if (state.play_ms > 0 && state.playing_until_ms === 0) {
      return { playing_until_ms: t + state.play_ms, play_ms: 0 };
    }
    if (state.playing_until_ms !== 0 && t >= state.playing_until_ms) {
      const cs = [0, 1].filter((c) => state.play_channels & 1 << c);
      return {
        ...patch(cs, { wave: WAVE_OFF, wave_running: 0, sw_wave: 0, code: DAC_MID }),
        ...state.play_remute ? muteState(state) : {},
        currently_playing: "",
        play_channels: 0,
        play_remute: 0,
        playing_until_ms: 0
      };
    }
    return {};
  },
  // OUT.0+ = pad 7, OUT.0- = pad 6, OUT.1+ = pad 9, OUT.1- = pad 8.
  padOutputs(state) {
    const o0 = outLevel(state, 0);
    const o1 = outLevel(state, 1);
    return { "7": o0, "6": o0, "9": o1, "8": o1 };
  },
  // Single V+ rail (pad 10): DAC (150 µA/ch) + two amps (TPA2028D1 IDD, or the
  // software-shutdown current) + the audio power at ~91 % Class-D efficiency.
  power(state, ctx) {
    const vMv = ctx?.padVoltage["10"] ?? state.vplus_mv;
    const intRef = [state.ch0_gain, state.ch1_gain].filter((g) => g >= 2).length;
    const dacUa = state.sleeping ? I_DAC_SLEEP_UA : 2 * I_DAC_CH_UA + intRef * I_DAC_INTREF_CH_UA;
    const ampOn = !state.sleeping && state.amp_enabled === 1;
    const ampUa = 2 * ampIq(vMv, ampOn);
    const o0 = outLevel(state, 0);
    const o1 = outLevel(state, 1);
    const pmaxW = AMP_PMAX_5V_W * Math.pow(vMv / 5e3, 2);
    const audioW = (o0 * o0 + o1 * o1) * pmaxW;
    const audioUa = vMv > 0 ? audioW / (AMP_EFF * (vMv / 1e3)) * 1e6 : 0;
    const drawUa = Math.round(dacUa + ampUa + audioUa);
    const outUa = (o) => Math.round(o * vMv / R_SPEAKER_OHM / Math.SQRT2 * 1e3);
    return {
      draw_ua: drawUa,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: vMv,
          i_ua: drawUa,
          pads: ["10"],
          note: "DAC + 2\xD7 amp + audio"
        },
        {
          name: "OUT.0",
          role: "output",
          v_mv: Math.round(o0 * vMv),
          i_ua: outUa(o0),
          pads: ["7", "6"],
          note: "amp0 speaker (8 \u03A9 assumed), peak"
        },
        {
          name: "OUT.1",
          role: "output",
          v_mv: Math.round(o1 * vMv),
          i_ua: outUa(o1),
          pads: ["9", "8"],
          note: "amp1 speaker (8 \u03A9 assumed), peak"
        }
      ]
    };
  }
};
var drive_a_2_default = sim;
export {
  drive_a_2_default as default
};
