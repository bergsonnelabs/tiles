// src/sims/drive_h.ts
var MODE_INTERNAL_TRIG = 0;
var MODE_EXT_EDGE = 1;
var MODE_EXT_LEVEL = 2;
var MODE_PWM_ANALOG = 3;
var MODE_AUDIO = 4;
var MODE_RTP = 5;
var STATUS_DEVICE_ID = 96;
var EFFECT_MS = 140;
var INIT_CONFIG = {
  mode_reg: MODE_INTERNAL_TRIG,
  standby: 0,
  library: 6,
  n_erm_lra: 1,
  // FEEDBACK_CTRL 0xB6
  closed_loop: 1,
  rated_voltage: 86,
  // 1.8 Vrms
  od_clamp: 140,
  fb_brake: 3,
  // 0xB6[6:4]
  loop_gain: 1,
  // 0xB6[3:2]
  drive_time: 16
  // ~238 Hz coin LRA
};
var ready = (s) => s.standby === 0;
var vbatReg = (mv) => Math.max(0, Math.min(255, Math.floor(mv * 255 / 5600)));
var lraPeriodReg = (hz) => hz > 0 ? Math.max(1, Math.min(255, Math.round(1e6 / (hz * 98.46)))) : 0;
function driveStrength(s) {
  if (s.standby) return 0;
  if (s.mode_reg === MODE_RTP) {
    if (s.rtp_unsigned) return Math.min(1, s.rtp_amplitude / 255);
    const v = s.rtp_amplitude > 127 ? s.rtp_amplitude - 256 : s.rtp_amplitude;
    return Math.min(1, Math.abs(v) / 127);
  }
  if (s.mode_reg <= MODE_EXT_LEVEL && s.go) return 0.8;
  return 0;
}
function sampled(s) {
  return {
    vbat_reg: vbatReg(s.vbat_mv),
    ...s.n_erm_lra ? { lra_period_reg: lraPeriodReg(s.resonance_hz) } : {}
  };
}
function isqrt(x) {
  return Math.floor(Math.sqrt(x));
}
function parseSequence(seq) {
  const slots = [];
  let ms = 0;
  let first = 0;
  for (const raw of seq.slice(0, 8)) {
    const v = Number(raw) & 255;
    if (v === 0) break;
    if (v & 128) {
      const w = (v & 127) * 10;
      slots.push(`w${w}ms`);
      ms += w;
    } else {
      if (first === 0) first = v;
      slots.push(String(v));
      ms += EFFECT_MS;
    }
  }
  return { slots, ms, first };
}
function startSequence(s, seq, label) {
  const { slots, ms, first } = parseSequence(seq);
  const playing = slots.some((x) => !x.startsWith("w"));
  return {
    last_effect: first,
    effect_repeats: 0,
    last_sequence: slots.join(","),
    go: playing ? 1 : 0,
    currently_playing: playing ? label : "",
    play_ms: playing ? ms : 0,
    playing_until_ms: 0,
    ...playing && s.mode_reg <= MODE_EXT_LEVEL ? sampled(s) : {}
  };
}
var engineOk = (s) => s.vbat_mv >= 2500 && s.vbat_mv <= 5500 && !s.oc_detect;
var sim = {
  tile: "Drive.H",
  defaultState: {
    ...INIT_CONFIG,
    go: 0,
    rtp_amplitude: 0,
    last_effect: 0,
    effect_repeats: 0,
    last_sequence: "",
    currently_playing: "",
    play_ms: 0,
    playing_until_ms: 0,
    vbat_mv: 3700,
    resonance_hz: 175,
    over_temp: 0,
    oc_detect: 0,
    calibrated: 0,
    diag_result: 0,
    vbat_reg: 0,
    lra_period_reg: 0,
    // CONTROL2 reset 0xF5, CONTROL3 reset 0xA0 (signed RTP), timing offsets 0.
    sample_time: 3,
    blanking_time: 1,
    idiss_time: 1,
    ot_overdrive: 0,
    ot_sustain_pos: 0,
    ot_sustain_neg: 0,
    ot_brake: 0,
    rtp_unsigned: 0,
    rtp_bidir: 1,
    // ATV_CTRL reset 0x05; ATV min/max input 0x19/0xFF, min/max drive 0x19/0xFF.
    atv_peak_time: 1,
    atv_filter: 1,
    atv_min_input: 25,
    atv_max_input: 255,
    atv_min_drive: 25,
    atv_max_drive: 255,
    otp_programmed: 0
  },
  controls: [
    {
      type: "slider",
      field: "vbat_mv",
      label: "VBAT",
      min: 2300,
      max: 5500,
      step: 10,
      unit: "mV",
      description: "Supply at the chip. The VBAT register samples it only while a waveform is playing; diagnostics and calibration fail outside 2.5-5.5 V."
    },
    {
      type: "slider",
      field: "resonance_hz",
      label: "LRA resonance",
      min: 50,
      max: 300,
      step: 1,
      unit: "Hz",
      description: "Mechanical resonance of the wired LRA. The LRA_PERIOD register measures it while driving; get_resonance_hz() reads it back."
    },
    {
      type: "toggle",
      field: "over_temp",
      label: "Fault: over-temp",
      description: "Latch STATUS OVER_TEMP. Cleared when the firmware reads get_status()."
    },
    {
      type: "toggle",
      field: "oc_detect",
      label: "Fault: over-current",
      description: "Latch STATUS OC_DETECT (shorted actuator). Cleared on get_status(); diagnostics fail while set."
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    tile_drive_h_find: () => ({ scalar: 1 }),
    tile_drive_h_init: () => ({
      nextState: { ...INIT_CONFIG, go: 0, rtp_amplitude: 0, play_ms: 0, playing_until_ms: 0 }
    }),
    // MODE = 0x40 (STANDBY, mode bits 0); READY-gated (tile_drive_h.c:440-445).
    tile_drive_h_standby: ({ state }) => ready(state) ? {
      nextState: {
        standby: 1,
        mode_reg: MODE_INTERNAL_TRIG,
        go: 0,
        currently_playing: "",
        play_ms: 0,
        playing_until_ms: 0
      }
    } : {},
    tile_drive_h_wake: ({ state }) => state.standby ? { nextState: { standby: 0, mode_reg: MODE_INTERNAL_TRIG } } : {},
    // ── waveform sequencer ──
    // Slot 0 = index, slot 1 = stop, GO; each extra repeat re-fires GO after
    // 200 ms (tile_drive_h.c:195-214). Does not touch MODE.
    tile_drive_h_play: ({ state, args }) => {
      if (!ready(state)) return {};
      const effect = (args[0] ?? 0) & 255;
      const repeats = (args[1] ?? 0) & 255;
      return {
        nextState: {
          last_effect: effect,
          effect_repeats: repeats,
          last_sequence: String(effect),
          go: 1,
          currently_playing: `effect ${effect}`,
          play_ms: EFFECT_MS + 200 * Math.max(0, repeats - 1),
          playing_until_ms: 0,
          ...state.mode_reg <= MODE_EXT_LEVEL ? sampled(state) : {}
        }
      };
    },
    tile_drive_h_play_sequence: ({ state, bufferIn }) => ready(state) ? { nextState: startSequence(state, bufferIn?.effects ?? [], "sequence") } : {},
    // Loads the slots without GO — an external trigger fires them.
    tile_drive_h_load_sequence: ({ state, bufferIn }) => {
      if (!ready(state)) return {};
      const { slots, first } = parseSequence(bufferIn?.effects ?? []);
      return {
        nextState: { last_effect: first, effect_repeats: 0, last_sequence: slots.join(",") }
      };
    },
    // Slot byte = 0x80 | steps (×10 ms); slot >= 8 ignored (tile_drive_h.c:459-472).
    tile_drive_h_set_sequence_wait: ({ state, args }) => {
      const slot = args[0] ?? 0;
      if (!ready(state) || slot < 0 || slot >= 8) return {};
      const ms = ((args[1] ?? 0) & 127) * 10;
      const parts = state.last_sequence ? state.last_sequence.split(",") : [];
      while (parts.length <= slot) parts.push("0");
      parts[slot] = `w${ms}ms`;
      return { nextState: { last_sequence: parts.join(",") } };
    },
    // DRIVE_H_TRIG_EDGE → MODE 1, LEVEL → 2, anything else → internal
    // (tile_drive_h.c:259-279).
    tile_drive_h_set_trigger: ({ state, args }) => {
      if (!ready(state)) return {};
      const m = args[0] ?? 0;
      return {
        nextState: {
          mode_reg: m === 1 ? MODE_EXT_EDGE : m === 2 ? MODE_EXT_LEVEL : MODE_INTERNAL_TRIG
        }
      };
    },
    tile_drive_h_is_playing: ({ state }) => ({ scalar: state.go ? 1 : 0 }),
    // GO = 0 only (tile_drive_h.c:286-289).
    tile_drive_h_stop: () => ({
      nextState: { go: 0, currently_playing: "", play_ms: 0, playing_until_ms: 0 }
    }),
    // ── library + actuator tuning ──
    tile_drive_h_set_library: ({ state, args }) => {
      const lib = args[0] ?? 0;
      if (!ready(state) || lib < 0 || lib > 6) return {};
      return {
        nextState: { library: lib, ...lib !== 0 ? { n_erm_lra: lib === 6 ? 1 : 0 } : {} }
      };
    },
    // 0 leaves RATED/OD untouched; 0xFF leaves brake/gain (tile_drive_h.c:505-535).
    tile_drive_h_set_actuator_params: ({ state, args }) => {
      if (!ready(state)) return {};
      const rv = (args[0] ?? 0) & 255;
      const od = (args[1] ?? 0) & 255;
      const fb = (args[2] ?? 255) & 255;
      const lg = (args[3] ?? 255) & 255;
      const u = {};
      if (rv !== 0) u.rated_voltage = rv;
      if (od !== 0) u.od_clamp = od;
      if (fb !== 255) u.fb_brake = fb & 7;
      if (lg !== 255) u.loop_gain = lg & 3;
      return { nextState: u };
    },
    tile_drive_h_set_loop_mode: ({ state, args }) => ready(state) ? { nextState: { closed_loop: args[0] ? 1 : 0 } } : {},
    // mV → RATED_VOLTAGE / OD_CLAMP, DRV2605 §7.5.2 Eq 2-5, integer math as the
    // driver (tile_drive_h.c:556-609): f from DRIVE_TIME, t_sample from CONTROL2.
    tile_drive_h_set_actuator_voltage: ({ state, args }) => {
      if (!ready(state)) return {};
      const rated = Math.min(3600, Math.max(300, (args[0] ?? 0) & 65535));
      const od = Math.min(5e3, Math.max(rated, (args[1] ?? 0) & 65535));
      let ratedReg;
      let odReg;
      if (!state.n_erm_lra) {
        ratedReg = Math.floor(rated * 100 / 2133);
        odReg = Math.floor(od * 100 / 2196);
      } else {
        const f = Math.floor(1e6 / (2 * (500 + 100 * (state.drive_time & 31))));
        const ts = [150, 200, 250, 300][state.sample_time & 3];
        const loss = Math.min(900, Math.floor((4 * ts + 300) * f / 1e3));
        ratedReg = Math.floor(rated * isqrt((1e3 - loss) * 1e3) / 20710);
        const loss2 = Math.min(900, Math.floor(f * 800 / 1e3));
        odReg = Math.floor(od * 1e5 / (2132 * isqrt((1e3 - loss2) * 1e3)));
      }
      return {
        nextState: {
          rated_voltage: Math.min(255, Math.max(1, ratedReg)),
          od_clamp: Math.min(255, Math.max(1, odReg))
        }
      };
    },
    // DRIVE_TIME = (half-period − 0.5 ms) / 0.1 ms (tile_drive_h.c:611-628).
    tile_drive_h_set_resonance_hz: ({ state, args }) => {
      if (!ready(state)) return {};
      const hz = Math.min(300, Math.max(125, args[0] ?? 0));
      const n = Math.min(31, Math.floor((Math.floor(5e5 / hz) - 500) / 100));
      return { nextState: { drive_time: n } };
    },
    tile_drive_h_set_resonance_params: ({ state, args }) => {
      if (!ready(state)) return {};
      const st = (args[0] ?? 255) & 255;
      const bt = (args[1] ?? 255) & 255;
      const it = (args[2] ?? 255) & 255;
      const u = {};
      if (st !== 255) u.sample_time = st & 3;
      if (bt !== 255) u.blanking_time = bt & 3;
      if (it !== 255) u.idiss_time = it & 3;
      return { nextState: u };
    },
    tile_drive_h_set_waveform_timing: ({ state, args }) => ready(state) ? {
      nextState: {
        ot_overdrive: (args[0] ?? 0) & 255,
        ot_sustain_pos: (args[1] ?? 0) & 255,
        ot_sustain_neg: (args[2] ?? 0) & 255,
        ot_brake: (args[3] ?? 0) & 255
      }
    } : {},
    // ── real-time playback (MODE 5) ──
    // MODE = RTP, RTP = 0; GO is not used in RTP mode (tile_drive_h.c:291-299).
    tile_drive_h_rtp_start: ({ state }) => ready(state) ? { nextState: { mode_reg: MODE_RTP, rtp_amplitude: 0 } } : {},
    tile_drive_h_rtp_write: ({ state, args }) => {
      const s = { ...state, rtp_amplitude: (args[0] ?? 0) & 255 };
      return {
        nextState: {
          rtp_amplitude: s.rtp_amplitude,
          ...driveStrength(s) > 0 ? sampled(s) : {}
        }
      };
    },
    tile_drive_h_set_rtp_format: ({ state, args }) => ready(state) ? { nextState: { rtp_unsigned: args[0] ? 1 : 0, rtp_bidir: args[1] ? 1 : 0 } } : {},
    tile_drive_h_rtp_stop: () => ({
      nextState: { rtp_amplitude: 0, mode_reg: MODE_INTERNAL_TRIG }
    }),
    // ── PWM / analog / audio on TRIG (MODE 3 / 4) ──
    tile_drive_h_pwm_input_start: ({ state }) => ready(state) ? { nextState: { mode_reg: MODE_PWM_ANALOG } } : {},
    tile_drive_h_analog_input_start: ({ state }) => ready(state) ? { nextState: { mode_reg: MODE_PWM_ANALOG } } : {},
    tile_drive_h_pwm_input_stop: () => ({ nextState: { mode_reg: MODE_INTERNAL_TRIG } }),
    tile_drive_h_audio_start: ({ state }) => ready(state) ? { nextState: { mode_reg: MODE_AUDIO } } : {},
    // 0xFF leaves a field untouched (tile_drive_h.c:779-814).
    tile_drive_h_set_audio_params: ({ state, args }) => {
      if (!ready(state)) return {};
      const v = (i) => (args[i] ?? 255) & 255;
      const u = {};
      if (v(0) !== 255) u.atv_peak_time = v(0) & 3;
      if (v(1) !== 255) u.atv_filter = v(1) & 3;
      if (v(2) !== 255) u.atv_min_input = v(2);
      if (v(3) !== 255) u.atv_max_input = v(3);
      if (v(4) !== 255) u.atv_min_drive = v(4);
      if (v(5) !== 255) u.atv_max_drive = v(5);
      return { nextState: u };
    },
    tile_drive_h_audio_stop: () => ({ nextState: { mode_reg: MODE_INTERNAL_TRIG } }),
    // ── status / diagnostics ──
    // STATUS read; the latched flags clear on read (tile_drive_h.c:112-114).
    tile_drive_h_get_status: ({ state }) => ({
      scalar: STATUS_DEVICE_ID | (state.diag_result ? 8 : 0) | (state.over_temp ? 2 : 0) | (state.oc_detect ? 1 : 0),
      nextState: { diag_result: 0, over_temp: 0, oc_detect: 0 }
    }),
    // Blocking in the driver: MODE 6 + GO, wait, read DIAG_RESULT, restore
    // MODE 0. Returns 1 = pass (tile_drive_h.c:317-365).
    tile_drive_h_diagnose: ({ state }) => {
      if (!ready(state)) return { scalar: 0 };
      const ok = engineOk(state);
      return {
        scalar: ok ? 1 : 0,
        nextState: { diag_result: ok ? 0 : 1, mode_reg: MODE_INTERNAL_TRIG, go: 0 }
      };
    },
    // Blocking: MODE 7 + GO, wait, DIAG_RESULT 0 = converged; FEEDBACK_CTRL and
    // loop mode are restored, MODE back to 0 (tile_drive_h.c:367-418). The cal
    // drives the actuator, so VBAT/LRA_PERIOD get sampled.
    tile_drive_h_calibrate: ({ state }) => {
      if (!ready(state)) return { scalar: 0 };
      const ok = engineOk(state);
      return {
        scalar: ok ? 1 : 0,
        nextState: {
          calibrated: ok ? 1 : 0,
          diag_result: ok ? 0 : 1,
          mode_reg: MODE_INTERNAL_TRIG,
          go: 0,
          ...sampled(state)
        }
      };
    },
    // !DIAG_RESULT while READY (tile_drive_h.c:907-913).
    tile_drive_h_is_calibrated: ({ state }) => ({
      scalar: ready(state) && !state.diag_result ? 1 : 0
    }),
    // VBAT: live while driving, else the last sample; raw 0 → 0.
    tile_drive_h_get_vbat_mv: ({ state }) => {
      const raw = driveStrength(state) > 0 ? vbatReg(state.vbat_mv) : state.vbat_reg;
      return { scalar: Math.floor(raw * 5600 / 255) };
    },
    // LRA_PERIOD: Hz = 10156 / raw; raw 0 → 0.
    tile_drive_h_get_resonance_hz: ({ state }) => {
      const raw = driveStrength(state) > 0 && state.n_erm_lra ? lraPeriodReg(state.resonance_hz) : state.lra_period_reg;
      return { scalar: raw > 0 ? Math.floor(10156 / raw) : 0 };
    },
    tile_drive_h_get_otp_status: ({ state }) => ({ scalar: state.otp_programmed ? 1 : 0 }),
    // ── idiomatic helpers ──
    tile_drive_h_play_click: ({ state }) => {
      if (!ready(state)) return {};
      return {
        nextState: {
          ...startSequence(state, [1], "click"),
          play_ms: 50
        }
      };
    },
    tile_drive_h_play_double_tap: ({ state }) => ready(state) ? { nextState: { ...startSequence(state, [10, 0], "double-tap"), play_ms: 150 } } : {},
    tile_drive_h_play_alert: ({ state }) => ready(state) ? { nextState: { ...startSequence(state, [14, 56, 14, 0], "alert"), play_ms: 1200 } } : {},
    // RTP at 0x7F for `ms`, then RTP 0 + MODE 0 (tile_drive_h.c:893-905). The
    // driver blocks; the twin plays it out on deriveState's clock.
    tile_drive_h_play_buzz: ({ state, args }) => {
      if (!ready(state)) return {};
      const ms = (args[0] ?? 0) & 65535;
      if (ms === 0) return { nextState: { mode_reg: MODE_INTERNAL_TRIG, rtp_amplitude: 0 } };
      const s = { ...state, mode_reg: MODE_RTP, rtp_amplitude: 127 };
      return {
        nextState: {
          mode_reg: MODE_RTP,
          rtp_amplitude: 127,
          currently_playing: `buzz ${ms} ms`,
          play_ms: ms,
          playing_until_ms: 0,
          ...sampled(s)
        }
      };
    }
  },
  provenance: {
    tile_drive_h_find: "canonical",
    // fixed 0x5A, ACK → 1
    tile_drive_h_get_status: "canonical",
    // STATUS bits, DEVICE_ID 3
    tile_drive_h_get_vbat_mv: "canonical",
    // raw*5600/255, sampled while driving
    tile_drive_h_get_resonance_hz: "canonical",
    // 10156/raw
    tile_drive_h_set_trigger: "canonical",
    // MODE[2:0]
    tile_drive_h_set_library: "canonical",
    // LIBRARY_SEL / N_ERM_LRA
    tile_drive_h_rtp_start: "canonical",
    tile_drive_h_rtp_write: "canonical",
    // RTP 0x02 encoding
    tile_drive_h_rtp_stop: "canonical",
    tile_drive_h_set_rtp_format: "canonical",
    // CONTROL2[7]/CONTROL3[3]
    tile_drive_h_is_playing: "canonical",
    // GO bit
    tile_drive_h_stop: "canonical",
    // GO=0
    tile_drive_h_standby: "canonical",
    // STANDBY bit
    tile_drive_h_wake: "canonical",
    tile_drive_h_play: "canonical",
    // SEQ + GO (duration nominal)
    tile_drive_h_get_otp_status: "canonical",
    // CONTROL4[2]
    tile_drive_h_set_loop_mode: "canonical",
    // CONTROL3[5]/[0]
    tile_drive_h_set_actuator_params: "canonical",
    tile_drive_h_set_actuator_voltage: "canonical",
    // §7.5.2 Eq 2-5, driver integer math
    tile_drive_h_set_resonance_hz: "canonical",
    // CONTROL1 DRIVE_TIME
    tile_drive_h_set_resonance_params: "canonical",
    // CONTROL2 fields (stored only)
    tile_drive_h_is_calibrated: "canonical",
    // !DIAG_RESULT
    // pass/fail is a proxy (supply in spec, no over-current), not back-EMF
    tile_drive_h_diagnose: "inferred",
    tile_drive_h_calibrate: "inferred",
    // stored for observability; effect on the waveform not modeled
    tile_drive_h_set_audio_params: "inferred",
    tile_drive_h_set_waveform_timing: "inferred",
    tile_drive_h_set_sequence_wait: "inferred",
    // effect durations are nominal, not the ROM library's
    tile_drive_h_play_sequence: "inferred",
    tile_drive_h_play_click: "inferred",
    tile_drive_h_play_double_tap: "inferred",
    tile_drive_h_play_alert: "inferred",
    tile_drive_h_play_buzz: "inferred",
    power: "inferred"
    // base currents from the datasheet; actuator load is an assumed 25 Ω LRA
  },
  // Expire a timed playback: GO drops as the sequencer drains; play_buzz's RTP
  // returns to internal trigger with amplitude 0. Standby cancels GO.
  deriveState: (state, { t }) => {
    const u = {};
    if (state.play_ms > 0 && state.playing_until_ms === 0) {
      u.playing_until_ms = t + state.play_ms;
      u.play_ms = 0;
    } else if (state.playing_until_ms !== 0 && t >= state.playing_until_ms) {
      u.playing_until_ms = 0;
      u.currently_playing = "";
      if (state.mode_reg === MODE_RTP) {
        u.mode_reg = MODE_INTERNAL_TRIG;
        u.rtp_amplitude = 0;
      } else {
        u.go = 0;
      }
    }
    if (state.standby && state.go) u.go = 0;
    return u;
  },
  // Legacy SimulatorPane only: slow cosmetic drift on the physical inputs.
  automatic: ({ t }) => {
    const tSec = t / 1e3;
    return {
      vbat_mv: Math.sin(tSec * (Math.PI / 15)) * 20,
      resonance_hz: Math.sin(tSec * (Math.PI / 22) + 0.7) * 3
    };
  },
  // OUT+ (pad 7) / OUT- (pad 8): drive envelope magnitude.
  padOutputs(state) {
    const s = driveStrength(state);
    return { "7": s, "8": s };
  },
  // Two tile rails (Drive-H-a.json power[]): V+ (pad 10, 1.8-5 V) and V_MOTOR
  // (pad 9, 2.5-5.5 V). Logic: standby 1.9 µA, enabled 0.6 mA; while driving,
  // ~2.9 mA operating plus the actuator load. Output level from the amplitude
  // registers — closed loop references RATED_VOLTAGE (20.71 mV/LSB), open loop
  // OD_CLAMP (21.32 mV/LSB) — so a low RATED_VOLTAGE shows as weak drive.
  power(state, ctx) {
    const vPlus = ctx?.padVoltage["10"] ?? 3300;
    const vMotor = ctx?.padVoltage["9"] ?? state.vbat_mv;
    if (state.standby) {
      return {
        draw_ua: 1.9,
        rails: [
          { name: "V+", role: "supply", v_mv: vPlus, i_ua: 1.9, pads: ["10"], note: "standby" },
          { name: "V_MOTOR", role: "supply", v_mv: vMotor, i_ua: 0, pads: ["9"], note: "idle" }
        ]
      };
    }
    const s = driveStrength(state);
    const fullScaleMv = state.closed_loop ? state.rated_voltage * 20.71 : state.od_clamp * 21.32;
    const outMv = Math.round(s * fullScaleMv);
    const R_ACTUATOR_OHM = 25;
    const loadUa = Math.round(outMv / R_ACTUATOR_OHM * 1e3);
    const logicUa = 600;
    const driveUa = s > 0 ? 2900 + loadUa : 0;
    return {
      draw_ua: logicUa + driveUa,
      rails: [
        { name: "V+", role: "supply", v_mv: vPlus, i_ua: logicUa, pads: ["10"], note: "logic" },
        {
          name: "V_MOTOR",
          role: "supply",
          v_mv: vMotor,
          i_ua: driveUa,
          pads: ["9"],
          note: "H-bridge + actuator (25 \u03A9 load assumed)"
        },
        {
          name: "OUT\xB1",
          role: "output",
          v_mv: outMv,
          i_ua: loadUa,
          pads: ["7", "8"],
          note: state.closed_loop ? "RMS drive from RATED_VOLTAGE" : "drive from OD_CLAMP"
        }
      ]
    };
  }
};
var drive_h_default = sim;
export {
  drive_h_default as default
};
