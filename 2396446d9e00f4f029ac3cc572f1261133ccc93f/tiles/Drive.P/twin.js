// src/sims/drive_p.ts
var MODE_IDLE = 0;
var MODE_SENSE_FINE = 1;
var MODE_SENSE_COARSE = 2;
var MODE_PLAY_DIRECT = 3;
var MODE_PLAY_FIFO = 4;
var MODE_PLAY_RAM_SYNTH = 5;
var REG_IC_STATUS = 16;
var REG_SENSE_VAL = 24;
var REG_CHIP_ID = 30;
var CHIP_ID = 1921;
var STATE_IDLE = 0;
var STATE_RUNNING = 512;
var STATE_ERROR = 768;
var UVLO_BIT = 8;
var SC_BIT = 4;
var PLAYST_BIT = 1;
var SAMPLE_FS = 2047;
var UVLO_MV = 3e3;
var PRESS_MV = 3e3;
var IQ_SLEEP_RET_UA = 2.4;
var IQ_SLEEP_NORET_UA = 0.6;
var IDLE_UA = 530;
var VPLUS_ACTIVE_UA = 50;
var K_STATIC_UA_PER_V = 33.4;
var K_DYN = 0.287;
var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
var isPlayMode = (m) => m === MODE_PLAY_DIRECT || m === MODE_PLAY_FIFO || m === MODE_PLAY_RAM_SYNTH;
var powered = (s) => !s.sleeping && !s.auto_slept && s.supply_mv >= UVLO_MV;
function sineQ12(phase) {
  const q = phase >> 14 & 3;
  const idx = phase >> 8 & 63;
  const lut = (i) => Math.round(Math.sin(Math.PI / 2 * (i / 64)) * 2047);
  if (q === 0) return lut(idx);
  if (q === 1) return lut(63 - idx);
  if (q === 2) return -lut(idx);
  return -lut(63 - idx);
}
var scale = (sample, pct) => Math.trunc(sample * Math.min(100, pct) / 100);
var pctArg = (v) => Math.min(100, (v ?? 0) & 255);
function driveStrength(s) {
  if (!powered(s) || !isPlayMode(s.mode) || s.fault_inject) return 0;
  return clamp(s.amplitude / SAMPLE_FS, 0, 1);
}
var outFsMv = (s) => s.output_range === 1 ? 13280 : 95e3;
function senseRaw(s) {
  const mv = s.touch_detected ? Math.max(Math.abs(s.sense_mv), PRESS_MV) : s.sense_mv;
  const lsbTenths = s.sense_gain === 1 ? 76 : 545;
  return clamp(Math.round(mv * 10 / lsbTenths), -2048, 2047);
}
function statusWord(s) {
  if (s.supply_mv < UVLO_MV) return STATE_ERROR | UVLO_BIT;
  if (s.fault_inject) return STATE_ERROR | SC_BIT;
  const running = powered(s) && isPlayMode(s.mode);
  const fifoEmpty = s.playing_until_ms === 0 && s.play_ms === 0;
  return (running ? STATE_RUNNING : STATE_IDLE) | (fifoEmpty ? PLAYST_BIT : 0);
}
function readReturn(s) {
  if (s.return_reg === REG_SENSE_VAL) return senseRaw(s) & 4095;
  if (s.return_reg === REG_CHIP_ID) return CHIP_ID;
  return statusWord(s);
}
function setMode(s, mode) {
  if (s.sleeping) return {};
  const m = mode >= MODE_IDLE && mode <= MODE_PLAY_RAM_SYNTH ? mode : MODE_IDLE;
  const sense = m === MODE_SENSE_FINE || m === MODE_SENSE_COARSE;
  return {
    mode: m,
    auto_slept: 0,
    return_reg: sense ? REG_SENSE_VAL : REG_IC_STATUS,
    ...m === MODE_SENSE_FINE ? { sense_gain: 1 } : {},
    ...m === MODE_SENSE_COARSE ? { sense_gain: 0 } : {},
    ...isPlayMode(m) ? {} : { amplitude: 0, currently_playing: "", play_ms: 0, playing_until_ms: 0 }
  };
}
function play(s, label, peak, freq, ms, hold) {
  if (s.sleeping) return {};
  return {
    ...setMode(s, MODE_PLAY_FIFO),
    amplitude: clamp(Math.abs(peak), 0, SAMPLE_FS),
    freq_hz: freq,
    hold_amplitude: clamp(Math.abs(hold), 0, SAMPLE_FS),
    currently_playing: label,
    play_ms: Math.max(1, ms),
    playing_until_ms: 0
  };
}
var clickPeak = (pct) => scale(sineQ12(8 * 2048), pct);
var clickHold = (pct) => scale(sineQ12(15 * 2048), pct);
var sim = {
  tile: "Drive.P",
  // After tile_drive_p_init(): soft reset (RDADDR = CHIP_ID, IDLE), GAINS=1,
  // GAIND=0, RET=0 (retain), TOUT=0, UPI=0 (tile_drive_p.c:148-183).
  defaultState: {
    sense_mv: 0,
    touch_detected: 0,
    fault_inject: 0,
    supply_mv: 3700,
    load_nf: 260,
    // init tunes PARCAP for a 260 nF piezo (tile_drive_p.c:174)
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
    currently_playing: "",
    play_ms: 0,
    playing_until_ms: 0,
    last_fifo_sample: 0,
    last_samples_count: 0,
    last_wfs_count: 0,
    read_counter: 0
  },
  controls: [
    {
      type: "slider",
      field: "sense_mv",
      label: "Sense voltage",
      min: -3300,
      max: 3300,
      step: 10,
      unit: "mV",
      description: "Voltage the piezo presents to the sense channel. read_sense() returns it as a signed 12-bit value; is_touched() compares |mV| to its threshold."
    },
    {
      type: "toggle",
      field: "touch_detected",
      label: "Press",
      description: "A firm press on the piezo: drives the sense pins to at least 3 V."
    },
    {
      type: "toggle",
      field: "fault_inject",
      label: "Fault: output short",
      description: "IC_STATUS reports SC with STATE = ERROR and the output stops; check_and_recover() runs a reset + re-enters the mode."
    },
    {
      type: "slider",
      field: "supply_mv",
      label: "V_DRIVE supply",
      min: 2500,
      max: 5500,
      step: 50,
      unit: "mV",
      description: "Boost supply. Below 3.0 V the chip reports UVLO and does not drive."
    },
    {
      type: "slider",
      field: "load_nf",
      label: "Piezo load",
      min: 1,
      max: 470,
      step: 1,
      unit: "nF",
      description: "Capacitance of the external piezo; the drive current scales with it."
    }
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
        currently_playing: "",
        play_ms: 0,
        playing_until_ms: 0
      }
    }),
    // CONFIG DS=1 (tile_drive_p.c:354-360). Not gated.
    tile_drive_p_sleep: () => ({
      nextState: {
        sleeping: 1,
        mode: MODE_IDLE,
        amplitude: 0,
        currently_playing: "",
        play_ms: 0,
        playing_until_ms: 0
      }
    }),
    // Error STATE or any fault bit → reset + READY + set_mode(restore)
    // (tile_drive_p.c:362-384). The reset restores the shadow defaults.
    tile_drive_p_check_and_recover: ({ state, args }) => {
      const status = statusWord(state);
      const needs = (status & 768) === STATE_ERROR || (status & 252) !== 0;
      if (!needs) return { scalar: 0 };
      const reset = {
        ...state,
        sleeping: 0,
        output_range: 0,
        sense_gain: 1,
        sleep_retention: 1,
        auto_sleep: 0,
        upi: 0
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
          ...setMode(reset, (args[0] ?? 0) & 255)
        }
      };
    },
    // ── mode + raw access ──
    tile_drive_p_set_mode: ({ state, args }) => ({
      nextState: setMode(state, (args[0] ?? 0) & 255)
    }),
    tile_drive_p_read: ({ state }) => ({
      scalar: readReturn(state),
      nextState: { read_counter: state.read_counter + 1 }
    }),
    tile_drive_p_read_sense: ({ state }) => ({
      scalar: senseRaw(state),
      nextState: { return_reg: REG_SENSE_VAL, read_counter: state.read_counter + 1 }
    }),
    tile_drive_p_read_status: ({ state }) => ({
      scalar: statusWord(state),
      nextState: { return_reg: REG_IC_STATUS }
    }),
    // REFERENCE write: in a play mode the sample goes out (and stays there).
    tile_drive_p_write_fifo: ({ args }) => {
      const raw = (args[0] ?? 0) & 65535;
      const sample = raw > 32767 ? raw - 65536 : raw;
      return {
        nextState: {
          last_fifo_sample: sample,
          amplitude: clamp(Math.abs(sample), 0, SAMPLE_FS),
          auto_slept: 0
        }
      };
    },
    // Up to 8 WFS words to REFERENCE (tile_drive_p.c:342-352); RAM synth not modeled.
    tile_drive_p_wfs_write: ({ bufferIn }) => ({
      nextState: { last_wfs_count: Math.min(8, (bufferIn?.words ?? []).length) }
    }),
    // ── config setters ──
    // GAIND / GAINS land with an IDLE CONFIG write when READY, which clears OE
    // (tile_drive_p.c:390-419) — a play mode stops.
    tile_drive_p_set_output_range: ({ state, args }) => ({
      nextState: {
        output_range: args[0] === 1 ? 1 : 0,
        ...state.sleeping ? {} : { ...setMode(state, MODE_IDLE), return_reg: state.return_reg }
      }
    }),
    tile_drive_p_set_sense_gain: ({ state, args }) => ({
      nextState: {
        sense_gain: args[0] === 1 ? 1 : 0,
        ...state.sleeping ? {} : { ...setMode(state, MODE_IDLE), return_reg: state.return_reg }
      }
    }),
    tile_drive_p_set_sleep_retention: ({ args }) => ({
      nextState: { sleep_retention: args[0] ? 1 : 0 }
    }),
    // COMM rewrite points RDADDR at IC_STATUS (tile_drive_p.c:438-456).
    tile_drive_p_set_auto_sleep: ({ args }) => ({
      nextState: { auto_sleep: args[0] ? 1 : 0, return_reg: REG_IC_STATUS }
    }),
    tile_drive_p_set_upi: ({ args }) => ({ nextState: { upi: args[0] ? 1 : 0 } }),
    // ── playback helpers ──
    tile_drive_p_play_click: ({ state, args }) => {
      const pct = pctArg(args[0]);
      return {
        nextState: play(state, `click @ ${pct}%`, clickPeak(pct), 0, 2, clickHold(pct))
      };
    },
    // FIFO sine at 8 ksps for `ms` (tile_drive_p.c:537-566); no-op on 0 Hz / 0 ms.
    tile_drive_p_play_sine: ({ state, args }) => {
      const freq = (args[0] ?? 0) & 65535;
      const pct = pctArg(args[1]);
      const ms = (args[2] ?? 0) & 65535;
      if (freq === 0 || ms === 0) return {};
      const step = Math.floor(freq * 65536 / 8e3);
      const last = scale(sineQ12((ms * 8 - 1) * step & 65535), pct);
      return {
        nextState: play(
          state,
          `sine ${freq} Hz @ ${pct}% / ${ms} ms`,
          scale(2046, pct),
          freq,
          ms,
          last
        )
      };
    },
    // play_sine at 150 Hz (tile_drive_p.c:568-572).
    tile_drive_p_play_buzz: ({ state, args }) => {
      const pct = pctArg(args[0]);
      const ms = (args[1] ?? 0) & 65535;
      if (ms === 0) return {};
      const step = Math.floor(150 * 65536 / 8e3);
      const last = scale(sineQ12((ms * 8 - 1) * step & 65535), pct);
      return {
        nextState: play(state, `buzz @ ${pct}% / ${ms} ms`, scale(2046, pct), 150, ms, last)
      };
    },
    tile_drive_p_play_pulse_train: ({ state, args }) => {
      const pct = pctArg(args[0]);
      const count = (args[1] ?? 0) & 255;
      const gap = (args[2] ?? 0) & 65535;
      if (count === 0) return {};
      return {
        nextState: play(
          state,
          `pulse train ${count}\xD7 @ ${pct}%, ${gap} ms gap`,
          clickPeak(pct),
          count > 1 ? Math.round(1e3 / (2 + gap)) : 0,
          count * 2 + (count - 1) * gap,
          clickHold(pct)
        )
      };
    },
    // set_mode(SENSE_FINE) + read_sense, |raw·7.6 mV| ≥ threshold
    // (tile_drive_p.c:585-596).
    tile_drive_p_is_touched: ({ state, args }) => {
      const threshold = (args[0] ?? 0) & 65535;
      const next = { ...state, ...setMode(state, MODE_SENSE_FINE) };
      const mv = Math.abs(Math.trunc(senseRaw(next) * 76 / 10));
      return {
        scalar: mv >= threshold ? 1 : 0,
        nextState: {
          ...setMode(state, MODE_SENSE_FINE),
          return_reg: REG_SENSE_VAL,
          read_counter: state.read_counter + 1
        }
      };
    },
    // Polls is_touched every 1 ms up to timeout_ms, then clicks
    // (tile_drive_p.c:598-612). Collapsed to one look at the present state.
    tile_drive_p_play_on_touch: ({ state, args }) => {
      const pct = pctArg(args[0]);
      const threshold = (args[1] ?? 0) & 65535;
      const timeout = args[2] ?? 0;
      if (timeout <= 0) return { scalar: 0 };
      const sensing = { ...state, ...setMode(state, MODE_SENSE_FINE) };
      const mv = Math.abs(Math.trunc(senseRaw(sensing) * 76 / 10));
      if (mv < threshold) {
        return {
          scalar: 0,
          nextState: { ...setMode(state, MODE_SENSE_FINE), return_reg: REG_SENSE_VAL }
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
          clickHold(pct)
        )
      };
    },
    // args = [samples pointer, count]: the manifest has no in-buffer for
    // `samples`, so only the length is known — the envelope is assumed full scale.
    tile_drive_p_play_samples: ({ state, args }) => {
      const count = (args[1] ?? 0) & 65535;
      if (count === 0) return {};
      return {
        nextState: {
          ...play(state, `samples \xD7${count}`, SAMPLE_FS, 0, Math.ceil(count / 8), 0),
          last_samples_count: count
        }
      };
    },
    // No out-buffer in the manifest: only the mode switch is observable.
    tile_drive_p_read_sense_samples: ({ state }) => ({
      nextState: { ...setMode(state, MODE_SENSE_FINE), return_reg: REG_SENSE_VAL }
    })
  },
  provenance: {
    tile_drive_p_find: "canonical",
    tile_drive_p_read_sense: "canonical",
    // signed 12-bit, 7.6 / 54.5 mV LSB
    tile_drive_p_read_status: "canonical",
    // IC_STATUS STATE / fault / PLAYST
    tile_drive_p_read: "canonical",
    // COMM.RDADDR return register
    tile_drive_p_set_mode: "canonical",
    // CONFIG per mode, RDADDR, GAINS
    tile_drive_p_set_output_range: "canonical",
    // GAIND, clears OE
    tile_drive_p_set_sense_gain: "canonical",
    // GAINS, clears OE
    tile_drive_p_write_fifo: "canonical",
    // REFERENCE; last sample holds
    tile_drive_p_is_touched: "canonical",
    // 7.6 mV/LSB threshold compare
    tile_drive_p_play_click: "canonical",
    // driver's 16-sample half-sine
    tile_drive_p_play_sine: "canonical",
    // driver's Q12 phase accumulator
    tile_drive_p_play_buzz: "canonical",
    tile_drive_p_play_pulse_train: "inferred",
    tile_drive_p_play_on_touch: "inferred",
    // polling loop collapsed to one look
    tile_drive_p_check_and_recover: "inferred",
    tile_drive_p_sleep: "inferred",
    tile_drive_p_set_sleep_retention: "inferred",
    tile_drive_p_set_auto_sleep: "inferred",
    tile_drive_p_set_upi: "inferred",
    tile_drive_p_wfs_write: "hallucinated",
    // RAM synthesis not modeled
    tile_drive_p_play_samples: "hallucinated",
    // sample values not visible
    tile_drive_p_read_sense_samples: "hallucinated",
    // buffer not filled
    power: "inferred"
    // CV²f + HV-hold fit to datasheet Table 7; loss factor inferred
  },
  // Playback end: the FIFO drains and the last sample holds on the output
  // (freq 0 = DC), or with COMM.TOUT the chip sleeps.
  deriveState: (state, { t }) => {
    if (state.play_ms > 0 && state.playing_until_ms === 0) {
      return { playing_until_ms: t + state.play_ms, play_ms: 0 };
    }
    if (state.playing_until_ms !== 0 && t >= state.playing_until_ms) {
      const tout = state.auto_sleep && (state.mode === MODE_PLAY_FIFO || state.mode === MODE_PLAY_DIRECT);
      return {
        playing_until_ms: 0,
        currently_playing: "",
        freq_hz: 0,
        amplitude: tout ? 0 : state.hold_amplitude,
        auto_slept: tout ? 1 : 0
      };
    }
    return {};
  },
  padOutputs(state) {
    const s = driveStrength(state);
    return { "7": s, "8": s };
  },
  // V+ logic (pad 10) + V_DRIVE boost (pad 9); boosted differential drive on
  // OUT± (pads 7/8). V_DRIVE draw responds to amplitude, tone frequency, output
  // range, piezo load and supply voltage.
  power(state, ctx) {
    const supplyMv = ctx?.padVoltage["9"] ?? state.supply_mv;
    const vPlus = ctx?.padVoltage["10"] ?? 3300;
    const s = driveStrength(state);
    let driveUa;
    if (state.sleeping || state.auto_slept) {
      driveUa = state.sleep_retention ? IQ_SLEEP_RET_UA : IQ_SLEEP_NORET_UA;
    } else if (supplyMv < UVLO_MV) {
      driveUa = 0;
    } else if (s <= 0) {
      driveUa = IDLE_UA;
    } else {
      const vpk = s * outFsMv(state) / 1e3;
      const vpp = 2 * vpk;
      const c = Math.max(1, state.load_nf) * 1e-9;
      const staticUa = K_STATIC_UA_PER_V * vpk;
      const dynUa = K_DYN * c * vpp * vpp * state.freq_hz / (supplyMv / 1e3) * 1e6;
      driveUa = IDLE_UA + staticUa + dynUa;
    }
    const vplusUa = powered(state) ? VPLUS_ACTIVE_UA : 0;
    return {
      draw_ua: Math.round(driveUa + vplusUa),
      rails: [
        { name: "V+", role: "supply", v_mv: vPlus, i_ua: vplusUa, pads: ["10"], note: "logic/IO" },
        {
          name: "V_DRIVE",
          role: "supply",
          v_mv: supplyMv,
          i_ua: Math.round(driveUa),
          pads: ["9"],
          note: state.sleeping || state.auto_slept ? "boost supply \u2014 sleep" : s > 0 ? "boost supply + piezo drive" : "boost supply \u2014 idle"
        },
        {
          name: "OUT\xB1",
          role: "output",
          v_mv: Math.round(s * outFsMv(state)),
          pads: ["7", "8"],
          note: "boosted differential piezo drive (190 Vpp full scale)"
        }
      ]
    };
  }
};
var drive_p_default = sim;
export {
  drive_p_default as default
};
