// src/sims/sense_acp.ts
var ST_PSAT = 64;
var ST_ASAT = 128;
var ALS_GAIN_MUL = [1, 4, 16, 64];
var ALS_GAIN_REF = 16;
var ATIME_REF = 47;
var PROX_GAIN_MUL = [1, 2, 4, 8];
var PROX_GAIN_REF = 4;
var PROX_DRIVE_REF = 12;
var clamp = (v, lo, hi) => v < lo ? lo : v > hi ? hi : v;
var pick = (args, i, cur) => args.length > i && Number.isFinite(args[i]) ? args[i] : cur;
var alsMax = (s) => Math.min(65535, 1024 * ((s.atime & 255) + 1));
function alsCount(base, s) {
  if (!s.als_on) return 0;
  const gain = ALS_GAIN_MUL[s.als_gain & 3] / ALS_GAIN_REF;
  const time = (s.atime + 1) / (ATIME_REF + 1);
  return clamp(Math.round(base * gain * time), 0, alsMax(s));
}
function proxCount(s) {
  if (!s.prox_on) return 0;
  const gain = PROX_GAIN_MUL[s.prox_gain & 3] / PROX_GAIN_REF;
  const drive = s.prox_drive_ma / PROX_DRIVE_REF;
  return clamp(Math.round(s.proximity * gain * drive), 0, 255);
}
function statusByte(s) {
  let status = 0;
  const max = alsMax(s);
  const alsSat = s.als_on === 1 && (alsCount(s.clear, s) >= max || alsCount(s.red, s) >= max || alsCount(s.green, s) >= max || alsCount(s.blue, s) >= max);
  if (alsSat) status |= ST_ASAT;
  if (s.prox_on === 1 && proxCount(s) >= 255) status |= ST_PSAT;
  return status;
}
function quantiseDriveMa(ma) {
  const steps = clamp(Math.floor(clamp(ma, 6, 192) / 6), 1, 32);
  return steps * 6;
}
var sim = {
  tile: "Sense.ACP",
  defaultState: {
    // White-ish ambient: R/G/B ratios near the datasheet's 2700 K white-LED
    // figures (R/C ~60%, G/C ~35%, B/C ~22%).
    clear: 8e3,
    red: 4800,
    green: 2800,
    blue: 1760,
    proximity: 30,
    als_gain: 2,
    // 16x
    atime: 47,
    // ~133 ms
    prox_gain: 2,
    // 4x
    prox_drive_ma: 12,
    als_on: 1,
    prox_on: 1
  },
  controls: [
    { type: "slider", field: "clear", label: "Ambient (clear)", min: 0, max: 65535, step: 100 },
    { type: "slider", field: "red", label: "Red", min: 0, max: 65535, step: 100 },
    { type: "slider", field: "green", label: "Green", min: 0, max: 65535, step: 100 },
    { type: "slider", field: "blue", label: "Blue", min: 0, max: 65535, step: 100 },
    {
      type: "slider",
      field: "proximity",
      label: "Proximity (near\u2192high)",
      min: 0,
      max: 255,
      step: 1
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    // 1 when the ID register matches (tile_sense_acp.c:82), not the address
    tile_sense_acp_find: () => ({ scalar: 1 }),
    tile_sense_acp_init: () => ({
      nextState: {
        als_gain: 2,
        atime: 47,
        prox_gain: 2,
        prox_drive_ma: 12,
        als_on: 1,
        prox_on: 1
      }
    }),
    tile_sense_acp_sleep: () => ({ nextState: { als_on: 0, prox_on: 0 } }),
    tile_sense_acp_wake: () => ({ nextState: { als_on: 1, prox_on: 1 } }),
    // ── configuration ──
    tile_sense_acp_set_als_gain: ({ state, args }) => ({
      nextState: { als_gain: pick(args, 0, state.als_gain) & 3 }
    }),
    tile_sense_acp_set_integration_time: ({ state, args }) => ({
      nextState: { atime: pick(args, 0, state.atime) & 255 }
    }),
    tile_sense_acp_set_prox_gain: ({ state, args }) => ({
      nextState: { prox_gain: pick(args, 0, state.prox_gain) & 3 }
    }),
    tile_sense_acp_set_prox_drive_ma: ({ state, args }) => ({
      nextState: { prox_drive_ma: quantiseDriveMa(pick(args, 0, state.prox_drive_ma)) }
    }),
    // ── ambient light / colour ──
    tile_sense_acp_get_clear: ({ state }) => ({ scalar: alsCount(state.clear, state) }),
    tile_sense_acp_get_red: ({ state }) => ({ scalar: alsCount(state.red, state) }),
    tile_sense_acp_get_green: ({ state }) => ({ scalar: alsCount(state.green, state) }),
    tile_sense_acp_get_blue: ({ state }) => ({ scalar: alsCount(state.blue, state) }),
    // ── proximity ──
    tile_sense_acp_get_proximity: ({ state }) => ({ scalar: proxCount(state) }),
    // ── status ──
    tile_sense_acp_get_status: ({ state }) => ({ scalar: statusByte(state) })
  },
  provenance: {
    tile_sense_acp_find: "canonical",
    // I2C 0x39, ID reg 0xE4
    tile_sense_acp_set_als_gain: "canonical",
    // CFG1 AGAIN
    tile_sense_acp_set_integration_time: "canonical",
    // ATIME
    tile_sense_acp_set_prox_gain: "canonical",
    // PCFG1 PGAIN
    tile_sense_acp_set_prox_drive_ma: "canonical",
    // PCFG1 PLDRIVE, 6 mA steps
    // getters return modelled counts (gain/integration scaling is
    // datasheet-directional, but the base light is a control, not a
    // photodiode model)
    tile_sense_acp_get_clear: "inferred",
    tile_sense_acp_get_red: "inferred",
    tile_sense_acp_get_green: "inferred",
    tile_sense_acp_get_blue: "inferred",
    tile_sense_acp_get_proximity: "inferred",
    tile_sense_acp_get_status: "inferred",
    // modelled saturation flags
    power: "inferred"
    // estimated supply + boosted LED average
    // (init/sleep/wake default to "inferred")
  },
  // Supply on the 1.8 V input (pad 10). Sleep is near-zero; when running,
  // the ALS front-end plus the proximity analog path draw quiescent current,
  // and the proximity IR LED adds a boosted average on top (TPS61099 stepping
  // 1.8 V up to the LED rail). All figures are estimates → provenance inferred.
  power(state) {
    const running = state.als_on === 1 || state.prox_on === 1;
    if (!running) {
      return {
        draw_ua: 3,
        rails: [{ name: "1V8", role: "supply", v_mv: 1800, i_ua: 3, pads: ["10"], note: "sleep" }]
      };
    }
    const oscillator = 10;
    const alsPart = state.als_on ? 240 : 0;
    const proxSensor = state.prox_on ? 90 : 0;
    const proxLed = state.prox_on ? Math.round(state.prox_drive_ma * 130) : 0;
    const draw = oscillator + alsPart + proxSensor + proxLed;
    return {
      draw_ua: draw,
      rails: [
        {
          name: "1V8",
          role: "supply",
          v_mv: 1800,
          i_ua: draw,
          pads: ["10"],
          note: state.prox_on ? "active (incl. boosted prox LED avg)" : "ALS active"
        }
      ]
    };
  }
};
var sense_acp_default = sim;
export {
  sense_acp_default as default
};
