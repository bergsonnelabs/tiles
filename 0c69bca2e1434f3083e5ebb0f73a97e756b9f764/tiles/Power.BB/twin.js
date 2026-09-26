// src/sims/power_bb.ts
var UVLO_MV = 1750;
var VOUT_MV = 5e3;
var BB_IQ_UA = 0.075;
var LDO_IQ_UA = 6.5;
var LDO_MAX_UA = 3e5;
var EFF_5V = [
  [1800, 0.81],
  [2500, 0.85],
  [3300, 0.88],
  [5e3, 0.94]
];
var CAP_5V = [
  [1800, 350],
  [2800, 520],
  [3300, 600],
  [3800, 630],
  [4300, 750],
  [4800, 930],
  [5300, 1100]
];
function lerp(table, vin) {
  if (vin <= table[0][0]) return table[0][1];
  for (let k = 1; k < table.length; k++) {
    const [v1, y1] = table[k];
    if (vin <= v1) {
      const [v0, y0] = table[k - 1];
      return y0 + (y1 - y0) * (vin - v0) / (v1 - v0);
    }
  }
  return table[table.length - 1][1];
}
var capUa = (vin) => Math.round(lerp(CAP_5V, vin) * 1e3);
var effAt = (vin) => Math.round(lerp(EFF_5V, vin) * 100) / 100;
var vinOf = (s, ctx) => {
  const p = ctx?.padVoltage?.["3"];
  return p != null ? p : s.vin_mv || 0;
};
var sim = {
  tile: "Power.BB",
  defaultState: { vin_mv: 3700 },
  // Only knob is the fallback input voltage (used when pad 3 isn't wired to an
  // upstream source). EN is hardware-strapped on, so there's nothing to toggle.
  controls: [
    {
      type: "slider",
      field: "vin_mv",
      label: "VIN (if unwired)",
      min: 0,
      max: 5500,
      step: 50,
      unit: "mV"
    }
  ],
  // Regulated rails on/off (1/0): 5V0 (pad 8), 3V3 (pad 9), 1V8 (pad 10).
  // padOutputs gets no electrical ctx, so it follows the fallback VIN control.
  padOutputs(state) {
    const on = (state.vin_mv || 0) >= UVLO_MV ? 1 : 0;
    return { "8": on, "9": on, "10": on };
  },
  // Electrical. VIN omits i_ua so the solver back-fills it from the downstream
  // loads. The rails say how: 1V8 is fed from 3V3 and 3V3 from 5V0, each LDO
  // passing its current straight through (`linear`), so every load reaches the
  // buck-boost as current at 5 V; the buck-boost (`switching`) turns that into
  // VIN current at its Fig. 7-4 efficiency. The same chain puts the LDO loads
  // under the 5V0 limit. draw_ua is the tile's own no-load draw, referred to VIN
  // (the LDOs' ground current comes off 5V0 through the buck-boost).
  power(state, ctx) {
    const vin = vinOf(state, ctx);
    const on = vin >= UVLO_MV;
    const eff = effAt(vin);
    const own = on ? Math.round((BB_IQ_UA + 2 * LDO_IQ_UA * VOUT_MV / (vin * eff)) * 1e3) / 1e3 : 0.06;
    return {
      draw_ua: own,
      rails: [
        {
          name: "VIN",
          role: "supply",
          v_mv: vin,
          pads: ["3"],
          note: "buck-boost input (solver back-fills load)"
        },
        {
          name: "5V0",
          role: "output",
          v_mv: on ? VOUT_MV : 0,
          pads: ["8"],
          limit_ua: on ? capUa(vin) : void 0,
          conversion: "switching",
          efficiency: eff,
          note: "TPS63901 buck-boost output (typ capability at this VIN, Fig. 7-5)"
        },
        {
          name: "3V3",
          role: "output",
          v_mv: on ? 3300 : 0,
          pads: ["9"],
          limit_ua: LDO_MAX_UA,
          from: "5V0",
          conversion: "linear",
          note: "TPS7A20 LDO from 5V0 (also carries the 1V8 load)"
        },
        {
          name: "1V8",
          role: "output",
          v_mv: on ? 1800 : 0,
          pads: ["10"],
          limit_ua: LDO_MAX_UA,
          from: "3V3",
          conversion: "linear",
          note: "TPS7A20 LDO from 3V3"
        }
      ]
    };
  },
  provenance: {
    power: "inferred"
    // rails/topology/IQ canonical (schematic + datasheets); VIN load back-filled by the solver
  }
};
var power_bb_default = sim;
export {
  power_bb_default as default
};
