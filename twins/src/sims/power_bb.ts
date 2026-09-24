// Digital twin for Power.BB: multi-output buck-boost regulator. DRIVERLESS:
// the tile has no comms interface, so there are no host calls. It's a pure power
// transform, modeled entirely by the electrical layer.
//
// Topology (tile schematic): VIN (pad 3) feeds a TPS63901 buck-boost → 5V0
// (pad 8). 5V0 feeds a TPS7A20 LDO → 3V3 (pad 9), and 3V3 feeds a second
// TPS7A20 → 1V8 (pad 10): the LDOs are CASCADED, so the 1V8 load also flows
// through the 3V3 LDO. EN is pulled to VIN through R1 = 1 MΩ (always on). SEL,
// CFG1, CFG2 are grounded (VO(2) unused, input current limit UNLIMITED) and
// CFG3 = 36.5 kΩ (R2) sets VO(1) = 5.0 V (TPS63901 Table 6-3). GND on pad 1;
// pads 2/4/5/6/7 are unconnected.
//
// Numbers: TPS63901 (SLVSGC1B) IQ 75 nA, UVLO 1.75 V rising / 1.65 V falling,
// output capability at VO = 5 V from Fig. 7-5, efficiency at VO = 5 V from
// Fig. 7-4. TPS7A20 (SBVS338H) IGND 6.5 µA each, 300 mA max output.
// VIN current is left to the solver to back-fill from what the three rails feed
// downstream, so power is inferred (see note on power()).
import type { PowerCtx, TileSim } from '../tileSim';

const UVLO_MV = 1750; // TPS63901 VIT+(UVLO) typ
const VOUT_MV = 5000; // VO(1), CFG3 = 36.5 kΩ
const BB_IQ_UA = 0.075; // TPS63901 IQ into VIN (not switching)
const LDO_IQ_UA = 6.5; // TPS7A20 IGND at IOUT = 0 (each, drawn from its input)
const LDO_MAX_UA = 300_000; // TPS7A20 rated output current

// Efficiency at VO = 5 V vs VIN, read off Fig. 7-4 on its flat stretch (about
// 1 µA to 100 mA); it falls off above ~200 mA, which this does not model.
const EFF_5V: ReadonlyArray<readonly [number, number]> = [
  [1800, 0.81],
  [2500, 0.85],
  [3300, 0.88],
  [5000, 0.94],
];

// Typical 5 V output-current capability vs VIN (mA), read off Fig. 7-5 (VO = 5.0 V).
const CAP_5V: ReadonlyArray<readonly [number, number]> = [
  [1800, 350],
  [2800, 520],
  [3300, 600],
  [3800, 630],
  [4300, 750],
  [4800, 930],
  [5300, 1100],
];
/** Piecewise-linear lookup in a [vin, value] table, clamped at both ends. */
function lerp(table: ReadonlyArray<readonly [number, number]>, vin: number): number {
  if (vin <= table[0][0]) return table[0][1];
  for (let k = 1; k < table.length; k++) {
    const [v1, y1] = table[k];
    if (vin <= v1) {
      const [v0, y0] = table[k - 1];
      return y0 + ((y1 - y0) * (vin - v0)) / (v1 - v0);
    }
  }
  return table[table.length - 1][1];
}
const capUa = (vin: number) => Math.round(lerp(CAP_5V, vin) * 1000);
const effAt = (vin: number) => Math.round(lerp(EFF_5V, vin) * 100) / 100;

interface State {
  vin_mv: number; // fallback input voltage when pad 3 isn't wired
  [field: string]: number;
}

// VIN comes from the WIRING (pad 3) when available, else the fallback control,
// same pattern as the PMIC twins, so an upstream battery/USB tile drives it.
const vinOf = (s: State, ctx?: PowerCtx) => {
  const p = ctx?.padVoltage?.['3'];
  return p != null ? p : s.vin_mv || 0;
};

const sim: TileSim<State> = {
  tile: 'Power.BB',

  defaultState: { vin_mv: 3700 },

  // Only knob is the fallback input voltage (used when pad 3 isn't wired to an
  // upstream source). EN is hardware-strapped on, so there's nothing to toggle.
  controls: [
    {
      type: 'slider',
      field: 'vin_mv',
      label: 'VIN (if unwired)',
      min: 0,
      max: 5500,
      step: 50,
      unit: 'mV',
    },
  ],

  // Regulated rails on/off (1/0): 5V0 (pad 8), 3V3 (pad 9), 1V8 (pad 10).
  // padOutputs gets no electrical ctx, so it follows the fallback VIN control.
  padOutputs(state) {
    const on = (state.vin_mv || 0) >= UVLO_MV ? 1 : 0;
    return { '8': on, '9': on, '10': on };
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
    const own = on
      ? Math.round((BB_IQ_UA + (2 * LDO_IQ_UA * VOUT_MV) / (vin * eff)) * 1000) / 1000
      : 0.06; // TPS63901 below UVLO ≈ shutdown current (60 nA)
    return {
      draw_ua: own,
      rails: [
        {
          name: 'VIN',
          role: 'supply',
          v_mv: vin,
          pads: ['3'],
          note: 'buck-boost input (solver back-fills load)',
        },
        {
          name: '5V0',
          role: 'output',
          v_mv: on ? VOUT_MV : 0,
          pads: ['8'],
          limit_ua: on ? capUa(vin) : undefined,
          conversion: 'switching',
          efficiency: eff,
          note: 'TPS63901 buck-boost output (typ capability at this VIN, Fig. 7-5)',
        },
        {
          name: '3V3',
          role: 'output',
          v_mv: on ? 3300 : 0,
          pads: ['9'],
          limit_ua: LDO_MAX_UA,
          from: '5V0',
          conversion: 'linear',
          note: 'TPS7A20 LDO from 5V0 (also carries the 1V8 load)',
        },
        {
          name: '1V8',
          role: 'output',
          v_mv: on ? 1800 : 0,
          pads: ['10'],
          limit_ua: LDO_MAX_UA,
          from: '3V3',
          conversion: 'linear',
          note: 'TPS7A20 LDO from 3V3',
        },
      ],
    };
  },

  provenance: {
    power: 'inferred', // rails/topology/IQ canonical (schematic + datasheets); VIN load back-filled by the solver
  },
};

export default sim;
