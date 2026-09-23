// Digital twin for Power.BB — multi-output buck-boost regulator. DRIVERLESS:
// the tile has no comms interface, so there are no host calls — it's a pure power
// transform, modeled entirely by the electrical layer.
//
// Topology (from the tile schematic; the tile JSON is empty): VIN (pad 3) feeds a
// TPS63901 buck-boost → 5V0 (pad 8), which feeds two TPS7A20 LDOs → 3V3 (pad 9)
// and 1V8 (pad 10). EN is strapped to VIN (always-on). GND on pad 1; pads
// 2/4/5/6/7 are unconnected. So one wide-range input becomes three regulated
// rails.
//
// Output voltages + topology are canonical (schematic); quiescent is datasheet
// (TPS63901 ~75 µA + 2× TPS7A20 ~5 µA). VIN current is left to the solver to
// back-fill from whatever the three rails feed downstream, so power is inferred.
import type { PowerCtx, TileSim } from '../tileSim';

const UVLO_MV = 1800; // buck-boost needs at least this on VIN to hold the rails up
const BB_IQ_UA = 75; // TPS63901 quiescent
const LDO_IQ_UA = 5; // TPS7A20 quiescent (×2)
const QUIESCENT_UA = BB_IQ_UA + 2 * LDO_IQ_UA;

interface State {
  vin_mv: number; // fallback input voltage when pad 3 isn't wired
  [field: string]: number;
}

// VIN comes from the WIRING (pad 3) when available, else the fallback control —
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
  padOutputs(state) {
    const on = (state.vin_mv || 0) >= UVLO_MV ? 1 : 0;
    return { '8': on, '9': on, '10': on };
  },

  // Electrical. VIN (pad 3) → buck-boost 5V0 (pad 8) → LDOs 3V3 (pad 9) / 1V8
  // (pad 10). VIN omits i_ua so the solver back-fills it from the downstream
  // loads on the three output nets; draw_ua is the tile's own quiescent floor.
  power(state, ctx) {
    const vin = vinOf(state, ctx);
    const on = vin >= UVLO_MV;
    return {
      draw_ua: QUIESCENT_UA,
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
          v_mv: on ? 5000 : 0,
          pads: ['8'],
          note: 'TPS63901 buck-boost output',
        },
        {
          name: '3V3',
          role: 'output',
          v_mv: on ? 3300 : 0,
          pads: ['9'],
          note: 'TPS7A20 LDO from 5V0',
        },
        {
          name: '1V8',
          role: 'output',
          v_mv: on ? 1800 : 0,
          pads: ['10'],
          note: 'TPS7A20 LDO from 5V0',
        },
      ],
    };
  },

  provenance: {
    power: 'inferred', // rail voltages/topology canonical; VIN load back-filled by the solver
  },
};

export default sim;
