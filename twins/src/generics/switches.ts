// Generic SPST switch twin — a NON-GENDERED single connection (A1 ↔ B1) you wire
// inline between two parts. Modeled as a symmetric power transform: whichever
// contact is driven is the input (an undeclared-current supply, so the upstream
// source delivers the load *through* the switch via the solver's current-
// balance); the other carries that voltage while closed, 0 while open. Either
// side may be the source. A signal-type SPST carries no power, so its contacts
// sit at 0 and the twin has no electrical effect — it's just the position.
import type { SimState, TileSim } from '../tileSim';

export interface PoleState extends SimState {
  /** 1 = closed (conducting), 0 = open. */
  closed: number;
}

export const spst: TileSim<PoleState> = {
  tile: 'Generic.SPST',
  defaultState: { closed: 1 },
  controls: [{ type: 'toggle', field: 'closed', label: 'Closed' }],
  power: (s, ctx) => {
    const va = ctx?.padVoltage?.['A1'] ?? 0;
    const vb = ctx?.padVoltage?.['B1'] ?? 0;
    const closed = (s.closed ?? 1) !== 0;
    // Whichever contact sees the higher voltage is the driven (supply) side.
    const [supplyPad, outPad, driven] = va >= vb ? ['A1', 'B1', va] : ['B1', 'A1', vb];
    return {
      draw_ua: 0,
      rails: [
        // Undeclared current on the driven side → the solver fills it from the
        // out-net load, so the source delivers the system through a closed switch.
        { name: supplyPad, role: 'supply', v_mv: driven, pads: [supplyPad], note: 'contact' },
        {
          name: outPad,
          role: 'output',
          v_mv: closed ? driven : 0,
          pads: [outPad],
          note: closed ? 'closed' : 'open',
        },
      ],
    };
  },
  provenance: { power: 'inferred' },
};

export const switches = [spst];
