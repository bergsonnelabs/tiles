// @bergsonnelabs/tile-twins — shared tile digital-twin models.
//
// Re-exports the TileSim contract and every authored twin, plus a `twins`
// registry keyed by tile name (matches TileSim.tile). Consumed by the studio
// simulator and the portal twin editor/runner.

export * from './tileSim';
import type { TileSim } from './tileSim';

import senseCamP from './sims/sense_cam_p';
import senseHr from './sims/sense_hr';
import senseI6P6 from './sims/sense_i_6p6';
import senseI9 from './sims/sense_i_9';
import displayRgbw from './sims/display_rgbw';
import driveA2 from './sims/drive_a_2';
import driveDcH from './sims/drive_dc_h';
import driveH from './sims/drive_h';
import driveP from './sims/drive_p';
import senseTof from './sims/sense_tof';
import senseTC from './sims/sense_t_c';
import senseM3G from './sims/sense_m_3g';
import senseMic from './sims/sense_mic';
import senseCap from './sims/sense_cap';
import coreL4 from './sims/core_st_l4';
import coreW5 from './sims/core_st_w5';
import powerL1N from './sims/power_l_1n';
import powerL1T from './sims/power_l_1t';
import powerBB from './sims/power_bb';
import storeO128 from './sims/store_o_128';
import { sources } from './generics/sources';
import { loads } from './generics/loads';
import { switches } from './generics/switches';

export {
  senseCamP,
  senseHr,
  senseI6P6,
  senseI9,
  displayRgbw,
  driveA2,
  driveDcH,
  driveH,
  driveP,
  senseTof,
  powerL1N,
  powerL1T,
  powerBB,
  storeO128,
};
export * from './generics/sources';
export * from './generics/loads';
export * from './generics/switches';

// A twin with its concrete State erased — the registry is heterogeneous, so it
// holds twins over any State (each sim keeps its precise type at its own export).
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type AnyTileSim = TileSim<any>;

// tile name (matches TileSim.tile) → twin model. Driver tiles + passive generics
// (battery, USB, …) share one registry; consumers don't care which is which.
// Public tiles only: a tile that is not public keeps its twin in its private
// overlay repo (twins/src/sims/), which the bundle builder picks up
// (scripts/build-bundle.mjs --overlay).
export const twins: Record<string, AnyTileSim> = {
  'Sense.CAM.P': senseCamP,
  'Sense.HR': senseHr,
  'Sense.I.6P6': senseI6P6,
  'Sense.I.9': senseI9,
  'Display.RGBW': displayRgbw,
  'Drive.A.2': driveA2,
  'Drive.DC.H': driveDcH,
  'Drive.H': driveH,
  'Drive.P': driveP,
  'Sense.TOF': senseTof,
  'Sense.T.C': senseTC,
  'Sense.M.3G': senseM3G,
  'Sense.MIC': senseMic,
  'Sense.CAP': senseCap,
  'Core.ST.L4': coreL4,
  // Pre-2026-09 name of the same board (not the Core.ST.L4.2), kept so a
  // project saved under it still finds its twin.
  'Core.ST.L4.1': coreL4,
  'Core.ST.W5': coreW5,
  'Power.L.1N': powerL1N,
  'Power.L.1T': powerL1T,
  'Power.BB': powerBB,
  'Store.O.128': storeO128,
  ...Object.fromEntries(sources.map((s) => [s.tile, s])),
  ...Object.fromEntries(loads.map((s) => [s.tile, s])),
  ...Object.fromEntries(switches.map((s) => [s.tile, s])),
};

// Look up a twin by tile name (e.g. "Drive.H").
export function getTwin(tile: string): AnyTileSim | undefined {
  return twins[tile];
}
