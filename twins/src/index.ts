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
import senseBp from './sims/sense_bp';
import displayRgbw from './sims/display_rgbw';
import driveA2 from './sims/drive_a_2';
import driveDcH from './sims/drive_dc_h';
import driveH from './sims/drive_h';
import driveP from './sims/drive_p';
import senseTof from './sims/sense_tof';
import senseAcp from './sims/sense_acp';
import senseTC from './sims/sense_t_c';
import senseM3G from './sims/sense_m_3g';
import senseMic from './sims/sense_mic';
import senseCap from './sims/sense_cap';
import senseAdc6 from './sims/sense_adc_6';
import coreL41 from './sims/core_st_l4_1';
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
  senseBp,
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
export const twins: Record<string, AnyTileSim> = {
  'Sense.CAM.P': senseCamP,
  'Sense.HR': senseHr,
  'Sense.I.6P6': senseI6P6,
  'Sense.I.9': senseI9,
  'Sense.BP': senseBp,
  'Display.RGBW': displayRgbw,
  'Drive.A.2': driveA2,
  'Drive.DC.H': driveDcH,
  'Drive.H': driveH,
  'Drive.P': driveP,
  'Sense.TOF': senseTof,
  'Sense.ACP': senseAcp,
  'Sense.T.C': senseTC,
  'Sense.M.3G': senseM3G,
  'Sense.MIC': senseMic,
  'Sense.CAP': senseCap,
  'Sense.ADC.6': senseAdc6,
  'Core.ST.L4.1': coreL41,
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
