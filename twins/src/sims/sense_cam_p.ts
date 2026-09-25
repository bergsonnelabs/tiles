// Digital twin for Sense.CAM.P — PixArt PAG7920J3 320 x 240 monochrome
// global-shutter camera (driver tile_sense_cam_p.{h,c}; PAG7920J3 datasheet
// V0.8, L1033EN).
//
// Scope: the twin models what the driver's calls RETURN, not the image. The
// program-facing calls are identity (part_id, spi_id), geometry (width, height,
// frame_bytes), capture's pass/fail and reset. capture() fills a caller buffer
// of 19200 / 76800 bytes the simulator cannot hand the twin, so pixel data is
// out of scope: the twin answers whether a frame WOULD have been captured and
// counts them (frames_captured). With no pixels to change, a scene or
// brightness stimulus would move nothing the program can read, so none is
// declared; auto-exposure on the real part holds mean brightness constant
// anyway.
//
// State after init: Studio's generated main() passes cfg = NULL, so the driver
// runs 160 x 120 (analog 2x skip); on a Core the SPI bus's own chip select is
// used (the PAL comes from core_tiles_pal2, I2C + SPI). The vendor tables leave the
// sensor running continuously (R_TG_En = 1) at 30 fps: base table writes
// R_Frame_Time = 0x061A80 = 400000 (bank 0, 0x4C..0x4F), and frame rate =
// 12 MHz / R_Frame_Time (datasheet §9.1, 24 MHz system clock / 2).
//
// Pads (Sense-CAM-P-a.json): GND 1, GPIO1 2 (LED strobe / SPI image interrupt,
// not configured by the driver), GPIO2 3 (I2C address strap: float 0x35,
// VDDIO 0x25, GND 0x40), I2C 4/5, SPI MISO/CLK/CS/MOSI 6/7/8/9, V+ 10
// (1.8-3.3 V). The driver drives no pad, so there are no pad outputs.
import type { PowerCtx, TileSim } from '../tileSim';

interface State {
  // ── fault injection (controls) ──
  /** 1 = the host services buffers too slowly: FB_Ovf trips and capture fails. */
  readout_too_slow: number;

  // ── driver / sensor state ──
  /** tile->state == READY: init succeeded and reset() has not been called. */
  tile_ready: number;
  /** R_TG_En = 1: the sensor free-runs and the SPI register side answers. */
  running: number;
  /** sense_cam_p_res_t: 0 = 160 x 120, 1 = 320 x 240. Driver-global. */
  resolution: number;
  /** Frames the sensor has produced since init (deriveState, 30 fps). */
  sensor_frames: number;
  /** `t` of the last modeled sensor frame. */
  last_frame_t: number;
  /** Successful capture() calls. */
  frames_captured: number;
}

const PART_ID = 0x7920;
/** Mean pixel level auto-exposure held on the bench (validator, 2026-08-10). */
const AE_MEAN = 98;
const SPI_CHECK_ID = 0xa55a;
/** Frame period after the vendor tables: 400000 / 12 MHz = 33.3 ms (30 fps). */
const FRAME_MS = 1000 / 30;

const width = (s: State) => (s.resolution === 1 ? 320 : 160);
const height = (s: State) => (s.resolution === 1 ? 240 : 120);
const frameBytes = (s: State) => width(s) * height(s);

// ── Supply current (datasheet Table 4, 25 °C) ──
// Parallel QVGA output: IDDMA 0.15 / 1.64 mA and IDDIO 0.15 / 1.74 mA at
// 15 / 180 fps, interpolated linearly in frame rate. Not in the datasheet:
// the SPI-output figure (the driver's mode) and the 160 x 120 skip-mode
// figure; both are assumed equal to parallel QVGA. Suspend: IDDMA 10 µA +
// IDDIO 1 µA.
//
// On the tile (BOM): VDDIO is pad 10 directly. VDDMA comes from a TPS610995
// boost (3.6 V) through a TPS7A2033 LDO (6.5 µA ground current), so its share
// of the pad current is scaled by 3.6 V / V+ / boost efficiency (~80 % at
// these loads, assumed) plus the boost's ~1 µA quiescent.
const IDDMA_UA = [150, 1640]; // at 15, 180 fps
const IDDIO_UA = [150, 1740];
const IDDMA_SUS_UA = 10;
const IDDIO_SUS_UA = 1;
const BOOST_MV = 3600;
const BOOST_EFF = 0.8;
const BOOST_IQ_UA = 1;
const LDO_IQ_UA = 6.5;
const lerpFps = (pair: number[], fps: number) =>
  pair[0]! + ((fps - 15) * (pair[1]! - pair[0]!)) / (180 - 15);
/** Current into pad 10 at V+ = vin_mv, running at `fps` or suspended. */
export function padCurrentUa(fps: number | null, vinMv: number): number {
  const ddma = fps == null ? IDDMA_SUS_UA : lerpFps(IDDMA_UA, fps);
  const ddio = fps == null ? IDDIO_SUS_UA : lerpFps(IDDIO_UA, fps);
  const vin = Math.max(1800, vinMv);
  return Math.round(ddio + ((ddma + LDO_IQ_UA) * BOOST_MV) / (vin * BOOST_EFF) + BOOST_IQ_UA);
}

const sim: TileSim<State> = {
  tile: 'Sense.CAM.P',

  defaultState: {
    readout_too_slow: 0,
    tile_ready: 1,
    running: 1,
    resolution: 0,
    sensor_frames: 0,
    last_frame_t: 0,
    frames_captured: 0,
  },

  controls: [
    {
      type: 'toggle',
      field: 'readout_too_slow',
      label: 'Readout too slow',
      description:
        'Each 4800-byte buffer must be read within ~3.4 ms or FB_Ovf voids the frame; capture() then returns 0.',
    },
  ],

  hostCalls: {
    tile_sense_cam_p_find: () => ({ scalar: 1 }),
    tile_sense_cam_p_init: () => ({
      scalar: 0,
      nextState: { tile_ready: 1, running: 1, resolution: 0 },
    }),
    // I²C PartID (bank 0, 0x00/0x01): answers whether or not the sensor runs.
    tile_sense_cam_p_part_id: () => ({ scalar: PART_ID }),
    // SPI CheckID reads 0x0000 whenever R_TG_En = 0 (after reset()).
    tile_sense_cam_p_spi_id: ({ state }) => ({
      scalar: state.running ? SPI_CHECK_ID : 0,
    }),
    tile_sense_cam_p_width: ({ state }) => ({ scalar: width(state) }),
    tile_sense_cam_p_height: ({ state }) => ({ scalar: height(state) }),
    tile_sense_cam_p_frame_bytes: ({ state }) => ({
      scalar: frameBytes(state),
    }),
    // dst is caller-sized (caps.dst = its length). 0 when not READY, when it
    // is too small for one frame, or when a buffer overflowed (FB_Ovf). A good
    // capture fills one frame: flat at the level auto-exposure held on the
    // bench (mean ~98), since no scene is modeled.
    tile_sense_cam_p_capture: ({ state, caps }) => {
      const len = caps?.dst ?? 0;
      if (state.tile_ready !== 1 || state.running !== 1) return { scalar: 0 };
      if (len < frameBytes(state)) return { scalar: 0 };
      if (state.readout_too_slow) return { scalar: 0 };
      return {
        scalar: 1,
        out: { dst: new Array(frameBytes(state)).fill(AE_MEAN) },
        nextState: { frames_captured: state.frames_captured + 1 },
      };
    },
    // Soft reset: registers to defaults (sensor stopped, SPI side silent), tile
    // state NONE until init runs again. The resolution is driver-global and
    // survives.
    tile_sense_cam_p_reset: () => ({
      nextState: { tile_ready: 0, running: 0 },
    }),
  },

  provenance: {
    tile_sense_cam_p_find: 'inferred',
    tile_sense_cam_p_init: 'inferred',
    tile_sense_cam_p_part_id: 'canonical', // PartID 0x7920, datasheet register map
    tile_sense_cam_p_spi_id: 'canonical', // CheckID 0xA55A, 0 while stopped (Table 29 + bench note)
    tile_sense_cam_p_width: 'canonical',
    tile_sense_cam_p_height: 'canonical',
    tile_sense_cam_p_frame_bytes: 'canonical', // width x height, 8-bit RAW
    tile_sense_cam_p_capture: 'inferred', // pass/fail per the readout rules; pixels flat at the bench AE mean
    tile_sense_cam_p_reset: 'inferred',
    power: 'inferred', // Table 4 parallel-output currents, interpolated; SPI / skip mode assumed equal; boost efficiency assumed
  },

  // The sensor free-runs at 30 fps while R_TG_En = 1.
  deriveState(state, { t }) {
    if (state.running !== 1) return {};
    if (t - state.last_frame_t < FRAME_MS) return {};
    const n = Math.floor((t - state.last_frame_t) / FRAME_MS);
    return { sensor_frames: state.sensor_frames + n, last_frame_t: t };
  },

  power(state, ctx?: PowerCtx) {
    const running = state.running === 1;
    const vin = ctx?.padVoltage?.['10'] ?? 3300;
    const ua = padCurrentUa(running ? 30 : null, vin);
    return {
      draw_ua: ua,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: vin,
          i_ua: ua,
          pads: ['10'],
          note: running
            ? 'streaming 30 fps: VDDIO direct, VDDMA via 3.6 V boost + 3.3 V LDO (Table 4 interpolated)'
            : 'stopped after reset (suspend figures) + regulator quiescent',
        },
      ],
    };
  },
};

export default sim;
