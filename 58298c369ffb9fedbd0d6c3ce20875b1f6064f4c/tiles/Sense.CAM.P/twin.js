// src/sims/sense_cam_p.ts
var PART_ID = 31008;
var AE_MEAN = 98;
var SPI_CHECK_ID = 42330;
var FRAME_MS = 1e3 / 30;
var width = (s) => s.resolution === 1 ? 320 : 160;
var height = (s) => s.resolution === 1 ? 240 : 120;
var frameBytes = (s) => width(s) * height(s);
var IDDMA_UA = [150, 1640];
var IDDIO_UA = [150, 1740];
var IDDMA_SUS_UA = 10;
var IDDIO_SUS_UA = 1;
var BOOST_MV = 3600;
var BOOST_EFF = 0.8;
var BOOST_IQ_UA = 1;
var LDO_IQ_UA = 6.5;
var lerpFps = (pair, fps) => pair[0] + (fps - 15) * (pair[1] - pair[0]) / (180 - 15);
function padCurrentUa(fps, vinMv) {
  const ddma = fps == null ? IDDMA_SUS_UA : lerpFps(IDDMA_UA, fps);
  const ddio = fps == null ? IDDIO_SUS_UA : lerpFps(IDDIO_UA, fps);
  const vin = Math.max(1800, vinMv);
  return Math.round(ddio + (ddma + LDO_IQ_UA) * BOOST_MV / (vin * BOOST_EFF) + BOOST_IQ_UA);
}
var sim = {
  tile: "Sense.CAM.P",
  defaultState: {
    readout_too_slow: 0,
    tile_ready: 1,
    running: 1,
    resolution: 0,
    sensor_frames: 0,
    last_frame_t: 0,
    frames_captured: 0
  },
  controls: [
    {
      type: "toggle",
      field: "readout_too_slow",
      label: "Readout too slow",
      description: "Each 4800-byte buffer must be read within ~3.4 ms or FB_Ovf voids the frame; capture() then returns 0."
    }
  ],
  hostCalls: {
    tile_sense_cam_p_find: () => ({ scalar: 1 }),
    tile_sense_cam_p_init: () => ({
      scalar: 0,
      nextState: { tile_ready: 1, running: 1, resolution: 0 }
    }),
    // I²C PartID (bank 0, 0x00/0x01): answers whether or not the sensor runs.
    tile_sense_cam_p_part_id: () => ({ scalar: PART_ID }),
    // SPI CheckID reads 0x0000 whenever R_TG_En = 0 (after reset()).
    tile_sense_cam_p_spi_id: ({ state }) => ({
      scalar: state.running ? SPI_CHECK_ID : 0
    }),
    tile_sense_cam_p_width: ({ state }) => ({ scalar: width(state) }),
    tile_sense_cam_p_height: ({ state }) => ({ scalar: height(state) }),
    tile_sense_cam_p_frame_bytes: ({ state }) => ({
      scalar: frameBytes(state)
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
        nextState: { frames_captured: state.frames_captured + 1 }
      };
    },
    // Soft reset: registers to defaults (sensor stopped, SPI side silent), tile
    // state NONE until init runs again. The resolution is driver-global and
    // survives.
    tile_sense_cam_p_reset: () => ({
      nextState: { tile_ready: 0, running: 0 }
    })
  },
  provenance: {
    tile_sense_cam_p_find: "inferred",
    tile_sense_cam_p_init: "inferred",
    tile_sense_cam_p_part_id: "canonical",
    // PartID 0x7920, datasheet register map
    tile_sense_cam_p_spi_id: "canonical",
    // CheckID 0xA55A, 0 while stopped (Table 29 + bench note)
    tile_sense_cam_p_width: "canonical",
    tile_sense_cam_p_height: "canonical",
    tile_sense_cam_p_frame_bytes: "canonical",
    // width x height, 8-bit RAW
    tile_sense_cam_p_capture: "inferred",
    // pass/fail per the readout rules; pixels flat at the bench AE mean
    tile_sense_cam_p_reset: "inferred",
    power: "inferred"
    // Table 4 parallel-output currents, interpolated; SPI / skip mode assumed equal; boost efficiency assumed
  },
  // The sensor free-runs at 30 fps while R_TG_En = 1.
  deriveState(state, { t }) {
    if (state.running !== 1) return {};
    if (t - state.last_frame_t < FRAME_MS) return {};
    const n = Math.floor((t - state.last_frame_t) / FRAME_MS);
    return { sensor_frames: state.sensor_frames + n, last_frame_t: t };
  },
  power(state, ctx) {
    const running = state.running === 1;
    const vin = ctx?.padVoltage?.["10"] ?? 3300;
    const ua = padCurrentUa(running ? 30 : null, vin);
    return {
      draw_ua: ua,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: vin,
          i_ua: ua,
          pads: ["10"],
          note: running ? "streaming 30 fps: VDDIO direct, VDDMA via 3.6 V boost + 3.3 V LDO (Table 4 interpolated)" : "stopped after reset (suspend figures) + regulator quiescent"
        }
      ]
    };
  }
};
var sense_cam_p_default = sim;
export {
  sense_cam_p_default as default,
  padCurrentUa
};
