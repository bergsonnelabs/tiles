# Tile twins

A tile's **digital twin** is its behavior in software: what its driver calls
return, how its state evolves, what it draws from its supply, which LEDs it
shows and pads it drives, and what a person can do to it in Studio's Simulate
panel. One twin per tile, in `src/sims/<driver>.ts`, next to the driver it
models (`drivers/tile_<driver>.{h,c}`). Passive parts (batteries, USB, LEDs)
are in `src/generics/`.

Studio runs the same twin in its simulator worker (answering the firmware's
driver calls) and on its main thread (power, indicators, controls); the portal
runs it to score twins.

## The contract

`src/tileSim.ts`. The points that matter when writing one:

- **The driver decides.** Field names follow the driver, `defaultState` is the
  state after the driver's `init`, and behavior matches the driver and the
  datasheet (`~/Documents/local/tile references/<Stem>/`), including the
  driver's bugs until they're fixed.
- **Handlers are memory-free.** Return plain numbers: `scalar`, `array` (a
  fixed-length out array), `out[param]` (a caller-sized one, clipped to
  `caps[param]`), `outScalars[name]`. The simulator writes them at the width
  the manifest declares.
- **Pad outputs are keyed by pad number** (`'9'`), from the tile definition.
- **`stimuli`** declares what a person does to the part (move the object, tilt
  the board), as plain data.
- **Self-contained:** a twin file may import types only.
- **Provenance is a promise:** `canonical` (checked against the datasheet),
  `inferred` (from the driver), `hallucinated` (a placeholder).

Bump `CONTRACT_VERSION` for any change a consumer must understand.

## Checks

```
cd twins && npm ci && npm run check
```

`test/contract.test.ts` holds every twin to its driver's generated manifest
(`manifests/tile_*.json`): every call it answers exists, results have the shape
the driver declares, pad keys are pad numbers, and every field a call writes or
`power` / `indicators` / `padOutputs` / `deriveState` / `stimuli` read exists in
the state. CI runs it on every PR that touches `twins/`, `manifests/` or
`drivers/` (`.github/workflows/twins.yml`).

## The bundle

Every commit on `main` is published as a bundle (`npm run bundle`,
`.github/workflows/bundle.yml`): each tile's manifest, docs, definitions and
compiled twin (one ES module per tile), with an index carrying
`CONTRACT_VERSION` and the build fingerprint. It goes to the `bundles` branch
(`<sha>/…`, `latest.json`), and Studio loads it at runtime through the site,
which hands each viewer only the tiles they may see. A twin change reaches
Studio without a web deploy.

The same workflow compares the commit's build fingerprint
(`tools/build_fingerprint.py`: what the build server compiles, ignoring
comments and docs) with the deployed build server's, and asks web to redeploy
it only when they differ.
