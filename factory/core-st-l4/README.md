# Core.ST.L4 factory image

The firmware every new Core.ST.L4 ships with. The TileJig
(`projects/TileJig`, Core.ST.L4 mode) writes it into blank Cores over the ROM
bootloader on I2C, reads it back, and starts it.

What it does:

- **LED:** a 1 ms blip once a second.
- **USB serial:** on connect, one line, then quiet. `?` or Enter repeats it.
  ```
  Core.ST.L4 factory 1.0.0 sdk=factory-core-st-l4-v1.0.0 uid=<24 hex digits>
  ```
  The USB serial number is the same factory UID, so hosts tell Cores apart.
- **Updates:** USB serial update (`make flash-serial`, Studio) and remote
  reset, from the SDK's CDC driver. ROM DFU (`make flash-dfu`) always recovers.
- **Power:** sleeps (WFI) between SysTick ticks.

## Cutting a release

1. Change `FACTORY_VERSION` in `main.c` (semver), PR, merge.
2. Tag the merge commit `factory-core-st-l4-v<version>` and push the tag.
3. Build from a clean checkout at that tag. The banner's `sdk=` must read the
   tag, with no `-dirty`:
   ```
   git checkout factory-core-st-l4-v<version>
   make -C factory/core-st-l4 clean all
   ```
4. Attach `build/core-st-l4.bin` and its CRC-32 to a GitHub release on that tag.
5. Point the TileJig at that `.bin` (`mkimage.py`), rebuild and flash the jig.

## Before a release goes out

On one Core loaded by the jig: power-cycle it, plug it into USB, check the
banner, then `make flash-serial` any project onto it and confirm it runs.
