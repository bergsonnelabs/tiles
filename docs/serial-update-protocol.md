# Serial update protocol

Flash a **running** Core over the USB serial (CDC) port it already has, without
the ST ROM bootloader. No new USB device appears, so there is no driver to
install (no Zadig / WinUSB on Windows), no device picker in the browser, and no
unplug-and-replug afterwards.

Status: **Core.ST.L4 / L4.2 (STM32L422), ROM-DFU layout.** Hardware-verified
2026-09-24. Other Cores need their own flasher and their own check of the boot
rules in §5 before they get this.

- Firmware: `sdk/serial_update/` (flasher + protocol core), hook in
  `sdk/hal/hal_usb_cdc.c`, embedded blob `sdk/hal/hal_serial_update_blob_l4.h`.
- Host reference client: `tools/serial_update.py` (`make flash-serial`).
- Studio's client mirrors it (web `apps/studio/src/lib/serialUpdate.ts`).
- Tests: `make -C sdk/serial_update test` (no hardware).
- Design discussion: web `docs/usb-serial-update.md`.

## 1. Sequence

```
host                                              Core
────                                              ────
open the CDC port at 2400 baud  ───────────────►  app (USB IRQ) sees SET_LINE_CODING 2400:
                                                  arms the status stage, stops SysTick /
                                                  MPU / DMA, copies the flasher to SRAM,
                                                  writes the handoff block, jumps
Q  ────────────────────────────────────────────►  flasher (same USB connection, polled)
   ◄─────────────────────  SU READY 1 464 128 2048 0
B  size, crc32, DEV_ID  ───────────────────────►  checks chip + size (nothing erased)
   ◄─────────────────────  SU OK B
D  offset 0, crc, page 0  ─────────────────────►  checks the vector table, THEN erases
   ◄─────────────────────  SU OK D 2048           page 0 and holds it in RAM
D  offset 2048 … (one page per frame)  ────────►  erase, program, read back
   ◄─────────────────────  SU OK D <next>
E  ────────────────────────────────────────────►  CRC of the whole image; write page 0
   ◄─────────────────────  SU DONE                body, then its first 8 bytes last;
                                                  reset (OBL_LAUNCH) into the new app
```

The host re-sends `Q` every ~250 ms for ~1.5 s: the handoff takes a moment, and
firmware that predates serial update never answers (fall back to DFU). Replies
to a repeated `Q` can arrive late; a client skips `READY` lines except when
waiting for one.

## 2. Frames (host → Core)

16-byte little-endian header, then `len` payload bytes:

| bytes | field    | |
|-------|----------|-|
| 0–1   | `"SU"`   | sync |
| 2     | type     | `Q` `B` `D` `E` `X` |
| 3     | flags    | 0 |
| 4–5   | len      | payload bytes, ≤ 2048 |
| 6–7   | reserved | 0 |
| 8–11  | arg0     | |
| 12–15 | arg1     | |

| type | arg0 | arg1 | payload | |
|------|------|------|---------|-|
| `Q` query | – | – | – | → `SU READY` |
| `B` begin | image size | CRC-32 of the image | u32 DEV_ID (0x464) | size ≤ flash − 4 KB (the core_nvm pages); nothing is erased yet |
| `D` data  | offset | CRC-32 of payload | one page (the last may be short) | strictly in order from 0 |
| `E` end   | – | – | – | verify, write page 0, reset |
| `X` abort | – | – | – | reply, then reset |

A header that isn't plausible (unknown type, non-zero flags/reserved, len > 2048)
is skipped a byte at a time, **silently**: output the app printed before the
handoff can contain "SU", and must not be answered.

CRC-32 is IEEE (zlib `crc32`, JS equivalents), initial 0.

## 3. Replies (Core → host)

One ASCII line per frame, each shorter than a 64-byte USB packet:

```
SU READY <version> <dev_id hex> <flash KB> <page bytes> <page0 erased 0|1>
SU OK <type> [<next offset>]
SU ERR <code> <word>
SU DONE
```

| code | word | meaning | host does |
|------|------|---------|-----------|
| 1 | length | DATA empty, > page, or short before the end | bug: stop |
| 2 | device | DEV_ID isn't this chip | stop; wrong image |
| 3 | size | image empty, reaching the core_nvm pages (the top 4 KB), or chunk past the image | stop |
| 4 | vectors | image doesn't start with SP-in-SRAM + Thumb reset vector inside the image | stop; flash untouched |
| 5 | flash | erase / program / read-back failed | stop |
| 6 | order | not the next offset | stop |
| 7 | crc | chunk CRC mismatch | re-send the chunk |
| 8 | image-crc | whole-image CRC wrong at END | page 0 stays erased; start over |
| 9 | state | frame not valid now | stop |

A `D` for the chunk just written (its OK was lost) is acknowledged again without
rewriting — a client that times out simply re-sends. Seen on hardware.

## 4. Safety order

The rule the design rests on: after any interruption, the Core boots **the old
app**, **the ROM bootloader**, or **the complete new image** — never a mix.

1. Page 0 (vector table) is validated **before anything is erased**. A wrong or
   corrupt image is refused with the old app intact.
2. Page 0 is erased **before any other page is touched**. From that moment the
   chip is "empty" and boots the ROM bootloader (§5), so a half-written body is
   never run.
3. Page 0's data is held in RAM; the whole image's CRC is checked against
   flash + that buffer **before page 0 is written**.
4. Page 0 is written body first, its first double-word (SP + reset vector)
   **last**. A failure before that final write leaves `0x08000000` erased.
5. With page 0 erased the flasher never resets on its own; it waits for the
   host. With flash untouched it gives up after 10 s idle and resets into the
   old app.
6. The IWDG is fed throughout (Studio arms 5 s on every L4; it can't be stopped).
7. Only the pages the image covers are erased, and an image may not reach the
   top 4 KB (`SU_NVM_RESERVED`): that is core_nvm's store (its last two pages,
   `sdk/core/core_nvm.h`), so saved settings survive an update. The linker
   scripts end every image below it; the flasher and `tools/serial_update.py`
   both refuse one that doesn't (`SU ERR 3`).

`tools/test_serial_update.py` checks this on a simulated flash: a power cut after
every frame, and a failed program at every call.

## 5. The L4 boot rule — and why every reset is OBL_LAUNCH

RM0394 §2.6: with BOOT0 selecting main flash, *"a flash empty check mechanism is
implemented to force the boot from system flash if the first flash memory
location is not programmed (0xFFFF FFFF)"* — but the EMPTY flag *"is updated
only during the loading of option bytes"*: at power-on, or when `OBL_LAUNCH` is
set. A plain system reset keeps the old verdict.

So the flasher resets through `OBL_LAUNCH` (FLASH_CR bit 27, after the OPTKEYR
unlock) every time. The empty check then sees today's page 0: erased → ROM
bootloader, written → the new app. That includes a Core that was DFU-flashed and
never power-cycled, where a plain reset would re-enter the ROM.

The same rule is why a Core needs **one replug after a ROM-DFU flash** that
followed an empty flash: the ROM's `:leave` doesn't reload option bytes. Seen
again on the bench during recovery.

If option bytes force main-flash boot (`nSWBOOT0 = 0, nBOOT0 = 1`) the empty
check is off, and an interrupted update leaves a Core that needs BOOT0 held to
recover. Cores ship with factory option bytes; nothing in the SDK changes them.

## 6. Handoff (STM32L422)

The app writes `su_handoff_t` (`su_protocol.h`) at `0x20007000`: pointers to its
USB descriptors and strings (still in flash — the flasher copies them before
erasing anything), the current line coding, and whether the device is
configured. SRAM while the flasher runs:

| address | |
|---------|-|
| `0x20000000` | flasher image, vector table first, then `.bss` (< 0x7000) |
| `0x20007000` | handoff block |
| `0x20009FE0` | flasher stack top |
| `0x20009FF0` | DFU magic + brick-recovery words — never touched |

The app refuses the handoff (and stays running) if its interrupt stack has grown
below `0x20007400`.

The flasher keeps the USB peripheral exactly as the app left it — address,
endpoints, PMA layout, data toggles — so the host's open port never notices. It
answers every control request the app would, so a bus reset mid-update
re-enumerates as the same device (same VID/PID/serial string).

**Endpoint registers are written race-free**: `CTR_RX`/`CTR_TX` are rc_w0, so
bits not being cleared are written as 1, and "packet waiting" / "IN busy" come
from the `STAT` bits the hardware owns. The `ll_usb.h` helpers write back the
value they read, which loses a completion that lands between the read and the
write; the first bench run stalled on exactly that.

## 7. Limits

- **First flash** of a blank Core, a Core running older firmware, and a Core
  whose firmware has crashed or never started USB still go through the ROM
  bootloader (`make flash-dfu`, Studio's DFU path). Ship kits pre-flashed.
- Not under the parked custom-bootloader layout (`APP_OFFSET`): the flasher
  writes from `0x08000000`.
- `HAL_SERIAL_UPDATE_DISABLE` compiles it out.
- The trigger is any `SET_LINE_CODING` at 2400 baud. Something that opens the
  Core's port at 2400 by accident gets a 10 s pause and a reset into the same app.

## 8. Bench record (2026-09-24, Core.ST.L4 rev b, macOS)

| | |
|-|-|
| 15 KB image | 0.8 s, 10/10 back-to-back updates verified by the running image |
| 120 KB image (> the 5 s watchdog) | 6.8 s, 13 in a row |
| lost reply | client re-sent, flasher acknowledged the duplicate |
| abort after page 0 erased | came back as the ROM bootloader |
| power pulled at 100/120 KB | ROM bootloader on replug; recovered by DFU + one replug |
| firmware without serial update | no answer; client exits 2 → use `make flash-dfu` once |

Not yet run: a Windows host (the point of all this), a Windows host with
STM32CubeProgrammer's driver installed, Linux.
