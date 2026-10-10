# BLE update protocol (Core.ST.W5)

Install a new firmware image on a **running** Core.ST.W5 (STM32WBA55, no USB)
over Bluetooth LE, from Studio. The running app receives the image into free
flash above itself (staging) while BLE keeps working, checks it, and only then
hands over to a small copier that lives in flash page 0 and survives the copy.

Status: firmware and host-side tests in tiles (2026-10). Full updates benched
on a Core.ST.W5 (2026-10-10: several in a row, each confirmed by `READY`'s
CRC; a host vanishing mid-transfer and a link drop both left the old program
running). Delta updates (`S`, §3.1) benched 2026-10-10 (one constant changed:
2048 bytes sent, `READY`'s CRC confirmed). Kept pages (§3.1) benched
2026-10-10: a one-constant edit sent 2,048 of 158,532 B, transfer 4.2 s, the
board back on the new program soon after (total with a 3.7 s build ≈ 6-9 s,
against 14.9 s for a full update). A power cut during the transfer and during
the install both recovered. **The reflash guard (§8.2) is host-tested only.**
Recovery is SWD (Studio's SWD flasher or a probe).

- Firmware: `sdk/ble/ble_update.c` (BLE glue, staging, commit),
  `sdk/ble/ble_update_boot.c` (page-0 boot hook + RAM copier),
  `sdk/serial_update/su_core.c` built with `SU_STAGING` (the protocol state
  machine the USB flashers use), `sdk/serial_update/su_ota.c` (reserved page,
  install marker, copy algorithm), `sdk/serial_update/su_cmac.c` (AES-128,
  AES-CMAC).
- Host tests (no hardware): `make -C sdk/serial_update test` (`test/ota_sim.c`).
- Build guard: every BLE link runs `tools/check_ota_boot.py`, which fails the
  build if the copier could branch or point outside page 0 / its SRAM copy.
- Framing: the USB serial-update protocol, `docs/serial-update-protocol.md`.
  This document is self-contained; that one explains where the frames came from.

It is automatic: every Core.ST.W5 build with BLE enabled (`"ble": {"enabled":
true}`) carries it. There is no API to call and nothing in `main.c`; the app
only has to keep calling `core_ble_process()`.

## 1. GATT

One custom service, **Studio Link**, in the SDK's base UUID
`0000xxxx-8e22-4541-9d4c-21edae82ed19`. It is registered by `core_ble`
whenever BLE is on, after the application's own services (so it never shifts
their handles).

| what | 16-bit id | UUID | properties |
|------|-----------|------|------------|
| service Studio Link | `0x5C00` | `00005c00-8e22-4541-9d4c-21edae82ed19` | primary |
| Studio Scope (scope builds only) | `0x5C01` | `00005c01-8e22-4541-9d4c-21edae82ed19` | notify (`docs/scope-protocol.md`) |
| **Studio Update** | `0x5C02` | `00005c02-8e22-4541-9d4c-21edae82ed19` | write, write without response, notify |

- **Host → Core:** write-without-response (plain write also works) packets of
  **at most 180 bytes**. A frame (§3) is split across as many packets as it
  needs; packet boundaries carry no meaning. Negotiate an ATT MTU of at least
  183 (every desktop and phone central does; Web Bluetooth does it for you).
- **Core → host:** one notification per reply line (§4), ASCII, ending in `\n`,
  at most 63 bytes. **Subscribe to notifications before the first write.**
- Each characteristic has a User Description descriptor ("Studio Update").
- **Advertising:** the Core advertises the Studio Link UUID in its **scan
  response** (`11 07 19 ed 82 ae ed 21 4c 9d 41 45 22 8e 00 5c 00 00`: complete
  list of 128-bit UUIDs, little-endian), so a central can filter on it (Web
  Bluetooth `filters: [{ services: [...] }]`; needs an active scan, which
  Chrome does). The advertising packet itself carries Flags, the connection
  interval range and the complete name (up to 20 characters).
- If the application calls `core_ble_enable_pairing()`, every characteristic,
  Studio Link included, requires an encrypted link: pair first.
- Studio should be the only client while it updates (the transfer state is
  per Core, not per connection).
- The host should reserve `0x5C00`-`0x5C0F` (no user contract may use them).

## 2. Sequence

```
host (Studio)                                         Core (running app, BLE up)
─────────────                                         ──────────────────────────
connect, discover 0x5C00, subscribe to 0x5C02
Q  ──────────────────────────────────────────────►
   ◄──────────────────────  SU READY 3 492 1024 2048 0 507904 1 5a17c0de 8c1e33a7 delta 1
C  (challenge)  ─────────────────────────────────►    16 random bytes from the RNG
   ◄──────────────────────  SU NONCE 00112233445566778899aabbccddeeff
A  payload = CMAC(K, "SUA1" ‖ nonce)  ───────────►    constant-time compare
   ◄──────────────────────  SU OK A
B  size, crc32, DEV_ID 0x492  ───────────────────►    fit check, picks the staging
   ◄──────────────────────  SU OK B 0                 base (nothing erased yet)
D  offset 0, crc, 2048 bytes  ───────────────────►    vector check (chunk 0); erase the
   ◄──────────────────────  SU OK D 2048              staging page at each 8 KB boundary;
D  offset 2048 …  ───────────────────────────────►    program + read back in radio-safe
   ◄──────────────────────  SU OK D 4096              windows; absorb into the image MAC
S  offset 4096, length 8192 (delta, §3.1)  ──────►    the same, copying those bytes from
   ◄──────────────────────  SU OK S 12288             the running image instead
   …
E  payload = CMAC(K, "SUE1" ‖ nonce ‖ image_tag) ►    CRC-32 of the staged flash, then
   ◄──────────────────────  SU DONE                   the tag; write the install marker,
                                                      reset ~0.5 s later
(link drops: the Core resets)                         page-0 copier: staging → 0x08000000,
                                                      page 0 last (skipped if unchanged);
                                                      marker DONE; reset
                                                      new app boots, advertises again
```

- Send `Q` first and re-send it every ~250 ms (up to ~2 s) until `READY`
  arrives: bytes written before the Core has seen the connection can be
  dropped (§6). Ignore unexpected `READY` lines later on.
- `C` / `A` authenticate **this connection**. A disconnect forgets it; a new
  connection starts again at `Q`, `C`, `A`.
- **Flow control (window of two):** a host may keep **at most two** `D` / `S`
  frames unacknowledged, counting both kinds together (send frame n+1 before
  the `OK` of frame n arrives, never n+2). The Core's receive ring holds two whole frames on its own
  (4224 B = 2 × 2064 + 96, §6), so this never drops bytes even if the app's
  main loop is slow. Stop-and-wait (one frame) is always fine too. Wait up to
  **3 s** for any reply (an erase waits for a radio window; an 8 KB `S`
  programs a whole page; `E` reads the whole staged image).
- **The one recovery rule** (the firmware follows exactly this; host-tested):
  after **any** `ERR 6` / `7` / `9`, a timeout, or a reconnect, wait ~300 ms
  for stray replies, then send `B` again with the same size and CRC (after `C`
  / `A` if the link is new). `SU OK B <next>` is where to continue: the next
  chunk the Core does not have. A chunk that got `ERR 7` was not written, so
  `next` is its offset; with two in flight the second one also gets `ERR 6`
  and is covered by the same `B`. (With stop-and-wait, simply re-sending the
  chunk after `ERR 7` also works.) Within one power cycle an interrupted
  transfer resumes across disconnects; after the Core resets, `next` is 0.
- After `SU DONE` the Core ignores every frame (no replies) until it resets
  ~0.5 s later; it drops the link itself at ~0.3 s. Reconnect after ~2 s plus
  the copy time (§9) and send `Q`.
- **Confirming the install:** `READY`'s last field is the CRC-32 of the image
  the Core is running (§4). Equal to the CRC sent in `B` → installed. Equal to
  the old image's → not installed (a lost `E`, a failed check, or an abandoned
  install): start over. Don't infer it from `B`: `OK B 0` only means "no
  transfer in progress", which is true both after an install and after a
  reset that lost a transfer.
- **`X` is always harmless.** It only forgets the in-RAM transfer state: it
  never erases or programs flash and never touches the install marker or the
  staged image. Before the post-`DONE` reset it is ignored like every frame;
  on the new app there is nothing to forget. Sending it "to be safe" is fine,
  just unnecessary; use `READY`'s CRC to decide.

## 3. Frames (host → Core)

16-byte little-endian header, then `len` payload bytes (the same header as
serial update):

| bytes | field    | |
|-------|----------|-|
| 0-1   | `"SU"`   | `0x53 0x55` |
| 2     | type     | ASCII: `Q` `C` `A` `B` `D` `S` `E` `X` |
| 3     | flags    | 0 |
| 4-5   | len      | payload bytes, ≤ 2048 |
| 6-7   | reserved | 0 |
| 8-11  | arg0     | u32 |
| 12-15 | arg1     | u32 |

| type | arg0 | arg1 | payload | auth needed | reply |
|------|------|------|---------|-------------|-------|
| `Q` query | 0 | 0 | none | no | `SU READY …` |
| `C` challenge | 0 | 0 | none | no | `SU NONCE <32 hex>` |
| `A` authenticate | 0 | 0 | 16 B auth tag (§5) | no | `SU OK A` |
| `B` begin | image size | CRC-32 of the image | u32 DEV_ID = `0x00000492` | yes | `SU OK B <next offset>` |
| `D` data | offset | CRC-32 of this payload | 2048 B of image (the last chunk may be shorter) | yes | `SU OK D <next offset>` |
| `S` same (delta, §3.1) | offset | length | none | yes | `SU OK S <next offset>` |
| `E` end | 0 | 0 | 16 B end tag (§5) | yes | `SU DONE` |
| `X` abort | 0 | 0 | none | yes | `SU OK X` |

- **`B`** checks the chip (DEV_ID[11:0] = `0x492`) and that the image fits
  (§7). Nothing is erased. If a transfer of the same size and CRC is already
  in progress (same power cycle) it is kept and `next` is where it stopped;
  any other `B` starts a new transfer at 0.
- **`D`** chunks go strictly in order from offset 0, exactly 2048 bytes each
  except the last. Chunk 0 must start with a vector table for this chip:
  initial SP in SRAM (`0x20000000` < SP ≤ `0x20020000`, word aligned) and a
  Thumb reset vector inside the image as linked at `0x08000000`. A resend of
  the chunk just written (its `OK` was lost) is acknowledged again, unchanged.
- **`S`**: see §3.1.
- **`E`** is accepted once every chunk is in. The Core checks the end tag
  (`ERR 13`), then the image's CRC-32 (`ERR 8`), writes the install marker
  (§7) and replies `SU DONE`. A wrong tag or CRC forgets the transfer (start
  over at `B`, offset 0). Both values accumulate as the frames arrive (each
  staged chunk as read back from flash, each kept page as read from the
  running image), so `E` reads no flash; the copier checks the assembled
  image again before it erases anything (§8.2).
- **`X`** forgets the transfer. The running app carries on; nothing resets.
- Unknown types, non-zero flags/reserved, or `len` > 2048 are skipped a byte
  at a time **silently**, as on USB.

CRC-32 is IEEE 802.3 (zlib `crc32`, initial value 0), as serial update.

### 3.1 Delta updates: the `S` frame

Since the link-order change (§7) a small program edit changes about one 8 KB
page, so most of a new image equals the running one byte for byte. `S`
("same") says so instead of sending the bytes.

- **When:** only if `READY` ends in `delta 1` (§4) **and** the host holds the
  image whose CRC-32 equals `READY`'s running-image CRC (Studio keeps the
  `.bin` it last installed). Otherwise send `D` frames only.
- **Frame:** type `'S'` (0x53), the usual 16-byte header, `len` = 0 (no
  payload), arg0 = offset in the new image, arg1 = length. Authentication
  needed, like `B` / `D` / `E`. Valid only during a transfer started by `B`.
- **Meaning:** the new image's bytes `[offset, offset + length)` equal the
  **running** program's bytes at the same offsets. Two cases, by shape:
  - **Keep** (the usual one): an `S` that is exactly one 8 KB flash page
    (offset a multiple of 8192, length 8192), or the image's partial last
    page (offset a multiple of 8192, ending exactly at the image size). The
    page stays where it is: the Core reads it once, for the image CRC and the
    image tag, records it as kept, and neither stages it nor, at install,
    erases or rewrites it.
  - **Copy:** any other `S` (part of a page). The Core copies those bytes
    from its running image into staging exactly as if `D` frames had carried
    them (staging erase at each 8 KB boundary, program + read-back in radio
    windows). A page that mixes `S` and `D` chunks is staged whole this way.
  Either way the bytes go into the image tag (`SUI1` …) and the image CRC in
  order, so `E` still proves the assembled image is the host's image.
- **Rules:**
  - offset = the next expected offset (so a multiple of 2048). Anything else:
    `ERR 6 order`, as for `D`. `S` at offset 0 is fine (page 0 is usually
    unchanged): chunk 0's vector check (§3, `D`) runs on the copied bytes,
    which pass it unless the reset vector lies past the new image's end
    (`ERR 4`).
  - 1 ≤ length ≤ 8192, a multiple of 2048, except a final run, which may end
    exactly at the image size.
  - offset + length ≤ the new image size (from `B`) **and** ≤ the running
    image's size (its end; the Core knows it).
  - `len` ≠ 0, or any length / range rule broken (either image's end
    included): `ERR 1 length` (a host bug: stop). Before `B`: `ERR 9`;
    before `A`: `ERR 12`.
  - A resend of the run just written (its `OK` was lost) is acknowledged
    again, unchanged, as for `D`.
- **Reply:** `SU OK S <next offset>`, like `OK D`. The window of two (§2)
  counts `S` and `D` together. `B`'s resume offset covers both.
- **Wrong base:** if the running program isn't the image the host diffed
  against, the copied bytes differ from the host's, and `E` answers
  **`ERR 13 mac`** (the tag is checked before the CRC). Nothing is installed.
  Start over with `D` frames only (a full transfer). Treat an `ERR 8` after a
  transfer that used `S` the same way.
- **Host plan** (`tools/page_diff.py --frames` computes it): walk the new
  image in 2048-byte chunks; a chunk that lies inside the old image and
  equals it joins the current `S` run, which is sent when it reaches an
  8 KB page boundary, at the next differing chunk, or at the end; a
  differing chunk (or one past the old image's end) goes as a `D`. So every
  unchanged page goes as one page-aligned 8 KB `S` and is kept. (Runs that
  cross page boundaries are valid too, they are just copied.)
- **Cost on the Core:** a kept page ≈ 45-65 ms (CMAC ≈ 40-60 ms, CRC ≈ 4
  ms at 32 MHz), no flash writes; a staged page adds ≈ 60 ms of erase +
  program in radio windows. A typical small edit (one constant changed in
  `tests/ota-bench-a`, a 155000-byte image, 19 pages) is **19 `S` + 1 `D`**
  (1400 bytes sent; 18 pages kept, the last one staged) instead of 76 `D`.
  Estimates, not benched: ≈ 1-1.3 s on the Core for the transfer, ≈ 0.5 s
  for the install (§9). The software CMAC is most of the transfer time.

## 4. Replies (Core → host)

One ASCII line per notification:

```
SU READY 3 <dev_id hex> <flash KB> <chunk bytes> 0 <image limit bytes> <key 0|1> <key_id hex> <image crc hex> delta <0|1>
SU NONCE <32 lowercase hex digits = 16 bytes, byte 0 first>
SU OK <type> [<next offset>]
SU ERR <code> <word>
SU DONE
```

`READY` version 3 (this protocol), e.g. `SU READY 3 492 1024 2048 0 507904 1 5a17c0de 8c1e33a7 delta 1`
(the first seven fields line up with v1/v2; v3 appends; parsers ignore
fields they don't know):

| token | value |
|-------|-------|
| 3 | protocol version (1 = L4 USB, 2 = H5 USB, 3 = W5 BLE staging) |
| `492` | DEV_ID, hex (STM32WBA5x: RM0493 §43.12.7 DBGMCU_IDCODER, DEV_ID[11:0]) |
| `1024` | flash size, KB |
| `2048` | chunk size: every `D` payload but the last is exactly this |
| `0` | page 0 erased: always 0 here (kept so v1/v2 parsers line up) |
| `507904` | the largest image this Core accepts **right now**, bytes (§7: depends on the running app's size) |
| `1` | a board key is provisioned (0: BLE update refused, `ERR 10`) |
| `5a17c0de` | the provisioned key's id (hex, as written at provisioning, §7.2); `0` without a key |
| `8c1e33a7` | CRC-32 of the running image: `[0x08000000, end of its last flash section)`, i.e. exactly the `.bin` it was built as (what `B`'s CRC covers). Computed once per boot, at the first `Q` (~50-100 ms) |
| `delta 1` | the literal word `delta`, then 1 if this firmware takes `S` frames (§3.1). Firmware from before `S` sends no such pair: treat a missing pair as `delta 0` |

Hex fields have no leading zeros (`0` for zero).

`OK B`, `OK D`, `OK S` carry the next offset. `OK A`, `OK X` carry nothing.
The longest `READY` (8-digit key id and CRC) is 61 characters plus `\n`.

| code | word | meaning | host does |
|------|------|---------|-----------|
| 1 | length | `D` empty, > 2048, or short before the end; `A`/`E` payload not 16 bytes; `S` with a payload, or any §3.1 length / range rule broken | bug: stop |
| 2 | device | DEV_ID in `B` isn't 0x492 | stop: wrong image |
| 3 | size | image empty, or it doesn't fit (§7), or a chunk past the end | stop: show the limit from `READY` |
| 4 | vectors | chunk 0 isn't a vector table for this chip | stop: not a W5 image |
| 5 | flash | erase / program / read-back / marker write failed; the transfer is forgotten | retry once from `B` (it starts at 0); then stop |
| 6 | order | not the next offset | re-`B`, continue from `next` |
| 7 | crc | chunk CRC mismatch; nothing written | re-`B`, continue from `next` (§2; a plain resend also works with stop-and-wait) |
| 8 | image-crc | CRC of the staged image wrong at `E` | start over (with `D` only if `S` was used) |
| 9 | state | frame not valid now (`D` before `B`, `E` before the last chunk) | re-`B`, continue from `next` |
| 10 | nokey | no board key provisioned | stop: "flash once over SWD to enable Bluetooth updates" |
| 11 | auth | `A` tag wrong (the nonce is spent: send `C` again) | check the key id; stop after 2 tries |
| 12 | noauth | `B`/`D`/`E`/`X` before a successful `A` on this connection | `C`, `A`, then re-`B` |
| 13 | mac | end tag wrong at `E`: the image or the key doesn't match; after `S` frames, usually the running image isn't the one the host diffed against | start over, with `D` only |
| 14 | rng | no nonce (RNG fault) | send `C` again |

After `SU DONE` there are no replies at all until the reset (not an error).

## 5. Authentication (per-board key)

Each Core has its own 128-bit key `K`, written by Studio's SWD flasher (§7.2)
and never sent over the radio. All tags are **AES-CMAC (RFC 4493, AES-128)**,
16 bytes, with a 4-byte ASCII label in front of every message so one tag can
never stand in for another:

```
auth_tag  = CMAC(K, "SUA1" ‖ nonce)                                  (20-byte message)
image_tag = CMAC(K, "SUI1" ‖ LE32(size) ‖ LE32(crc32) ‖ image)        (12 + size bytes)
end_tag   = CMAC(K, "SUE1" ‖ nonce ‖ image_tag)                      (36-byte message)
```

- `nonce`: the 16 bytes of the current connection's `SU NONCE` line (hex
  decoded, first pair = byte 0). A nonce is good for one `A` (right or wrong),
  and `E` uses the same nonce as the `A` that authenticated the connection.
- `size`, `crc32`: the values sent in `B`. `image`: exactly `size` bytes.
- The Core computes `image_tag` as chunks arrive and as `S` runs are copied or kept (it never needs the whole
  image in RAM) and checks `end_tag` at `E`, so an image or tag recorded from
  an earlier session does not replay: the end tag is bound to this nonce.
- Compare-time is constant; a wrong `A` spends the nonce.

**Browsers:** Web Crypto has no CMAC (or ECB). Build it from `AES-CBC` with a
zero IV (RFC 4493 §2.3: `L = AES-CBC(K, IV=0, 16 zero bytes)[0..15]` gives the
subkeys; the CBC-MAC is the last 16 bytes of `AES-CBC(K, IV=0, message
blocks with the last one masked)`, dropping Web Crypto's padding block).
Check it against the vectors below.

### Test vectors

RFC 4493 §4 (K = `2b7e151628aed2a6abf7158809cf4f3c`): the empty message gives
`bb1d6929e95937287fa37d129b756746`; the 16-byte `6bc1bee22e409f96e93d7e117393172a`
gives `070a16b46b4d4144f79bdd9dd04a287c`. The firmware's tests run all four.

Protocol vectors (checked against OpenSSL 3 `openssl mac CMAC`; the firmware
tests reproduce them):

```
K        = 000102030405060708090a0b0c0d0e0f
nonce    = f0e1d2c3b4a5968778695a4b3c2d1e0f
auth_tag = 9edcf74eaf564b287d7cfec175987d53

image    = the 40 bytes 00 01 02 … 27 (hex), size = 40
crc32    = 0x0da62e3c  (LE32 in the message: 3c 2e a6 0d)
image_tag= 559931c85ad5df723ad703750da306c5
end_tag  = f18a6f9daccd1c7c5de8e5b106df0802
```

## 6. What the Core does with the link

- Writes land in a **4224-byte** receive ring from the BLE event handler (two
  whole frames); the main loop moves them into the frame parser's own
  2128-byte buffer and handles each frame from `core_ble_process()`, never in
  the BLE event context, so the flash work can wait for radio windows. A write
  that doesn't fit is dropped whole (a host more than two frames ahead): the
  next frame then gets `ERR` or no reply; recover with `B`.
- A disconnect clears the authentication and any partial frame, and keeps the
  transfer (size, CRC, next offset, the running image tag) for a resume.
- Bytes that arrive between a reconnect and the next `core_ble_process()` can
  be dropped: that is why `Q` is repeated.
- Staging flash is written through ST's flash manager in radio-safe windows
  (`sdk/ble/ble_flash.c`), so advertising, the connection and the app's own
  services keep running during a transfer.

## 7. Flash layout (STM32WBA55, 1 MB, 128 pages of 8 KB)

```
0x08000000  page 0      the app's page 0: vector table, Reset_Handler, the boot hook,
                        and the copier (loaded into SRAM at every boot)
            pages 1..   the running app ...........  ends at __flash_image_end
            staging     from page_ceil(max(app end, new image size)), `size` bytes;
                        image page p at staging + p × 8 KB, kept pages left unused
0x080F8000  page 124    RESERVED: board key + install marker log (§7.1, §7.2)
0x080FA000  pages 125-126  core_nvm (core_nvm.h)
0x080FE000  page 127    BLE bonds (nvm_stub.c)
```

- **Every W5 image now ends below `0x080F8000`** (992 KB; both linker scripts,
  BLE and not). Studio's SWD flasher must refuse an image reaching page 124
  and must never erase pages 124-127 as part of an image.
- **Fit rule.** Staging starts at `S = page_ceil(max(A, N))`, where `A` is the
  running app's end (`__flash_image_end`) and `N` the new image size, and needs
  `S + N ≤ 0x080F8000 - 0x08000000` (= 1015808). Staging never overlaps the
  running app (it is live code) nor the destination of the copy (so a resumed
  copy re-reads intact data). Kept pages (§3.1) leave their staging page
  untouched (not erased, never read); the fit rule doesn't count on them, as
  `B` can't know which pages will be kept. The largest image that fits is therefore
  `L - page_ceil(A)` when `page_ceil(A) ≥ L/2`, else `L/2` = 507904, with
  L = 1015808. `READY` reports it; `B` applies the exact rule.
- An image with BLE off installs fine but can't take the next Bluetooth
  update (no BLE): the next update is SWD.
- **Link order.** Page 0 holds the vector table, `Reset_Handler` (it calls the
  boot hook and branches on to `Reset_Continue`, which sits just before the
  project's code and holds the `.data`/`.bss` bounds), the boot hook and the
  copier, then the start of the ST BLE stack. The project's own code and
  constants are linked last (Makefile `OBJECTS`, the linker scripts), so a
  program edit usually leaves page 0 and every page but the last one or two
  unchanged (`tools/page_diff.py`). Page 0 still changes with the SDK, with a
  project-defined interrupt handler (vector table) or constructor, and a new
  *initialized* project global shifts `.bss`, which changes most pages.
  Consequences: most of a new image can go as `S` runs (§3.1), and the old
  and the new page 0 are usually identical, which the copier uses to skip
  rewriting page 0 (§8.2).

### 7.1 Page 124 (`0x080F8000`-`0x080F9FFF`)

Every record is one or two 16-byte flash quad-words (the WBA programs 128-bit
words with ECC; each may be written once between erases). Little-endian u32s.

| offset | size | content |
|--------|------|---------|
| `0x000` | 32 B | **key record** (§7.2), written over SWD |
| `0x020`-`0x0FF` | | unused, 0xFF |
| `0x100`-`0x1FFF` | 496 × 16 B | **install marker log**, appended by the Core |

Marker log records (the Core writes them; Studio only ever erases them):

| record | u32 [0] | [1] | [2] | [3] |
|--------|---------|-----|-----|-----|
| COPY-A | `0x41504F43` "COPA" | staging address | image size | image CRC-32 |
| KEEP   | `0x5045454B` "KEEP" | kept pages 0-31 (bit p = image page p) | kept pages 32-63 | bitwise NOT of ([1] XOR [2]) |
| COPY-B | `0x33504F43` "COP3" | size of the image running at the commit (bytes, its end) | CRC-32 of that image | CRC-32 of COPY-A (16 B) ‖ KEEP (16 B) ‖ COPY-B[0..11] (12 B) |
| START  | `0x54525453` "STRT" | the COPY-B check it refers to | 0 | bitwise NOT of [1] |
| DONE   | `0x454E4F44` "DONE" | the COPY-B check it closes | 0 installed, 1 abandoned | bitwise NOT of [1] |

The app writes COPY-A, KEEP, COPY-B in three consecutive slots at `E` (a
full transfer writes KEEP with no bits set); the copier writes START just
before it first erases anything, and DONE at the end. The install is pending
("COPYING") when the last valid COPY-A/KEEP/COPY-B group has no DONE after
it, and started when a START for it follows. COPY-B's check covers all three,
so a torn KEEP (or COPY-A) leaves no pending install: the old app boots. A
record that reads back with an ECC error or doesn't check out is skipped.
The log is compacted (page erased, key record rewritten) when fewer than 8
slots remain: about every 99 installs (five records each). Images are at
most 62 pages (the fit rule), so 64 bits cover every page; a kept bit past
the image's last page, or on a page reaching past the old image's end, makes
the install abandoned.

**Old markers.** Earlier firmware wrote COPY-B as `0x42504F43` "COPB" (page-0
CRCs) or `0x32504F43` "COP2" (no KEEP). Current firmware doesn't treat either
as a COPY-B, so such an install reads as not pending. The only way a
current copier meets one is right after an older firmware installed it and
the power failed after page 0 was written (the install is then complete
except for its DONE) or a reflash that skipped §7.2 step 3; in both cases
booting what is there is right.

### 7.2 Key record (Studio's SWD flasher writes it)

32 bytes at `0x080F8000`, two quad-words:

| offset | u32 / bytes | value |
|--------|-------------|-------|
| `0x00` | u32 | magic `0x314B5553` (bytes `53 55 4B 31`, "SUK1") |
| `0x04` | u32 | key id: Studio's handle for this key, any value but 0 and 0xFFFFFFFF; `READY` reports it |
| `0x08` | u32 | CRC-32 (IEEE) of bytes `0x00-0x07` followed by the 16 key bytes `0x10-0x1F` (24 bytes) |
| `0x0C` | u32 | `0xFFFFFFFF` |
| `0x10` | 16 B | K, byte 0 first (the K of RFC 4493) |

A record that doesn't check out counts as "no key" (`READY … 0 0`, `ERR 10`).
Generate K with a CSPRNG, one per board; keep (key id → K) with the board in
the user's account.

**On every SWD flash of a Core.ST.W5** (the first flash provisions; later ones
keep the key and must clear a pending install):

1. Halt the core at the reset vector as today (so neither the app nor the
   copier runs during the flash).
2. Read the 32 bytes at `0x080F8000`. If they form a valid key record, keep
   them; else (first flash, or a torn one) make a new record (new K, new key
   id, CRC) and remember (key id → K).
3. **Erase page 124** (this clears the install marker log), then program the
   32-byte key record (two quad-words) at `0x080F8000`, and read it back.
4. Program the image (pages below 124 only), then reset.

Don't skip step 3 on a reflash. If the copy hadn't started, the copier's
reflash guard (§8.2) sees that the running image changed and abandons the
pending install; but if a copy was interrupted (START written), the copier
resumes it without that check and puts the staged image over the
SWD-flashed app. Never erase pages 125-127.

Note: K protects the radio path only. Code running on the Core and anything
with SWD access can read page 124 (RDP 0, no TrustZone).

## 8. Install (the copier) and recovery

### 8.1 Commit (the running app, at `E`)

After the end tag and the CRC check out, the app writes COPY-A, KEEP and
COPY-B (read back). COPY-B carries the running image's size
(`__flash_image_end`) and its CRC-32: the value `READY` reports, read from
flash once per boot (the running image can't change while it runs: the
update writes only above it and page 124). It replies `SU DONE` and resets
~0.5 s later. If the marker write fails it replies `ERR 5` and stays on the
old app.

### 8.2 Boot (every boot, before `.data`/`.bss` exist)

`Reset_Handler` (in page 0) calls the boot hook (page 0), which copies the
copier into SRAM (`.ota_ram`, ~4.7 KB with its CRC table), points VTOR at a 16-entry table in SRAM
(its NMI handler forgives flash ECC errors, so a torn marker can't hang the
boot), and scans the marker log. Not pending → VTOR back, normal startup
(cost: microseconds). Pending →

The **assembled image** is the new image as the copy will leave it: kept
pages from their place in flash, every other page from staging. Its CRC-32
must equal COPY-A's before anything is erased.

1. **Not started (no START).** One pass over flash computes two CRCs, each
   byte read once: the **reflash guard**, CRC-32 of `[0, COPY-B size)` as
   flash holds it now (kept pages included: they are part of the old
   image), and the assembled image's CRC. Guard ≠ COPY-B's CRC → another
   tool reflashed the Core while the install was pending: write DONE
   (abandoned), boot what is there. (Same size and CRC as the new image → it
   is already that image: DONE (installed), boot on.) Assembled CRC wrong →
   DONE (abandoned). Then write START.
2. **Started (a cut interrupted the copy):** no guard (flash is legitimately
   half replaced). Destination CRC already the new image's → DONE
   (installed), boot on (a cut after the last write, before DONE). Assembled
   CRC wrong (kept pages are never touched, so this is staging) → stop
   without DONE (flash is a mix of two images; SWD recovers; not seen
   without a hardware fault).
3. **Copy:** interrupts off, IWDG fed, every page except 0 **that isn't
   kept** (erase, program quad-words, read back; a kept page is never
   erased). Then page 0: kept, or staged but equal byte for byte to what is
   there (the image's first 8 KB, 0xFF past its end: a full transfer of an
   image whose page 0 didn't change) → left alone. Otherwise page 0 last (its
   body first, its first quad-word, SP + reset vector, last). Check the CRC
   of the whole destination, write DONE, reset into the new app.

A resumed copy repeats step 3 page by page: kept pages hold their bytes
throughout, staged pages are rewritten from staging, which the copy never
touches.

The copier runs entirely from SRAM and touches nothing outside page 0, its
SRAM copy and the flash it is copying, so a half-copied body never runs.

### 8.3 What survives what

| interruption | result |
|--------------|--------|
| link drop / host gone during the transfer | old app keeps running; reconnect, `C`, `A`, `B` resumes |
| Core reset or power cut during the transfer | old app boots; the transfer starts again at 0 |
| power cut while the marker is written | the torn record doesn't check out: old app boots |
| power cut while START is written | it doesn't check out: the next boot runs the guard again (nothing erased yet) and copies |
| power cut while body pages are copied | next boot: page 0 (old, or unchanged) runs the copier again, which sees START and copies the non-kept pages again (idempotent: staging is intact and doesn't overlap; kept pages are never erased) |
| power cut after page 0 is written (or skipped), before DONE | next boot: the copier sees the image installed, writes DONE |
| power cut **while page 0 itself is erased / programmed** (~65 ms; only when the new page 0 differs) | **unbootable: the WBA has no empty-flash check (RM0493 §4, Table 22)**. Recover over SWD. With page 0 kept or unchanged there is no such window (host-tested: 0 unrecoverable of every cut point, §8.4) |
| another tool reflashes the Core while an install is pending, before it starts | the guard sees the running image changed: abandoned, the reflashed program runs |
| bad image (wrong chip, not a vector table, too big) | refused at `B` / chunk 0; nothing erased |
| wrong key / tampered image | `ERR 11` / `ERR 13`; nothing installed |
| new image crashes at boot | it's installed; recover over SWD (no rollback yet) |

Not yet: an option-byte boot stub (NSBOOTADD0) that would close the page-0
window and allow rollback. SWD stays the recovery.

### 8.4 Host tests

`make -C sdk/serial_update test` (`test/ota_sim.c`) cuts the power at every
flash operation of the install (sampled for the large images), for full
transfers with page 0 changed and unchanged and for kept-mostly delta
installs (one page staged, page 0 kept or changed), then boots again. Every
cut finishes the install on the next boot, kept pages are never erased, and
the only unbootable cuts are the ones inside page 0's own erase / program
when page 0 changes.

## 9. Timings (estimates; bench figures in the status note at the top)

Measured 2026-10-10 on a Core.ST.W5 (158,532-byte image, macOS Chrome): full
update 9.9 s transfer + 4.8 s install and reconnect; one-constant delta 4.2 s
transfer (20 frames, window 2).

| | |
|-|-|
| transfer | ~5-15 KB/s (stop-and-wait, 2 KB chunks, 15-30 ms connection interval, flash in radio windows): a 140 KB BLE app in ~10-30 s |
| per chunk on the Core | program 128 quad-words ≈ 15 ms (118 µs each, DS Table 69), erase 1.5 ms (2.4 ms max) per 8 KB, CMAC ≈ 10-15 ms at 32 MHz |
| `S`, kept page (8 KB, 32 MHz) | CMAC ≈ 40-60 ms (software AES, ≈ 2500-3000 cycles per 16-byte block), CRC ≈ 4 ms (table, ≈ 14 cycles per byte); no flash writes; 16 bytes on the air |
| `S`, copied (8 KB) | as a kept page, plus one staging page erased + programmed + read back ≈ 60-100 ms in radio windows |
| `E` | tag compare + CRC compare (accumulated during the transfer) + three marker records: a few ms. No flash is re-read |
| boot checks (16 MHz HSI) | one pass: word reads (≈ 2 cycles per byte) feeding the guard CRC and the assembled-image CRC (≈ 14 cycles per byte each, table in SRAM): ≈ 30 cycles per byte of image, 150 KB ≈ 0.3 s, 500 KB ≈ 1 s. Then the final CRC of the result, ≈ 16 cycles per byte: 150 KB ≈ 0.15 s |
| copy (no radio, 16 MHz HSI) | ≈ 64 ms per staged 8 KB page (kept pages cost nothing): one changed page ≈ 0.06 s, a full 140 KB image ≈ 1.2 s, the 496 KB maximum ≈ 4 s |
| one constant changed (155 KB, 18 of 19 pages kept) | transfer on the Core ≈ 1-1.3 s (18 kept pages ≈ 0.8-1.2 s, the staged page ≈ 0.11 s); install ≈ 0.3 s checks + 0.06 s copy + 0.15 s final CRC ≈ 0.5 s |
| from `SU DONE` to the new app advertising | ~0.5 s + copy + the app's own startup |

## 10. What Studio must change (web)

- `apps/studio/src/lib/stlink/flash-wba.ts`: `FLASH_APP_END_PAGE` 125 → **124**
  (and its comment: page 124 is the key + install marker page); the SWD flash
  sequence gains §7.2 steps 2-3 (read/keep or create the key record, erase
  page 124, write it back) on every W5 flash. Its tests follow the constant.
- Any W5 image-size limit of 1000 KB (1024000 B) → **992 KB (1015808 B)**.
- `apps/studio/src/lib/ble/contract.ts`: `STUDIO_RESERVED_IDS` gains
  `0x5c02` (better: reserve `0x5c00`-`0x5c0f`).
- A BLE update client (the protocol above) and a per-board key store
  (key id → K, created at provisioning).
- Delta updates (§3.1): keep the `.bin` last installed on each board (keyed by
  its CRC), send `S` runs when `READY` says `delta 1` and its CRC matches,
  fall back to a full `D` transfer after `ERR 13` / `ERR 8`.
