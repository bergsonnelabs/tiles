# Thread provisioning over BLE (Core.ST.W5)

Read a Core.ST.W5's Thread status from a phone and move it to another Thread
network, so nodes no longer need the network baked into `config.json`. One
image runs BLE and Thread together (`"ble"` and `"thread"` both enabled); the
phone talks to a GATT service, the Core applies the new network, stores it and
re-attaches.

> **DEVELOPMENT ONLY. Writes are open.** There is no pairing and no
> authorization: any phone in range that connects can read the status and move
> the node to another network (and so off yours). It cannot read the network
> key back. The application can gate writes by defining
> `core_thread_prov_allow_write()` (§7); pairing-protected writes are planned.
> Do not ship a product with this service enabled as it is.

Status: firmware and host-side tests in tiles (2026-10-12). **Not yet run on a
board.** Everything about the radio side (attach after apply, persistence
across a power cycle and a BLE update, notifications) is unverified.

- Firmware: `sdk/thread/thread_prov_ble.c` (the GATT service),
  `sdk/core/core_thread.c` (apply, factory reset, boot precedence),
  `sdk/thread/thread_prov_codec.c` (byte layouts, validation, the precedence
  rule; plain C).
- Host tests (no hardware): `make -C tests/host-thread-prov`. They pin the
  worked example in §6.
- C API (the same, without BLE): `core_thread.h`, §8.

## 1. Enabling it

```json
"ble":    { "enabled": true },
"thread": { "enabled": true, "channel": 15, "network_key": "…", …,
            "ble_provisioning": true }
```

- `"ble_provisioning": true` adds the service and implies `"provisioning"`.
  It needs `"ble"` in the same image (coregen refuses it otherwise). Default
  off.
- `"provisioning": true` alone (Thread with or without BLE) turns on the
  precedence rule (§5) and the C API (§8), without the BLE service.
- coregen writes `THREAD_CFG_PROVISIONING` / `THREAD_CFG_BLE_PROVISIONING` into
  `coregen/thread_dataset.h`. Builds without them compile exactly as before.
- Nothing to call: `core_ble_init()` registers the service, and
  `core_thread_process()` (which `core_delay_ms()` and `core_watchdog_feed()`
  also run in Thread builds) handles it.

## 2. GATT

One custom service in the SDK's base UUID `0000xxxx-8e22-4541-9d4c-21edae82ed19`,
registered by `core_ble` after Studio Link (so it shifts no other handle).

| what | 16-bit id | UUID | properties | length |
|------|-----------|------|------------|--------|
| service Thread Provisioning | `0x5D00` | `00005d00-8e22-4541-9d4c-21edae82ed19` | primary | |
| Thread status | `0x5D01` | `00005d01-8e22-4541-9d4c-21edae82ed19` | read, notify | 39 |
| Thread dataset | `0x5D02` | `00005d02-8e22-4541-9d4c-21edae82ed19` | write, write without response | 58 |
| Thread control | `0x5D03` | `00005d03-8e22-4541-9d4c-21edae82ed19` | write, write without response, read, notify | write 1, read 4 |

- Each characteristic has a User Description ("Thread status", "Thread
  dataset", "Thread control").
- The ids sit next to Studio Link's `0x5C00`-`0x5C0F`. **Hosts should reserve
  `0x5D00`-`0x5D0F`** (no user contract may use them), as they do `0x5C00`-`0x5C0F`.
  Project contracts allocate from `0x0001` upward, and auto-numbered services
  use `0xB000`-`0xB3FF`, so neither reaches this block.
- **ATT MTU.** The dataset is one 58-byte write: the MTU must be at least 61.
  The Core starts an MTU exchange itself after connecting (offering 251), so
  phones get there without asking. A "long write" split by a central into
  pieces arrives as separate fragments and is refused (`ERR_LENGTH`).
  Notifications carry the first MTU-3 bytes; a status read is complete at any
  MTU (read blob).
- If the application calls `core_ble_enable_pairing()`, every characteristic,
  this service included, requires an encrypted link.
- Integers are little-endian. The ext PAN ID, mesh-local prefix and network
  key are byte strings in the order they are written down: `dead00beef00cafe`
  is `de ad 00 be ef 00 ca fe`.

## 3. Status (`0x5D01`, read + notify, 39 bytes)

| offset | size | field |
|-------:|-----:|-------|
| 0 | 1 | version, `0x01` (`0x00`: no status yet, right after boot) |
| 1 | 1 | role: 0 disabled, 1 detached, 2 child, 3 router, 4 leader |
| 2 | 1 | flags: bit 0 provisioned (the network is a stored one, not config.json's), bit 1 a valid dataset is staged, bit 2 Thread running (0 before start and after LEAVE), bit 3 writes allowed now (`core_thread_prov_allow_write()`); bits 4-7 0 |
| 3 | 1 | channel (11-26; 0 before Thread starts) |
| 4 | 2 | PAN ID |
| 6 | 2 | RLOC16 (`0xFFFE`: not attached) |
| 8 | 4 | partition ID (0: not attached) |
| 12 | 1 | neighbor count |
| 13 | 1 | parent RSSI, signed dBm (127: no parent, i.e. not a child) |
| 14 | 8 | extended PAN ID |
| 22 | 1 | network name length, 0-16 |
| 23 | 16 | network name, UTF-8, zero padded |

The channel, PAN ID, ext PAN ID and name are the network the node is set to
(its active dataset), attached or not. The first 20 bytes hold everything that
changes, so a notification at the default MTU still carries them.

Updated when OpenThread reports a change (role, partition, channel, PAN ID,
name, ext PAN ID, dataset), after every write, and once a second for the
neighbor count and RSSI; notified only when the bytes differ.

## 4. Writing a network

### 4.1 Dataset (`0x5D02`, write only, 58 bytes, version 1)

| offset | size | field | rule |
|-------:|-----:|-------|------|
| 0 | 1 | version | `0x01` |
| 1 | 1 | flags | `0x00` (reserved) |
| 2 | 1 | channel | 11-26 |
| 3 | 1 | network name length | 1-16 |
| 4 | 2 | PAN ID | `0x0000`-`0xFFFE` (`0xFFFF` is broadcast) |
| 6 | 8 | extended PAN ID | any |
| 14 | 8 | mesh-local prefix (first 8 bytes of the /64) | any (Thread expects a ULA, `fd…`) |
| 22 | 16 | network key | any |
| 38 | 4 | Active Timestamp, seconds | 0 means 1, the value every config.json network uses. Joining an existing network formed elsewhere: send that network's timestamp |
| 42 | 16 | network name | UTF-8, no NUL within the length; bytes past it are ignored |

A write only **stages** the dataset (in RAM): nothing changes until APPLY.
The result (§4.3, op `0x10`) says whether it was accepted. A refused write
clears whatever was staged before. The network key is never readable: the
characteristic has no read property, the Core overwrites its stored value
with zeros as soon as it has copied it, and the staged copy is wiped after
APPLY (applied or refused), CLEAR, or the next write.

### 4.2 Commands (`0x5D03`, write 1 byte)

| byte | command | does |
|-----:|---------|------|
| `0x01` | APPLY | make the staged dataset the network: stop Thread, set it active, mark it provisioned, start Thread (which attaches within seconds), store both (§5). `ERR_NOTHING_STAGED` without a valid staged dataset. Re-applying the network the node is already on writes nothing and does not detach |
| `0x02` | FACTORY_RESET | forget the stored network: back to config.json's, re-attach (also after LEAVE) |
| `0x03` | LEAVE | stop Thread (role 0). The network is kept; ATTACH or a reboot attaches again |
| `0x04` | ATTACH | start Thread again after LEAVE |
| `0x05` | CLEAR | drop the staged dataset |

### 4.3 Result (`0x5D03`, read + notify, 4 bytes)

| offset | field |
|-------:|-------|
| 0 | op: `0x00` nothing yet, `0x10` a dataset write, else the command byte |
| 1 | result code (below) |
| 2 | sequence: +1 per result (wraps), so a fresh result is told from an old one |
| 3 | detail: for `ERR_FAILED` the low byte of OpenThread's `otError` (`openthread/error.h`), else 0 |

Subscribe to notifications on `0x5D03` before writing, or read it after. One
result per write, in order.

| code | name | meaning |
|-----:|------|---------|
| `0x00` | OK | dataset staged / command done |
| `0x01` | ERR_LENGTH | wrong number of bytes (dataset not 58, command not 1) |
| `0x02` | ERR_VERSION | dataset byte 0 not `0x01` |
| `0x03` | ERR_RESERVED | dataset flags byte not 0 |
| `0x04` | ERR_CHANNEL | channel outside 11-26 |
| `0x05` | ERR_PANID | PAN ID `0xFFFF` |
| `0x06` | ERR_NAME | name length outside 1-16, or a NUL byte within it |
| `0x07` | ERR_NOTHING_STAGED | APPLY with no valid staged dataset |
| `0x08` | ERR_COMMAND | unknown command byte |
| `0x09` | ERR_DENIED | `core_thread_prov_allow_write()` refused the write |
| `0x0A` | ERR_STATE | Thread not started (`core_thread_start()` has not run or failed) |
| `0x0B` | ERR_FAILED | OpenThread refused; detail = `otError` |

### 4.4 How APPLY works

1. Validate the staged dataset (already checked when staged).
2. Build the operational dataset: OpenThread's defaults for what the blob does
   not carry (channel mask, security policy, from `otDatasetCreateNewNetwork`),
   the blob's fields, no PSKc (as config.json networks).
3. Already on it, mark set, Thread running: done, nothing written.
4. Otherwise: `otThreadSetEnabled(false)`, `otDatasetSetActive()`, read it back
   (a settings buffer that could not take it fails here), set the provisioned
   mark, `otThreadSetEnabled(true)`. This is OpenThread's own
   `thread stop` / `dataset commit active` / `thread start` sequence.
5. Any refusal: the previous dataset and mark are put back and Thread is
   restarted on the old network; the result is `ERR_FAILED` with the `otError`.
6. The dataset and the mark are both OpenThread settings, written to flash
   (pages 122-123) in one snapshot 250 ms later, so a power cut leaves either
   both old or both new.

A new network key resets the key sequence and frame counters (OpenThread's
`KeyManager::SetNetworkKey`); neighbors learn the new counters when the node
attaches. A node moved to a network nobody else is on forms its own partition
and becomes leader.

## 5. Precedence (which network a node runs at boot)

- **The stored (provisioned) network wins.** After APPLY (or
  `core_thread_set_dataset()`) the node keeps that network across reboots,
  power cuts and BLE firmware updates (the settings pages are outside the BLE
  update's staging area), whatever the new image's config.json says.
- **config.json's network is the factory default**, used while nothing is
  provisioned: a node that was never provisioned follows config.json, and a
  reflash with a different config.json moves it (written only when it
  differs, so an unchanged boot writes no flash).
- **FACTORY_RESET** (or `core_thread_factory_reset()`) clears the mark and
  makes config.json's network active again, then re-attaches.
- A stored mark whose dataset is missing or incomplete (corrupt settings):
  config.json's network, and the mark is cleared.
- An image built **without** `"provisioning"` ignores the mark: config.json's
  network wins, as before this feature. (The mark stays in flash; a later
  provisioning image would then keep whatever network is active. Factory
  reset clears it.)

Rule, as code (`thread_prov_boot_action()`, host-tested):

| mark | stored dataset complete | equals config.json's | boot does |
|:----:|:-----------------------:|:--------------------:|-----------|
| yes | yes | either | keep the stored network |
| yes | no | - | config.json's, clear the mark |
| no | yes | yes | nothing (no flash write) |
| no | yes | no | config.json's |
| no | no | - | config.json's |

## 6. Worked example: move a node with nRF Connect

Target network: name `lab-net`, channel 20, PAN ID `0xBEEF`, ext PAN ID
`1122334455667788`, mesh-local prefix `fd11:2233:4455:6677::/64`, network key
`0f1e2d3c4b5a69788796a5b4c3d2e1f0`, timestamp 0 (= 1).

1. Connect to the Core (e.g. `Core.ST.W5 Thread` for tests/hw-ble-thread).
   On Android, if writes are refused with a length error, request MTU 247
   from the menu first.
2. Open service `00005D00-…`. Enable notifications on `0x5D03` and `0x5D01`.
3. Write to `0x5D02` (byte array, write request), 58 bytes:

   ```
   01 00 14 07 EF BE 11 22 33 44 55 66 77 88 FD 11
   22 33 44 55 66 77 0F 1E 2D 3C 4B 5A 69 78 87 96
   A5 B4 C3 D2 E1 F0 00 00 00 00 6C 61 62 2D 6E 65
   74 00 00 00 00 00 00 00 00 00
   ```

   (version 1, flags 0, channel `0x14`, name length 7, PAN ID `EF BE`, ext PAN
   ID, prefix, key, timestamp 0, `lab-net` and 9 bytes of padding.) As one
   hex string: `01001407efbe1122334455667788fd112233445566770f1e2d3c4b5a69788796a5b4c3d2e1f0000000006c61622d6e6574000000000000000000`.

   `0x5D03` notifies `10 00 01 00`: dataset write, OK, sequence 1. The status
   flags gain bit 1 (staged).

4. Write `01` to `0x5D03` (APPLY). It notifies `01 00 02 00`. The status shows
   role 1 (detached), channel 20, PAN ID `EF BE`, `lab-net`, flags bit 0
   (provisioned); after a few seconds role 4 (leader) if it is alone, or 2 / 3
   once other `lab-net` nodes are around.
5. A provisioned router, as read from `0x5D01` (39 bytes):

   ```
   01 03 0D 14 EF BE 00 44 78 56 34 12 02 7F 11 22
   33 44 55 66 77 88 07 6C 61 62 2D 6E 65 74 00 00
   00 00 00 00 00 00 00
   ```

   (version 1, router, flags provisioned + running + writable, channel 20,
   PAN ID `0xBEEF`, RLOC16 `0x4400`, partition `0x12345678`, 2 neighbors, no
   parent, ext PAN ID, `lab-net`.)
6. Back to config.json's network: write `02` to `0x5D03` (`02 00 03 00`).

## 7. The write gate

```c
/* Default in the SDK (weak): any connected phone may write. */
int core_thread_prov_allow_write(void) { return 1; }
```

Define it in the application to decide per write (dataset writes and
commands; reading the status stays open). It runs from `core_thread_process()`
in the main loop, never in an interrupt. For example, accept writes only for
30 s after a button press, or only while a bonded central is connected. A
refused write answers `ERR_DENIED`, and status flag bit 3 shows the current
answer.

## 8. The same from C (no BLE needed)

With `"provisioning": true` (or `"ble_provisioning"`), `core_thread.h`:

| function | does |
|----------|------|
| `core_thread_set_dataset(&ds)` | APPLY with a `core_thread_dataset_t` (fields as §4.1; `network_name` NUL-terminated). `CORE_THREAD_ERR_ARG` for a bad dataset |
| `core_thread_factory_reset()` | FACTORY_RESET |
| `core_thread_is_provisioned()` | 1 when a stored network is in use, 0 for config.json's |
| `core_thread_get_network(&ds)` | the network the node is set to, network key zeroed (any Thread build) |
| `core_thread_set_enabled(0 / 1)` | LEAVE / ATTACH (any Thread build) |

Without `"provisioning"`, `set_dataset` and `factory_reset` return
`CORE_THREAD_ERR_UNSUPPORTED` and `is_provisioned` returns 0.
