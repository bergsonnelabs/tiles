# Scope protocol

The byte stream a Core sends so a host can plot its variables live. One page
on purpose: a decoder is about a hundred lines in any language, and the stream
describes itself, so a host needs no project file, no ELF and no setup.

- Device side: `sdk/core/core_scope.h` (the only encoder).
- Reference decoder: `tools/scope_decode.py`. Every other decoder (Studio's
  `scopeProtocol.ts`, MATLAB, Swift, …) should agree with it on the example
  frames at the bottom of this page.
- Tested end to end on the host by `tools/test_core_scope.py`.

All integers are little-endian. Floats are IEEE-754 binary32.

## Frame

```
A5 53 <type> <seq> <len> <payload: len bytes> <crc8>
```

| Field | Bytes | |
|---|---|---|
| magic | 2 | `A5 53` |
| type | 1 | Frame type, below. |
| seq | 1 | Counts frames, wraps 255 → 0. |
| len | 1 | Payload length, 0–255. |
| payload | len | |
| crc8 | 1 | CRC-8, polynomial `0x07`, initial value 0, no reflection, no final XOR, over `type`, `seq`, `len` and the payload (not the magic). |

A frame is at most 261 bytes.

**The stream may carry other bytes between frames** — on USB it is the same
CDC port `print` writes to. A decoder scans for `A5 53`, and accepts a frame
only if the type is one it knows *and* the CRC matches; otherwise it hands the
`A5` back as ordinary text and resumes one byte later. Bytes that are not part
of a valid frame are text, in order. On a link that carries nothing else (BLE),
such bytes are a damaged frame and are discarded.

**A decoder must ignore frame types it does not know** (treat as above: not a
frame). New types are how the protocol grows.

Frames are a byte stream, not packets: a BLE notification or a USB read may
hold several frames, or part of one. Reassemble across reads.

## 0x01 — schema (the channel list)

```
schema_id u8, count u8, then count × { type u8, name_len u8, name[name_len] }
```

Sent when a host starts listening, and again every second, so a host that
attaches mid-run learns the channels from the device within a second.

`schema_id` is the CRC-8 (same polynomial) of the payload from `count` to the
end. Different channel lists get different ids. On a new id a host discards
what it has plotted: the firmware changed.

`name` is UTF-8, not terminated, at most 63 bytes. Array elements arrive as
separate channels named `base[0]`, `base[1]`, … At most 32 channels.

| type | Value | Bytes in a row |
|---|---|---|
| 0 | int32 | 4 |
| 1 | float32 | 4 |
| 2 | bool, as int32 0 / 1 | 4 |
| 3 | uint32 | 4 |
| 4 | int16 | 2 |
| 5 | uint16 | 2 |
| 6 | int8 | 1 |
| 7 | uint8 | 1 |

A **row** is one value per channel, in schema order, packed with no padding.
A schema containing a type code the decoder does not know must be rejected
whole (its row layout cannot be computed); data for that `schema_id` is then
ignored.

## 0x02 — data (one timestamped sample)

```
schema_id u8, millis u32, row
```

`millis` is the device's millisecond clock when the sample was taken. It wraps
after 49.7 days. Plot against it, not against arrival time: links deliver in
bursts. A `millis` more than a second *behind* the last one means the device
rebooted.

Ignore a data frame whose `schema_id` is not the current schema's, or whose
length is not `5 + row bytes`.

**Loss.** A gap in `seq` means samples were lost — dropped on the device
because the link could not keep up (the device skips `seq` by the number it
dropped, up to 255), or lost in transit.

## 0x03 — rows (samples at a declared rate)

```
schema_id u8, n0 u32, rate_mHz u32, row × N        N = (len − 9) / row bytes, N ≥ 1
```

For data sampled by a timer or a sensor's data-ready interrupt, where the
firmware has declared the rate (`core_scope_declare_rate_hz`). Rows carry no
timestamps; row *k* of the frame is sample number `n0 + k`, taken at

```
t = (n0 + k) × 1000 / rate_mHz   seconds
```

`rate_mHz` is samples per 1000 seconds (1 kHz = 1 000 000), so rates that are
not whole hertz stay exact. `n` counts every sample the firmware took since
the host attached, including ones it had to drop, and wraps at 2³².

**Loss** is a jump in `n`: if a frame's `n0` is not the previous frame's
`n0 + N`, the difference is the number of samples dropped, and the time axis
keeps its true width. (`seq` still counts frames, but do not count loss from
it in this mode — `n` is exact.)

A stream uses 0x02 or 0x03 for a given schema, not both at once.

## Reserved

Type codes for later, so nobody else takes them. None are sent today.

| type | Purpose |
|---|---|
| 0x04 | Channel metadata: unit and scale per channel (the SDK is integer-only, so a channel in mdeg wants to plot in deg). |
| 0x05 | Text: `print` output for links with no text path of their own (BLE). |
| 0x10–0x1F | Host → device control: start, stop, rate, channel selection. |

## Links

| Link | Carries | Notes |
|---|---|---|
| USB CDC | frames + print text | Any baud setting; DTR must be asserted (opening the port does it) or the device sends nothing. Do not open at 1200 baud: closing such a port reboots the Core into its bootloader. |
| Bluetooth LE | frames only | Service **Studio Link** `00005c00-8e22-4541-9d4c-21edae82ed19`, characteristic **Studio Scope** `00005c01-8e22-4541-9d4c-21edae82ed19`, notify only. Subscribe to start the stream. Notifications are up to 180 bytes; negotiate an ATT MTU of at least 183. Frames are batched: expect a few per notification, every 15–50 ms. |

Nothing is sent while nobody is listening, and the schema frame always comes
first when somebody starts.

## Example frames

Three channels — `distance_mm` int32, `temp_c` float32, `pressed` bool —
schema id `F5`, then one sample at 1005 ms: −1234, 21.5, true.

```
a5 53 01 00 20  f5 03  00 0b 64 69 73 74 61 6e 63 65 5f 6d 6d
                       01 06 74 65 6d 70 5f 63
                       02 07 70 72 65 73 73 65 64              0a
a5 53 02 01 11  f5  ed 03 00 00  2e fb ff ff  00 00 ac 41  01 00 00 00   21
```

A rows frame: schema `20` (four int16 channels, 8-byte rows), first sample
number 0, 1 kHz (`40 42 0f 00` = 1 000 000 mHz), one row: 0, 0, 0, 7.

```
a5 53 03 01 11  20  00 00 00 00  40 42 0f 00  00 00 00 00 00 00 07 00   12
```
