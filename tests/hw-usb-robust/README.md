# hw-usb-robust

Bench test for the USB CDC stack on **Core.ST.L4** (`sdk/hal/hal_usb_cdc.c`,
STM32L422 branch) and for reaching a Core that sleeps in `core_stop_for()`.
The Core answers commands over CDC; `host_test.py` (python3 + pyserial) drives
it and scores the result.

```sh
make && make flash-serial        # or: make flash-dfu (ROM-DFU layout, "bootloader": "rom")
python3 host_test.py             # robustness, ~45 s
python3 host_test.py sleep       # sleep + reachability, ~15 s
```

Never build this with `BOOTLOADER := 1`.

Core.ST.H5 runs the same robustness checks: `make TILE=Core.ST.H5`, then
`TILE=Core.ST.H5 python3 host_test.py --no-reset`. `--no-reset` restarts the
Core for the `early` check with `make flash-serial` instead of a reset, for a
board strapped BOOT0-high (every reset lands in the ROM bootloader). 7/7 on
2026-09-26. The sleep mode and `W` are L4-only (the H5's Stop path doesn't
consult USB yet).

## Robustness (`host_test.py`)

| Check | What passes |
|-------|-------------|
| early  | after a reset, the first line the terminal sees is the `EARLY` line the Core printed right after `core_init()`, with no startup delay |
| serial | the USB serial number is the chip's 96-bit UID (24 hex digits), not `000001` |
| zlp    | one write of exactly 64 bytes, and of 128, reaches the host without a following write |
| burst  | 4096 bytes sent at once to a Core that reads 16 B/ms arrive intact (the 256 B ring fills; the endpoint NAKs until there is room) |
| dtr    | lines written while DTR is low arrive after it rises, none leak while it is low, and those writes don't wait |
| stall  | the host stops reading for 30 s while the Core prints flat out: the slowest write is at most the 50 ms bound (`HAL_USB_CDC_TX_TIMEOUT_MS`) |

## Sleep + reachability (`host_test.py sleep`)

The Core loops `core_stop_for(60)` and prints a counter. While it sleeps:

- `pong`: a line sent to it gets `PONG` from its USB interrupt;
- `flash`: `make flash-serial` reflashes it, no replug, no BOOT0;
- `back`: the CDC port comes back with the new image's `EARLY` line.

Each `WOKE` line reports `cyc_per_ms`: the DWT cycle counter runs in Sleep
mode and stops in Stop, so a value near `SYSCLK/1000` means the wait was in
Sleep mode (a USB host awake on the bus), a small one that it was in Stop.
macOS never suspends the Core's bus, so on a Mac every wait is in Sleep mode.

## W: the USB wakeup from Stop

`W <n>` (port closed; results queue until it reopens) runs n x
`core_stop_for(1)`, each with its Stop taken as if the bus were suspended
(test hook, `-DHAL_USB_CDC_TEST_HOOKS`, set in this Makefile). With nothing but
the RTC to end Stop, it would last the whole second; a Stop that ends after
~1 ms with `wakes=1` was ended by the USB wakeup on EXTI line 17 (vector 67),
after which the clocks came back and USB kept working. This proves the wake
chain, not the trigger: with a live host, SOF traffic only sometimes looks
like resume to the wake detector (RM0394 §46.5.5), which is why `W` must not
run with the port open (unanswered IN tokens make macOS reset the port). A
real host resume (20 ms of K) or reset is the trigger in normal use.

## Other commands

`I` identity, `S <s>` stall run, `R` stall result, `B <n>` burst, `Z <n>`
exact-size write, `D <k>` DTR cycle, `L` DTR history, `V` hello, `X` reset,
`P <s>` sleep mode (in sleep mode, `X` resets).
