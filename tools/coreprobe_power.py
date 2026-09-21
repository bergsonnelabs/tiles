#!/usr/bin/env python3
"""
coreprobe_power.py — decide and apply target power before an SWD flash.

The CoreProbe can supply the board it is programming. Whether to supply, and
what, is declared by the project in config.json; it is never guessed, because a
board that already has a rail must never be fed:

    "probe": { "target_power": "5v" }

`target_power: "off"` (or an absent block) means self-powered: the probe
supplies nothing. That is the default, and what every project that never
thought about it means.

On current boards the only supply is 5 V. Every CoreProbe has been reworked to
two modes, self-powered or 5 V, by removing the 3V3 and 1V8 load switches
(2026-09-21); rev b brings them back. A 5 V target is expected to regulate its
own logic rail.

**The logic level is never declared and never set by the host.** The firmware
senses the target and sets the level shifter itself: its default V_shift mode
is AUTO, and DAP_CONFIG_CONNECT_SWD calls target_on_connect(), which senses
before it drives the bus. The rule is that the shifter is never set before the
target has been sensed. This tool only ever puts the firmware back into AUTO,
which matters because an explicit SET_VSHIFT pins the level until the probe
resets, and a pinned level is used on connect without re-sensing. An older
version of this tool set the level from `target_logic` before sensing, so a
probe it touched may still be pinned. `target_logic` is now ignored.

Studio applies the same rules; see apps/studio/src/lib/cmsisdap/coreprobe.ts in
the web repo. They are duplicated rather than shared because this file has to
run from a bare SDK checkout with no Node toolchain. Keep `decide()` in step
with Studio's `connect()`; the tests next door pin the behaviour here.

Requires hidapi:  pip install hidapi

    coreprobe_power.py status
    coreprobe_power.py apply --config path/to/config.json
    coreprobe_power.py apply --config … --dry-run
    coreprobe_power.py off
"""
import argparse
import json
import os
import sys
import time

VID, PID = 0x1209, 0xDA01

# DAP_Vendor commands (0x80 + index). See CoreProbe firmware/src/target.h.
VND_SET_VPAD = 0x80
VND_GET_VPAD = 0x81
VND_SENSE_MV = 0x82
VND_SET_VSHIFT = 0x83
VND_GET_VSHIFT = 0x84

VPAD = {"off": 0, "1v8": 1, "3v3": 2, "5v": 3}
VPAD_NAMES = {v: k for k, v in VPAD.items()}

# V_shift selection. The host only ever sends AUTO; the fixed values exist so
# GET_VSHIFT can be read back and reported. There is deliberately no function
# here that sets a fixed level.
VSHIFT_AUTO = 0x02
VSHIFT_NAMES = {0x00: "1v8 (pinned)", 0x01: "3v3 (pinned)", 0x02: "auto"}

# Supplies the probe cannot drive on current hardware, and why. Mirrors
# VPAD_UNAVAILABLE in Studio. Rev b refits these switches; when it lands, it
# should report what it can supply rather than have this table edited by hand.
UNAVAILABLE = {
    "1v8": (
        "The 1.8 V load switch has been removed from current CoreProbe boards. "
        "Power the target itself, or supply 5 V to a board that regulates it."
    ),
    "3v3": (
        "The 3.3 V load switch has been removed from current CoreProbe boards. "
        "Power the target itself, or supply 5 V to a board that regulates it."
    ),
}

# A probe that was NOT reworked still has all three switches, and giving it 5 V
# back-feeds its 3V3 and 1V8 rails to about 4.4 V through the disabled switches'
# body diodes. Nothing here can detect that: there is no sense on T.V+. It is
# also NOT caught after supplying. The sense reads about 1445 mV on such a
# probe, because PA1 driven above VDDA corrupts the ADC, and 1445 mV sits inside
# the 1V8 band, so it classifies as a valid 1V8 target. The only protection is
# that every unit is reworked.

# Sensed VIO classification, from CoreProbe docs/level-shifter.md. The sense
# reads SWDIO held up by the target's own internal pull-up, so a real target
# sits near 1V8 or near 3V3; anything between is a pull-up that is not properly
# powered and must be reported rather than rounded to the nearer class.
ABSENT_MAX_MV = 300
ONE_V8 = (1400, 2200)
THREE_V3 = (2800, 3600)


def classify(mv):
    if mv < ABSENT_MAX_MV:
        return "absent"
    if ONE_V8[0] <= mv <= ONE_V8[1]:
        return "1v8"
    if THREE_V3[0] <= mv <= THREE_V3[1]:
        return "3v3"
    return "anomalous"


def available_supplies():
    """What `target_power` may be set to on this hardware."""
    return [p for p in VPAD if p not in UNAVAILABLE]


class PowerError(Exception):
    """A refusal the operator has to resolve — never worked around silently."""


def decide(sensed_mv, declared_power):
    """What to do, given what we sensed and what the project declared.

    Pure, so the rules can be tested without a probe attached. Returns
    (supply, level). `supply` is None when the board powers itself. `level` is
    the sensed class for a self-powered board, and None when we are about to
    supply it, because a board's logic level can only be read once it is up.
    """
    cls = classify(sensed_mv)

    if cls != "absent":
        # Self-powered. Never feed a board that already has a rail; the
        # declaration is deliberately ignored here.
        if cls == "anomalous":
            raise PowerError(
                f"Target VIO reads {sensed_mv} mV, which is neither 1V8 "
                f"({ONE_V8[0]}-{ONE_V8[1]}) nor 3V3 ({THREE_V3[0]}-{THREE_V3[1]}). "
                "Check the target's own supply."
            )
        return None, cls

    choices = " | ".join(available_supplies())
    if not declared_power or declared_power == "off":
        raise PowerError(
            f"Nothing is driving the target's SWDIO ({sensed_mv} mV), so the board "
            "has no power of its own, and this project does not say what to supply.\n"
            "Add to config.json:\n"
            '  "probe": { "target_power": "5v" }\n'
            f"  target_power: {choices}"
        )

    if declared_power not in VPAD:
        raise PowerError(f"probe.target_power: '{declared_power}' is not one of {choices}")
    if declared_power in UNAVAILABLE:
        raise PowerError(f"Refusing to supply {declared_power}. {UNAVAILABLE[declared_power]}")
    return declared_power, None


def read_declaration(config_path):
    """The `probe` block from config.json, or empty when there is none.

    Returns (target_power, target_logic). `target_logic` is read only so a
    stale one can be reported; it no longer affects anything.
    """
    if not config_path or not os.path.isfile(config_path):
        return None, None
    with open(config_path, encoding="utf-8") as f:
        cfg = json.load(f)
    probe = cfg.get("probe") or {}
    return probe.get("target_power"), probe.get("target_logic")


# ---------- probe transport ----------


def _open():
    import hid  # imported late so --help works without hidapi installed

    d = hid.device()
    d.open(VID, PID)
    d.set_nonblocking(0)
    return d


def _cmd(dev, payload):
    dev.write(bytes([0x00]) + bytes(payload) + bytes(64 - len(payload)))
    return bytes(dev.read(64, 1500))


def sense_mv(dev):
    return int.from_bytes(_cmd(dev, [VND_SENSE_MV])[1:3], "little")


def set_vpad(dev, level):
    _cmd(dev, [VND_SET_VPAD, VPAD[level]])


def get_vpad(dev):
    return VPAD_NAMES.get(_cmd(dev, [VND_GET_VPAD])[1], "unknown")


def get_vshift(dev):
    return _cmd(dev, [VND_GET_VSHIFT])[1]


def restore_auto(dev):
    """Put the firmware back to sensing the target level itself.

    Sending AUTO re-senses immediately and sets V_shift from the reading, and
    undoes any level an earlier tool pinned. Returns the selection read back.
    """
    _cmd(dev, [VND_SET_VSHIFT, VSHIFT_AUTO])
    return get_vshift(dev)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    sub.add_parser("off")
    a = sub.add_parser("apply")
    a.add_argument("--config", required=True, help="path to the project's config.json")
    a.add_argument(
        "--dry-run",
        action="store_true",
        help="sense and report the decision without driving anything",
    )
    args = ap.parse_args()

    try:
        dev = _open()
    except Exception as e:
        print(f"CoreProbe not found ({e}). Is it plugged in?", file=sys.stderr)
        return 2

    try:
        if args.cmd == "off":
            set_vpad(dev, "off")
            print("target supply off")
            return 0

        if args.cmd == "status":
            mv = sense_mv(dev)
            sel = get_vshift(dev)
            print(f"sensed VIO : {mv} mV ({classify(mv)})")
            print(f"V_shift    : {VSHIFT_NAMES.get(sel, f'unknown ({sel})')}")
            print(f"V_pad      : {get_vpad(dev)}")
            if sel != VSHIFT_AUTO:
                print(
                    "note: V_shift is pinned, so the probe will not re-sense on connect. "
                    "`apply` puts it back to auto.",
                    file=sys.stderr,
                )
            return 0

        power, logic = read_declaration(args.config)
        if logic:
            print(
                f"note: probe.target_logic ({logic}) is ignored. The logic level is "
                "sensed by the probe, not declared; it is safe to delete.",
                file=sys.stderr,
            )

        # Clear any pinned level BEFORE anything else, while the target is still
        # unpowered. AUTO then senses nothing and settles on 1V8, the safe low,
        # so a freshly supplied target never meets a shifter left high.
        if not args.dry_run:
            restore_auto(dev)

        mv = sense_mv(dev)
        try:
            supply, level = decide(mv, power)
        except PowerError as e:
            print(f"\n{e}\n", file=sys.stderr)
            return 1

        if supply is None:
            print(f"target is self-powered ({mv} mV, {level}); supplying nothing")
            if args.dry_run:
                print("(dry run: nothing driven)")
            return 0

        print(f"target unpowered ({mv} mV); supplying {supply}")
        if args.dry_run:
            print("(dry run: nothing driven)")
            return 0

        set_vpad(dev, supply)
        time.sleep(0.3)
        after = sense_mv(dev)
        cls = classify(after)
        if cls in ("absent", "anomalous"):
            set_vpad(dev, "off")
            reason = (
                "the target never came up. Check the strap between probe and target."
                if cls == "absent"
                else "that is neither 1V8 nor 3V3, so the logic level cannot be set safely."
            )
            print(f"Supplied {supply}; SWDIO reads {after} mV, and {reason}", file=sys.stderr)
            return 1

        # The board is up. Hand the level back to the firmware to sense now.
        sel = restore_auto(dev)
        print(f"rail came up at {after} mV ({cls} logic); level shifter left to the probe ({VSHIFT_NAMES.get(sel, sel)})")
        return 0
    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())
