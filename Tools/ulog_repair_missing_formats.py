#!/usr/bin/env python3
"""Repair ULog files that reference a topic whose FORMAT definition is missing.

Background
----------
PX4's logger writes a FORMAT ('F') message for every subscribed topic, plus an
ADD_LOGGED ('A') subscription and the DATA ('D') messages. If a topic's *expanded*
format string overflows the logger's format buffer (see
src/modules/logger/messages.h: ulog_message_format_s.format[]), the logger skips
writing the FORMAT but still logs the subscription and data. The result is a
structurally-valid file with a dangling reference: pyulog raises
`KeyError: '<topic>'` and PlotJuggler crashes, even though nothing is byte-corrupted.

This was hit on the sun-tracker branch with `estimator_status_flags` (the largest
message in the system) once new topics (BatteryCharging, sun_tracker_status) shifted
the format batching past the buffer threshold. The proper fix is to enlarge the
logger buffer and reflash; this script recovers logs already captured by an affected
build.

What it does
------------
Each ULog message is self-delimited ([uint16 size][uint8 type][body]). This script
walks the file message-by-message, finds every topic that is subscribed ('A') but
has no FORMAT ('F'), and drops those subscriptions and their data ('D') messages.
Everything else is copied byte-for-byte, producing a valid log that loses only the
unparseable topic(s). Originals are never modified.

Usage
-----
    ./Tools/ulog_repair_missing_formats.py LOG.ulg [LOG2.ulg ...]
    ./Tools/ulog_repair_missing_formats.py *.ulg
    ./Tools/ulog_repair_missing_formats.py --out-dir /tmp/fixed LOG.ulg

Writes <name>.repaired.ulg next to each input (or into --out-dir). If pyulog is
installed it verifies each repaired file parses.
"""

import argparse
import os
import struct
import sys

ULOG_MAGIC = b"\x55\x4c\x6f\x67\x01\x12\x35"  # 'ULog' + 0x01 0x12 0x35
HEADER_LEN = 16  # 8-byte magic/version + 8-byte timestamp


def _iter_messages(data):
    """Yield (offset, size, type_char, body) for each ULog message after the header."""
    off = HEADER_LEN
    n = len(data)
    while off + 3 <= n:
        msg_size, msg_type = struct.unpack("<HB", data[off:off + 3])
        body = data[off + 3:off + 3 + msg_size]
        if len(body) < msg_size:  # truncated tail
            break
        yield off, msg_size, chr(msg_type), body
        off += 3 + msg_size


def repair(path, out_dir=None):
    data = open(path, "rb").read()
    if not data.startswith(ULOG_MAGIC):
        raise ValueError(f"{path}: not a ULog file (bad magic)")

    # Pass 1: collect formats present and subscriptions (name -> set of msg_ids).
    formats = set()
    sub_ids = {}  # name -> {msg_id, ...}
    for _off, _sz, t, body in _iter_messages(data):
        if t == "F":
            formats.add(body.decode("latin1").split(":", 1)[0])
        elif t == "A":  # multi_id(1) + msg_id(2) + name
            msg_id = struct.unpack("<H", body[1:3])[0]
            name = body[3:].decode("latin1")
            sub_ids.setdefault(name, set()).add(msg_id)

    missing = {n: ids for n, ids in sub_ids.items() if n not in formats}
    if not missing:
        print(f"{path}: no missing formats — nothing to repair")
        return None

    bad_ids = set().union(*missing.values())

    # Pass 2: copy every message except A/D referencing a bad msg_id.
    out = bytearray(data[:HEADER_LEN])
    dropped_a = dropped_d = 0
    for off, sz, t, body in _iter_messages(data):
        rec = data[off:off + 3 + sz]
        if t == "A" and struct.unpack("<H", body[1:3])[0] in bad_ids:
            dropped_a += 1
            continue
        if t == "D" and len(body) >= 2 and struct.unpack("<H", body[0:2])[0] in bad_ids:
            dropped_d += 1
            continue
        out += rec

    base = os.path.basename(path)
    stem = base[:-4] if base.endswith(".ulg") else base
    out_name = stem + ".repaired.ulg"
    dest_dir = out_dir or os.path.dirname(path) or "."
    os.makedirs(dest_dir, exist_ok=True)
    out_path = os.path.join(dest_dir, out_name)
    open(out_path, "wb").write(out)

    miss_desc = ", ".join(f"{n}{sorted(ids)}" for n, ids in missing.items())
    print(f"{path}: stripped {miss_desc} | dropped A={dropped_a} D={dropped_d} | "
          f"{len(data)}->{len(out)} bytes -> {out_path}")
    return out_path


def verify(path):
    try:
        import pyulog
    except ImportError:
        return None
    u = pyulog.ULog(path)
    samples = sum(len(d.data["timestamp"]) for d in u.data_list)
    print(f"  verified: {len(u.data_list)} topics, {samples} samples")
    return True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="ULog file(s) to repair")
    ap.add_argument("--out-dir", help="output directory (default: next to each input)")
    ap.add_argument("--no-verify", action="store_true", help="skip pyulog parse check")
    args = ap.parse_args(argv)

    rc = 0
    for path in args.logs:
        try:
            out_path = repair(path, args.out_dir)
            if out_path and not args.no_verify:
                verify(out_path)
        except Exception as e:  # noqa: BLE001 - report and continue
            print(f"{path}: ERROR {e}", file=sys.stderr)
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
