#!/usr/bin/env python3
"""Bench test: prove multi-tag registration works off the 5008 stream, no flight.

Binds the same UDP 5008 AprilTag debug stream that host_tag_registry.py uses and
applies the SAME gates (id range, hamming, margin, N-frames-to-confirm). It does
NOT need telemetry or flight — it confirms tag IDs, which is what the claim latch
in the firmware was blocking. Hold tags in front of the camera and watch them
register. If several register here, send_trajectory_no_flash.py will capture them
in flight too.

Run ONE 5008 listener at a time: stop tag_debug.py / tag_stream.py /
send_trajectory_no_flash.py before running this.

Usage:
    python tag_registry_monitor.py --drone-id 9 --frames 2 --id-margin 15
"""

import argparse
import math
import socket
import time

from protocol import parse_at_debug


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--port", type=int, default=5008)
    ap.add_argument("--frames", type=int, default=3,
                    help="Frames a tag must be seen before it registers (host_tag_registry default 3)")
    ap.add_argument("--id-margin", type=float, default=25.0,
                    help="Min decision margin (host_tag_registry default 25)")
    ap.add_argument("--reset-gap", type=float, default=1.0,
                    help="Frame counter resets if a tag is unseen this long (s)")
    ap.add_argument("--id-min", type=int, default=0)
    ap.add_argument("--id-max", type=int, default=21)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", args.port))
    except OSError as exc:
        raise SystemExit(f"UDP {args.port} busy ({exc}). Stop tag_debug.py / "
                         f"tag_stream.py / send_trajectory_no_flash.py first.")
    sock.settimeout(0.5)

    print(f"Listening on UDP {args.port} for drone {args.drone_id}. "
          f"frames={args.frames} id-margin={args.id_margin} ids={args.id_min}-{args.id_max}")
    print("Hold tags in front of the camera. Ctrl+C to stop.\n")

    hits = {}          # id -> consecutive-ish frame count
    last_seen = {}     # id -> monotonic
    registered = {}    # id -> dict(count, last_margin, last_dist)
    last_frame = None
    seen_in_stream = set()   # every id that appeared at all, even if gated out

    try:
        while True:
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            pkt = parse_at_debug(data)
            if pkt is None or pkt.drone_id != args.drone_id:
                continue
            # firmware repeats the last frame at 10 Hz; count each once
            if not pkt.frame_ms or pkt.frame_ms == last_frame:
                continue
            last_frame = pkt.frame_ms
            now = time.monotonic()

            seen_this_frame = set()
            for d in pkt.detections:
                if d.id in seen_this_frame:
                    continue
                seen_this_frame.add(d.id)
                if d.id not in seen_in_stream:
                    seen_in_stream.add(d.id)
                    # report what the RAW stream shows, before gates — so you can
                    # see tags that are visible but being rejected and why
                    why = []
                    if not args.id_min <= d.id <= args.id_max:
                        why.append(f"id out of {args.id_min}-{args.id_max}")
                    if d.hamming > 1:
                        why.append(f"hamming={d.hamming}>1")
                    if not math.isfinite(d.margin) or d.margin <= args.id_margin:
                        why.append(f"margin={d.margin:.0f}<= {args.id_margin:.0f}")
                    tag = f"seen tag {d.id} (margin={d.margin:.0f}, dist={d.tz:.2f}m)"
                    print(tag + ("  REJECTED: " + ", ".join(why) if why else "  -> counting"))

                # apply the host_tag_registry gates
                if (not args.id_min <= d.id <= args.id_max or d.hamming > 1
                        or not math.isfinite(d.margin) or d.margin <= args.id_margin):
                    continue
                if d.id not in registered and now - last_seen.get(d.id, -100) > args.reset_gap:
                    hits[d.id] = 0
                last_seen[d.id] = now
                hits[d.id] = hits.get(d.id, 0) + 1
                if hits[d.id] >= args.frames and d.id not in registered:
                    registered[d.id] = True
                    print(f"\n*** REGISTERED tag {d.id}  "
                          f"(margin={d.margin:.0f}, dist={d.tz:.2f}m, frames={hits[d.id]}) ***")
                    print(f"    total registered so far: {sorted(registered)}\n")
    except KeyboardInterrupt:
        print("\n" + "=" * 50)
        print(f"Tags that appeared in the stream : {sorted(seen_in_stream)}")
        print(f"Tags that REGISTERED             : {sorted(registered)}")
        missed = sorted(seen_in_stream - set(registered))
        if missed:
            print(f"Seen but NOT registered          : {missed}")
            print("  -> either gated out (see REJECTED lines) or not held long "
                  f"enough to reach {args.frames} frames. Lower --frames or --id-margin.")
        print("=" * 50)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
