"""
The session logger: record AC telemetry to CSV for the spin early-warning model.

Run it (after `pip install -e .`, on the Windows machine running AC, with the
car on track):

    python -m driftscore.logger --kind natural --setup "TC off, 2nd gear"

One row per new physics packet, only while AC is live -- never paused or in
replay -- to data/raw/YYYY-MM-DD_HHMMSS.csv, plus a .json with car, track and
setup. CSV rather than Parquet keeps DriftScore stdlib-only, and a file
flushed every second survives a crash.
"""

import argparse
import bisect
import csv
import json
import platform
import sys
import time
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path

from driftscore.signals import SLIP_SPEED_FLOOR_KMH, slip_angle_deg
from driftscore.telemetry import (
    AC_LIVE,
    GRAPHICS_MEMORY_NAME,
    STATIC_MEMORY_NAME,
    ACGraphicsPrefix,
    ACStaticPrefix,
    ACStatus,
    open_page,
    open_physics,
    read_page,
    read_physics,
    wchars_to_str,
)


# All three axes of angular velocity and accG: which one is yaw or lateral
# gets decided offline, so a wrong guess never costs a re-recording. Wheels
# are in AC's order (FL, FR, RL, RR); logcheck confirms it on live data.
COLUMNS = [
    "t", "packet_id",
    "speed_kmh", "vel_x", "vel_y", "vel_z", "heading",
    "ang_vel_x", "ang_vel_y", "ang_vel_z",
    "acc_g_x", "acc_g_y", "acc_g_z",
    "steer", "gas", "brake", "gear", "rpms",
    "wheel_slip_fl", "wheel_slip_fr", "wheel_slip_rl", "wheel_slip_rr",
]

POLL_MS = 1                  # beat AC's ~333 Hz physics (3 ms) and keep only new packetIds: the rate is AC's, not ours
OUT_DIR = "data/raw"
FLUSH_EVERY_S = 1.0
STATUS_EVERY_S = 0.25
RATE_WINDOW_S = 1.0          # the status line's rows/s is over this window


def _f32(x) -> float:
    # AC's floats are float32, good for about 7 significant digits. Widened
    # to a Python float they print 17, which is noise and doubles the file.
    # The rounded value is within a few float32 steps of AC's, not bit-exact.
    return float(f"{x:.7g}")


def physics_row(phys, t) -> list:
    """One CSV row in COLUMNS order; `t` is seconds since the logger started."""
    v, w, g, s = phys.velocity, phys.localAngularVel, phys.accG, phys.wheelSlip
    return [
        round(t, 6), phys.packetId,
        _f32(phys.speedKmh), _f32(v[0]), _f32(v[1]), _f32(v[2]), _f32(phys.heading),
        _f32(w[0]), _f32(w[1]), _f32(w[2]),
        _f32(g[0]), _f32(g[1]), _f32(g[2]),
        _f32(phys.steerAngle), _f32(phys.gas), _f32(phys.brake), phys.gear, phys.rpms,
        _f32(s[0]), _f32(s[1]), _f32(s[2]), _f32(s[3]),
    ]


def should_write(packet_id, last_packet_id, status) -> bool:
    """True only for a packet not yet written, while AC is live."""
    # Polling faster than AC writes sees each packet several times. `!=`
    # rather than `>`, so a packetId that restarts still gets logged.
    return status == AC_LIVE and packet_id != last_packet_id


def session_paths(out_dir, started) -> tuple:
    """CSV and JSON paths for a session started at the datetime `started`."""
    stem = started.strftime("%Y-%m-%d_%H%M%S")
    out = Path(out_dir)
    return out / f"{stem}.csv", out / f"{stem}.json"


def session_summary(ts, packet_ids, breaks=()) -> dict:
    """Rate and gap statistics of the rows written, for the session JSON.

    `breaks` holds the indices of rows written after AC left live (pause,
    replay). The interval before each is left out of every number: a pause
    isn't a logging gap, and would hide the real ones.
    """
    breaks = set(breaks)
    steps = {"1": 0, "2": 0, "3+": 0, "reset": 0}
    live_s = longest_gap = 0.0
    intervals = 0
    for i in range(1, len(ts)):
        if i in breaks:
            continue
        gap = ts[i] - ts[i - 1]
        live_s += gap
        longest_gap = max(longest_gap, gap)
        intervals += 1
        step = packet_ids[i] - packet_ids[i - 1]
        # A step of 2+ means packets AC wrote between two of our polls;
        # "reset" is packetId going backwards.
        steps["reset" if step < 1 else "3+" if step >= 3 else str(step)] += 1
    return {
        "rows": len(ts),
        "duration_s": round(live_s, 3),
        "mean_rows_per_s": round(intervals / live_s, 1) if live_s > 0 else 0.0,
        "packet_id_steps": steps,
        "longest_gap_s": round(longest_gap, 3),
        "breaks": len(breaks),
    }


def status_name(status) -> str:
    """ACStatus name for printing; the raw int if a live read is out of range."""
    try:
        return ACStatus(status).name
    except ValueError:
        return str(status)


def status_line(t, packet_id, rows, rate, status, speed_kmh, slip_deg) -> str:
    """The live console line, under 80 characters so a narrow window won't wrap it."""
    # A wrapped line prints a new line every update instead of rewriting
    # one, and this `t` is what maps video time to telemetry time.
    return (
        f"t {t:7.2f}  pkt {packet_id:>8}  rows {rows:>7}  {rate:>3}/s  "
        f"{status_name(status):<6}  {speed_kmh:5.1f} km/h  slip {slip_deg:+6.1f}"
    )


def _write_json(path, meta):
    path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")


def record(args, phys_mm, gfx_mm, static):
    """Poll until Ctrl+C, writing new live packets to the session's CSV."""
    t0 = time.perf_counter()
    started = datetime.now()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path, json_path = session_paths(out_dir, started)
    meta = {
        "car": wchars_to_str(static.carModel),
        "track": wchars_to_str(static.track),
        "ac_version": wchars_to_str(static.acVersion),
        "sm_version": wchars_to_str(static.smVersion),
        "date": started.strftime("%Y-%m-%d"),
        "start_time": started.strftime("%H:%M:%S"),
        "kind": args.kind,
        "setup": args.setup,
        "poll_ms": args.poll_ms,
        "python": platform.python_version(),
        "platform": platform.platform(),
    }

    # "x" so a clash can never overwrite a recorded session.
    csv_file = open(csv_path, "x", newline="", encoding="utf-8")
    _write_json(json_path, meta)
    writer = csv.writer(csv_file)
    writer.writerow(COLUMNS)

    print(f"Car: {meta['car']}   Track: {meta['track']}")
    print(f"Logging to {csv_path} -- Ctrl+C to stop.\n")

    ts, packet_ids, breaks = [], [], []
    last_id = None
    left_live = False
    next_flush = next_status = 0.0
    try:
        while True:
            status = read_page(gfx_mm, ACGraphicsPrefix).status
            phys = read_physics(phys_mm)
            t = time.perf_counter() - t0

            if should_write(phys.packetId, last_id, status):
                if left_live:
                    breaks.append(len(ts))
                    left_live = False
                writer.writerow(physics_row(phys, t))
                ts.append(t)
                packet_ids.append(phys.packetId)
                last_id = phys.packetId
            elif status != AC_LIVE and ts:
                left_live = True

            if t >= next_flush:
                csv_file.flush()
                next_flush = t + FLUSH_EVERY_S

            if t >= next_status:
                rate = len(ts) - bisect.bisect_right(ts, t - RATE_WINDOW_S)
                speed = phys.speedKmh
                slip = 0.0 if speed < SLIP_SPEED_FLOOR_KMH else slip_angle_deg(phys.velocity, phys.heading)
                print(
                    status_line(t, phys.packetId, len(ts), rate, status, speed, slip),
                    end="\r",
                    flush=True,
                )
                next_status = t + STATUS_EVERY_S

            time.sleep(args.poll_ms / 1000)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        csv_file.close()
        meta["summary"] = summary = session_summary(ts, packet_ids, breaks)
        _write_json(json_path, meta)

    print(
        f"Wrote {summary['rows']} rows over {summary['duration_s']:.1f} s live "
        f"({summary['mean_rows_per_s']} rows/s) to {csv_path}"
    )
    print(
        f"packetId steps: {summary['packet_id_steps']}   "
        f"longest gap: {summary['longest_gap_s']} s   breaks: {summary['breaks']}"
    )


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="python -m driftscore.logger",
        description="Record AC telemetry to CSV: one row per new physics packet, live only.",
    )
    parser.add_argument(
        "--kind", required=True, choices=["natural", "deliberate"],
        help="natural: holding drifts at the limit; deliberate: trying to spin",
    )
    parser.add_argument("--setup", default="", help='setup notes, e.g. "TC off, 2nd gear"')
    parser.add_argument("--out", default=OUT_DIR, help=f"output folder (default: {OUT_DIR})")
    parser.add_argument(
        "--poll-ms", type=float, default=POLL_MS,
        help=f"poll interval in milliseconds (default: {POLL_MS})",
    )
    args = parser.parse_args(argv)

    if sys.platform == "win32" and sys.version_info < (3, 11):
        print(
            "Warning: before Python 3.11, time.sleep on Windows rounds up to "
            "~15.6 ms ticks, which caps the logging rate. Use Python 3.11+.\n"
        )

    print("Connecting to Assetto Corsa shared memory...")
    with ExitStack() as pages:
        try:
            phys_mm = pages.enter_context(open_physics())
            gfx_mm = pages.enter_context(open_page(GRAPHICS_MEMORY_NAME, ACGraphicsPrefix))
            static_mm = pages.enter_context(open_page(STATIC_MEMORY_NAME, ACStaticPrefix))
        except Exception as exc:
            print(f"  Could not connect: {exc}")
            print("  Is AC running on THIS machine with a session loaded?")
            return
        record(args, phys_mm, gfx_mm, read_page(static_mm, ACStaticPrefix))


if __name__ == "__main__":
    main()
