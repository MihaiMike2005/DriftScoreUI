# DriftScoreUI

Drift-rating tool for Assetto Corsa.

- **Layer 1 (current):** the engine — reads AC's live physics telemetry and
  computes drift signals. Terminal-only.
- **Layer 2 (later):** a visual overlay built on top of the engine.

## Requirements

- **Running it for real:** Windows, with Assetto Corsa running and a session
  loaded (car on track). AC's shared memory is local and Windows-only.
- **Developing / running tests:** any OS.

## Setup

From the repo root:

```
pip install -e .
```

The `-e` (editable) install makes Python import the package straight from
`src/`, so code edits take effect immediately without reinstalling.

## Run

```
python -m driftscore.app
```

## Recording a session

The logger records telemetry for the spin early-warning model: one CSV row
per new physics packet, only while AC is live (never paused or in replay),
plus a JSON with car, track and setup.

```
python -m driftscore.logger --kind natural --setup "TC off, 2nd gear"
```

| Option | Meaning |
| --- | --- |
| `--kind` | Required. `natural` (holding drifts at the limit) or `deliberate` (trying to spin) |
| `--setup` | Free-text setup notes, saved in the JSON |
| `--out` | Output folder (default `data/raw`, relative to where you run the logger) |
| `--poll-ms` | Poll interval in milliseconds, 0.1 to 1000 (default 1) |

It writes `YYYY-MM-DD_HHMMSS.csv` and `.json` into the output folder. Run it
from the repo root and that's `data/raw/` here, which git ignores. When you
stop, the JSON gets a summary: rows, live duration, rows/s, packetId steps,
the longest gap and the breaks (pause or replay). Each break is listed with
its times, the `t` of the rows either side, and its kind: the AC statuses
seen in between, such as `PAUSE` or `REPLAY`. Breaks are left out of the
duration, rate and gap, so a pause can't hide a logging hiccup.

**Before**

1. Car on track: a session loaded, not sitting in a menu.
2. Start the screen recording, with the logger's console in the scene. Keep
   the console at least 80 characters wide, or its status line wraps and
   scrolls instead of updating in place.
3. Start the logger. The `t` on its status line is what maps video time to
   telemetry time.

**During:** drive 10–20 min.

**After**

1. Ctrl+C. The logger closes the CSV and writes the summary into the JSON.
2. Still on the PC, check the recording before copying it:

   ```
   python -m driftscore.logcheck data/raw/2026-10-11_143000.csv
   ```

   It prints the rate, every gap and break, which axis is yaw, lateral and
   longitudinal, the steering range and the wheel order, then `OK` or
   `CHECK` with the reasons. What to do about each reason:

   | `CHECK` reason | What to do |
   | --- | --- |
   | yaw, lateral or longitudinal r under the limit; steer outside ±1.05 | The struct is probably reading the wrong fields. Stop recording and paste the output to Claude Code. |
   | the rears' median slip isn't above both fronts | The same, unless the car isn't rear-wheel drive. |
   | steer short of ±0.5; under 1 s on the gas and sliding | Too little driving to judge, not a fault. Next time, drift both ways for a few minutes. |
   | logging gaps | The PC held the logger up for over 0.1 s. The session is still usable: later steps cut it at gaps. If it keeps happening, close other programs while recording. |
   | packetId steps of 2 or more | The poll is too slow to keep every packet. If logcheck shows a poll over 1 ms, run the logger without `--poll-ms`, so it polls every 1 ms. At 1 ms already, the PC is holding the logger up: close other programs, and paste the output to Claude Code if it keeps happening. While rows/s stays above 60, the session is still usable: later steps resample to 60 Hz. |

   Some lines are notes, not `CHECK` reasons, and need nothing done:

   - `note: ... it kept counting` or `note: ... AC stalled` under a gap:
     the JSON and the packetId test disagree about that gap.
   - The last row was cut off, or the JSON has no summary or is damaged:
     the logger was stopped without Ctrl+C, or while writing the JSON. The
     rest checks normally; stop it with Ctrl+C next time. A damaged JSON
     also loses the car, track, setup and kind, but the `notes.md` line
     you add next keeps them.

3. Add the session's line to your session log, `notes.md`: date, car, track,
   setup, minutes, spin count, natural or deliberate.
4. Copy the `.csv` and `.json` to the Mac.

## Layout

| Path | What it is |
| --- | --- |
| `src/driftscore/telemetry.py` | AC shared-memory reading — all Windows-specific code |
| `src/driftscore/signals.py` | Pure signal math (slip angle; scoring later) — testable anywhere |
| `src/driftscore/app.py` | The terminal loop tying the two together |
| `src/driftscore/logger.py` | Session logger: telemetry to CSV + JSON for the spin early-warning model |
| `src/driftscore/logcheck.py` | Log check: rate, gaps, axes, steering and wheel order of a recording, then OK or CHECK |
