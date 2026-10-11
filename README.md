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
the longest gap and the number of breaks (pause or replay). Breaks are left
out of the duration, rate and gap, so a pause can't hide a logging hiccup.

Use Python 3.11+ on the AC PC: on Windows, older versions round `time.sleep`
up to ~15.6 ms, which caps the logging rate.

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
2. Add the session's line to your session log, `notes.md`: date, car, track,
   setup, minutes, spin count, natural or deliberate.
3. Copy the `.csv` and `.json` to the Mac.

## Layout

| Path | What it is |
| --- | --- |
| `src/driftscore/telemetry.py` | AC shared-memory reading — all Windows-specific code |
| `src/driftscore/signals.py` | Pure signal math (slip angle; scoring later) — testable anywhere |
| `src/driftscore/app.py` | The terminal loop tying the two together |
| `src/driftscore/logger.py` | Session logger: telemetry to CSV + JSON for the spin early-warning model |
