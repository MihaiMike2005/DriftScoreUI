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

## Layout

| Path | What it is |
| --- | --- |
| `src/driftscore/telemetry.py` | AC shared-memory reading — all Windows-specific code |
| `src/driftscore/signals.py` | Pure signal math (slip angle; scoring later) — testable anywhere |
| `src/driftscore/app.py` | The terminal loop tying the two together |
