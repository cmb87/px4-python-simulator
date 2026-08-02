# Multi-UAV bridge + web viewer

An additive layer on top of the PX4 Python lockstep SITL bridge: run **N drones in one
process on one shared clock**, and watch them live in a **self-contained web viewer**.

Everything here is new files, none of the existing single-vehicle code is modified.

## What is added

| File | What it does |
|---|---|
| `src/main_multi.py` | Multi-vehicle bridge. Bridges N PX4 SITL instances (`none_iris`, ports 4560+i) to N `World` models on one shared sim clock, shared GPS origin, per-drone NED spawn grid. Streams per-drone ground truth and a full-scene frame stream for the viewer. `SIM_SPEED` paces wall-clock (1x, 2x, or free-run). |
| `src/networking/viz_stream.py` | Broadcasts scene frames (drone poses) over one websocket, records them to JSON-lines for offline playback, and accepts client-injected annotations (e.g. a goal marker) merged into outgoing frames. |
| `support/tools/viz/drone_viewer.html` | Self-contained viewer (no build step, no external libs). 3D orbit view + side elevation, live or from a recording, timeline scrub, replay speed, per-drone trails, side-by-side video capture, and a "kill sim" button that frees the port. |
| `src/offboard_demo.py` | Minimal MAVLink offboard commander so the fleet actually flies: arms every PX4, takes off, hovers or flies a slow orbit, using PX4's own stock controllers (no custom control law). |
| `support/tools/run_multi.sh`, `support/tools/stop.sh` | Launch/teardown N PX4 + the bridge (+ `--fly` for the offboard demo). |

## Quick start

```bash
# build PX4 SITL once
cd ~/PX4-Autopilot && make px4_sitl_default

# bridge + 4 PX4, arm + take off + hover
cd <this-repo>
N=4 bash support/tools/run_multi.sh --fly

# watch: open support/tools/viz/drone_viewer.html in a browser, connect to ws://localhost:8766
#   (or support/tools/viz/drone_viewer.html?ws=ws://localhost:8766 to auto-connect)

bash support/tools/stop.sh   # tear down
```

Record a run for offline replay:

```bash
N=4 SIM_VIZ_RECORD=/tmp/run.jsonl bash support/tools/run_multi.sh --fly
# then in the viewer: "open rec…" -> /tmp/run.jsonl, and scrub
```

## Dependencies

`numpy`, `pymavlink`, `websockets` (already the bridge's deps). The viewer is a single
static HTML file, opened directly in a browser.

## Attribution & license

The base 6DOF bridge, sensor synthesis, and vehicle models are from this repository.
