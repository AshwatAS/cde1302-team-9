# CDE1302 Team 9 — Autonomous AprilTag Search Drone

Single-drone autonomous AprilTag exploration for the NUS CDE1302 Mini Project.
A PX4 quadrotor with an ESP32-S3 companion computer flies a pre-planned
edge-biased lawnmower sweep over a 3.8 m × 3.2 m arena, detects tag16h5
AprilTags (12 cm) at unknown positions, estimates each tag's position in the
local NED frame, then returns to its launch point and lands — all from one
ground-station command.

**Final flight output:** [`registered_tags.csv`](registered_tags.csv)

## What's in this repo

Built on top of the inherited `esp-everything-qgc-bridge` codebase (ESP32-S3
companion firmware originally written for the SAFMC 2026 Cat Swarm Challenge).
The firmware under `main/` is unchanged byte-for-byte — all CDE1302-specific
work is on the laptop side.

| File | Purpose |
|---|---|
| `laptop/generate_lawnmower_variable.py` | Edge-biased lawnmower CSV generator with tag-aware speed profile |
| `laptop/send_trajectory_no_flash.py` | Mission sender — uploads trajectory, runs ground-side tag registration, returns home, lands |
| `laptop/host_tag_registry.py` | Multi-tag registry reading the UDP 5008 detection stream (we added `--frames` and `--reset-gap` tunables) |
| `laptop/tag_registry_monitor.py` | Standalone bench monitor — proves registration gates work with no flight |
| `trajectory/lawnmower_tags.csv` | The exact trajectory that flew in the final demo (1,017 points, 203 s) |
| `registered_tags.csv` | Final mission output — detected tags with estimated local-NED coordinates |

## Hardware

- **Drone:** PX4 quadrotor with ESP32-S3 (Seeed Xiao) as companion computer
- **Camera:** OV3660, QVGA 320×240 grayscale, mounted at 45° downward tilt,
  2 cm forward of body origin
- **Camera intrinsics:** fx = 163.5, fy = 153.2, cx = 154.0, cy = 107.1
- **ToF:** VL53L5CX 8×8 time-of-flight sensors (for VFH collision avoidance)
- **Network:** WiFi router at 192.168.1.1/24; drone = 192.168.1.(200+ID),
  laptop = 192.168.1.(100+ID)
- **Tags:** AprilTag `tag16h5` family, 12 cm, IDs 0–11 as targets
  (12–21 reserved as nav/landmark tags in `setup.yaml`)

## Quick start

### 0. Install Python dependencies
```bash
python3 -m pip install -r laptop/requirements.txt
```

The ESP32 firmware is already flashed. No rebuild/reflash is needed.

### 1. Generate a trajectory for your arena + approximate tag positions
Tag coordinates are in local NED, measured from the hover/launch point
(= where the drone sits when its camera is directly over the tag):

```bash
cd laptop
python generate_lawnmower_variable.py --config setup.yaml \
    --tags "1.0,-1.4; 3.6,-1.3; 2.0,1.4; 3.8,1.2" \
    --fast-speed 0.4 --slow-speed 0.1 --slow-radius 0.7 --dwell 2 \
    --lanes 6 --edge-bias 0.7 --margin 0.4 --dt 0.2 \
    --output ../trajectory/lawnmower_tags.csv
```

The generator:
- Places lanes using a cosine (Chebyshev) distribution so lanes cluster at the
  arena borders (where tags usually sit) and widen mid-arena. `--edge-bias 0.7`
  is 70 % cosine + 30 % uniform.
- Splits each leg where it crosses a `--slow-radius` zone around a tag. In-zone
  sub-segments peak at `--slow-speed`; open legs run up to `--fast-speed`.
- Every sub-segment is a quintic rest-to-rest polynomial, so the velocity
  vector never flips at a 90° corner.

The printed report shows peak speed in each tag zone and warns if a tag
isn't within the sweep radius.

### 2. Bench-test detection (no flight)
Power the drone, point it at tags one at a time:

```bash
python tag_registry_monitor.py --drone-id 9 --frames 2 --reset-gap 3 --id-margin 50
```

Each tag should print `*** REGISTERED tag N ***` within ~2 s of steady view.
Holding tags still here mimics the drone dwelling over them in flight. Phantom
hamming-2 decodes are reported and rejected — this is tag16h5's known
false-positive behaviour.

### 3. Fly the mission
```bash
python send_trajectory_no_flash.py --drone-id 9 --takeoff --confirm \
    --dt 0.2 --trajectory ../trajectory/lawnmower_tags.csv --return-start \
    --frames 2 --reset-gap 3 --id-margin 50 \
    --return-offset-x 0.30 --return-offset-y 0.60
```

- `--dt` **must match** the generator's `--dt`.
- `--return-offset-*` compensates measured position-estimate drift over the
  flight. Re-measure for each new trajectory; it varies with flight duration.
- Home XY is captured **before** takeoff, so takeoff drift is excluded.
- `Ctrl+C` lands in place.

Output: `registered_tags.csv`, one row per detected tag:

```
drone_id,tag_id,localized,observations,x,y,z
9,20,True,17,0.079,0.284,0.174
```

Coordinates are in the drone's local NED frame (z positive down).

## Configuration

Arena bounds, drone IDs, start positions and nav tags live in
`laptop/setup.yaml`. The deployed arena:

```yaml
arena:
  min_x: 0.5
  max_x: 4.3
  min_y: -1.6
  max_y: 1.6
```

## System architecture at a glance

- **PX4 autopilot:** flight-control loops, local-NED state estimate, OFFBOARD setpoint execution.
- **ESP32-S3 companion:** six FreeRTOS tasks — MAVLink UART bridge, VFH
  navigator with trajectory playback, AprilTag detection, ToF ranging, WiFi
  telemetry/command I/O, mission state machine.
- **Laptop ground station:** trajectory generator, mission sender,
  `HostTagRegistry` reading UDP 5008.

Data flow in one mission:
- Laptop → drone (UDP 5006): `CMD_TRAJ_DATA` chunks + `CMD_TRAJ_START`, then
  `CMD_GOTO home` for return, `CMD_LAND` to land.
- Drone → laptop (UDP 5005): 10 Hz telemetry (pose, nav state).
- Drone → laptop (UDP 5008): 10 Hz raw AprilTag detection stream, one packet
  per camera frame, all detections included.

**Key design decision:** The inherited firmware "claims" the first AprilTag it
sees into a single telemetry field, so only one tag per drone ever reaches the
laptop via normal telemetry. The UDP 5008 debug stream, however, carries
**every** raw detection. Running the tag registry on the laptop against 5008
gave us multi-tag registration with zero firmware changes.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Only one tag registered during flight | Normal telemetry latch is firmware behaviour. Use `send_trajectory_no_flash.py` (reads UDP 5008), not `send_trajectory.py`. |
| `UDP 5008 unavailable` on startup | Close `tag_debug.py`, `tag_stream.py` or any other listener bound to 5008. |
| Tag seen on camera but never registered | Likely tag16h5 phantom flicker. Lower `--id-margin` or raise `--reset-gap`. Verify with `tag_registry_monitor.py`. |
| Drone lands metres off launch point | Position-estimate drift over flight time. Measure the offset, pass `--return-offset-x/-y`. Shorter flights drift less. |
| Return hangs, drone circles home | Firmware arrive-radius is 0.25 m. If you need tighter, flights get longer and drift wider — trade-off. |

## Credits

Firmware base: `esp-everything-qgc-bridge` — ESP32-S3 companion originally
written for the SAFMC 2026 Cat Swarm Challenge. See the original project's
[`README.md`](README.md) for firmware build/flash instructions (not needed for
normal operation; the drone is pre-flashed).

CDE1302 Team 9 — National University of Singapore, AY2026.
