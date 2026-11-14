# Laptop-only slow lawnmower mission

Firmware source in main/ has been restored byte-for-byte to this archive's original Git HEAD (b3bf08b). No firmware was built or flashed. Restoring source files does not change firmware already on your drone.

Use laptop/send_trajectory.py. It keeps onboard registration, accepts existing survey packets if available, and never starts a laptop camera detector. It no longer requires survey packets. Original firmware claims one non-navigation tag; slowing down does not remove that limit or produce missing tag coordinates. Existing survey firmware, if already flashed, can still supply its registrations to the unchanged CSV registry. CSV output is written only when survey entries arrive; legacy claims are logged in the terminal.

## Included route

trajectory/lawnmower_close_slow.csv uses your uploaded setup.yaml bounds x=0..3.8, y=-1..1, margin 0.1, 9 lanes spaced 0.225 m apart. Relative to starting hover, with local NED axes. It preserves the starting altitude. The route has 1364 points, within the existing 2400-point capacity.

Validate without connecting:

```bash
python3 laptop/send_trajectory.py --drone-id 9 --trajectory trajectory/lawnmower_close_slow.csv --dt 0.2 --speed-scale 0.6 --dry-run
```

Fly:

```bash
python3 laptop/send_trajectory.py --drone-id 9 --trajectory trajectory/lawnmower_close_slow.csv --dt 0.2 --speed-scale 0.6 --takeoff --confirm --finish land
```

This route peaks at approximately 0.15 m/s and takes 454 seconds (7 minutes 34 seconds), excluding takeoff, upload, and any return. Confirm that duration suits your flight battery. The slowdown changes the uploaded sample interval; it does not add points or change the route geometry. The firmware interpolates between samples. --max-speed remains a rejection limit, not a speed command.

Optional --return-start uses the original GOTO command to return to the sampled starting XY. That return uses the firmware's waypoint speed and cruise altitude, not --speed-scale. It needs a clear route. Arrival means within 0.25 m for one second. Ctrl+C requests landing in place.

To regenerate after editing arena bounds:

```bash
python3 laptop/generate_lawnmower_trajectory.py --config laptop/setup.yaml --lane-spacing 0.25 --margin 0.1 --speed 0.25 --dt 0.2 --output trajectory/lawnmower_close_slow.csv
```

The generator now defaults to 0.15 m/s when --speed is omitted; the command above explicitly uses 0.25 with the sender's 0.6 multiplier to give 0.15. Do not confuse CSV sampling --dt with playback speed. Use the same --dt in generation and sending.

Other historical scripts are retained but are not used by these commands. Camera calibration, detection thresholds, tag confirmation, and registration are unchanged. Python compilation and dry-run compatibility with the original protocol/comms passed. Hardware flight and tag-detection performance have not been tested.
