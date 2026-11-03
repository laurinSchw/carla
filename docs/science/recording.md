# Recording runs and using the data

A **run** is a directory written while the server is connected to CARLA. Start and stop it from the admin API
(`POST /admin/runs {"name": "...", "record_lidar": false}`, `POST /admin/runs/current/stop`, `GET /admin/runs`) or
programmatically (`PersistentCARLAServer.start_run`). In Docker the directory is `./runs/` on the host.

```
runs/<UTC>_<git sha>_<name>/
  manifest.json       config, seeds, versions, map, dt, driver profiles, DSRC parameters, start/end frame, row counts, column names
  events.jsonl        {frame, sim_time, kind, ...}: run_start/stop, spawn_background_vehicle (driver profile + TM settings),
                      client_join/leave, spawn_pedestrians, lidar (lifecycle), advisory (slow/restore), driver_setting_failed
  stations.jsonl      one line per V2X update (every 10th frame at 2 Hz): ground truth of every vehicle / RSU / VRU
  v2x_links.jsonl.gz  gzip, columnar: {frame, sim_time, rows: [[...], ...]}; column names in manifest.formats.v2x_links.columns
  lidar_meta.jsonl, lidar/<sensor>/<frame>.bin     only with record_lidar: protocol-v3 frames at FULL resolution
```
Every record has `frame` and `sim_time` from the CARLA snapshot; wall-clock time exists only in the manifest. Nothing is rounded
or decimated. Units are in the names (`x_m`, `speed_mps`, `yaw_deg`, `rx_power_dbm`). CARLA axes: x forward, y right, z up, yaw in
degrees from +x towards +y. `v2x_equipped` marks the vehicles that are V2X stations (`v2x_penetration_rate`).

## Reading it
```python
import gzip, json, pandas as pd
run = "runs/<run id>/"
man = json.load(open(run + "manifest.json"))
stations = pd.json_normalize([s | {"frame": r["frame"], "sim_time": r["sim_time"]}
                              for r in map(json.loads, open(run + "stations.jsonl")) for s in r["stations"]])
cols = man["formats"]["v2x_links"]["columns"]
links = pd.DataFrame([row for line in gzip.open(run + "v2x_links.jsonl.gz", "rt") for row in json.loads(line)["rows"]], columns=cols)
links.groupby("msg_type").delivered.mean()      # reception per message type; drop_cause says why a packet was lost
```
`delivered` means "reached the receiver's application": a packet the radio delivered but that is beyond the awareness range has
`delivered = False, drop_cause = "out_of_awareness_range"` (radio-level delivery is `drop_cause != "channel_or_mac"`).

## What is and is not reproducible (measured on the lab PC, CARLA 0.9.16, Town10HD)
- Same seed => the same driver profiles, spawn points and V2X-channel random draws (byte-identical files when the inputs are
  identical: `tests/unit/test_recorder.py`).
- Live, two runs with seed 42 on a freshly connected server: spawn positions identical, **vehicle trajectories agree to a median of
  ~0.4 cm over 20 s** (max 3 cm for the first 5 s, 24 cm at 10 s, 3.3 m for one vehicle at 20 s), whereas seed 43 differs by ~90 m median.
  So runs are *statistically and nearly trajectory-wise* reproducible, **not bit-identical**.
- Causes (not separated yet): commands arrive at slightly different frames because the loop is paced to real time; traffic-light phase
  depends on absolute world time; CARLA/TM float behaviour. A world reload before every run and frame-scheduled commands (handoff Phase 7)
  would be needed for bit-identical repeats. Treat single-vehicle outcomes as seed-dependent and aggregate over several seeds.
- Seeds set: master (spawns), Traffic Manager, pedestrians (`world.set_pedestrians_seed`), driver profiles (seed + 2), V2X penetration (seed + 1),
  DSRC channel (42, `DSRCConfig.random_seed`). All are in the manifest. Set the master seed with `/admin/connect {"seed": ...}` or `--seed`.

## Size
`v2x_links.jsonl.gz` costs about 54 bytes per transmission attempt: 25 vehicles at 2 Hz is about 2 MB/minute (the first uncompressed
version was 430 bytes per attempt, 28 MB for 30 s). Switch it off with `record_links: false` or `DSRCConfig.record_links = False`.
LiDAR frames are about 480 kB each at the default settings: `record_lidar` is only for short runs.
