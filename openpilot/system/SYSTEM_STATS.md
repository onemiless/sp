# SP system resource profiler

`system_statsd` is an opt-in, onroad-only profiler for processes owned by SP's manager.

## Data and overhead

- CPU, RSS/PSS memory: reuses `procLog` at 0.5 Hz; it does not perform a second process scan.
- Whole-device CPU, memory, GPU, storage capacity, power, and thermals: reuses `deviceState`.
- Per-service storage reads/writes: reads `/proc/<pid>/io` for running manager processes every 10 seconds.
- Results are aggregated in memory and atomically written once per minute. At most 10 completed sessions are retained in `/data/system_stats`.
- Percentiles use a bounded 256-value reservoir, so memory use does not grow with drive duration.

GPU attribution is whole-device only because the Qualcomm runtime does not expose reliable per-process GPU counters through the current SP messages.

## Enable and inspect

The persistent parameter defaults to off:

```sh
cd /data/sp
./scripts/set_params.py SystemStatsCollectionEnabled 1
```

It starts with the onroad process set and stops offroad. It never disables or changes another service.

```sh
cd /data/sp
PYTHONPATH=/data/sp /usr/local/venv/bin/python -m openpilot.system.system_statsd --report
```

The latest machine-readable report is `/data/system_stats/latest.json`. Heuristic recommendations identify high CPU, memory, or disk writers and low-activity review candidates. Low activity alone is explicitly not evidence that a service is unnecessary; dependencies and safety roles must be reviewed before any service change.
