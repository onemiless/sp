#!/usr/bin/env python3
"""Low-overhead, onroad resource profiler for manager-owned processes.

CPU and memory come from the existing procLog service. GPU, storage capacity,
power, and thermals come from deviceState. /proc/<pid>/io is read at a much
lower cadence to add per-service storage I/O without duplicating proclogd's
process scan.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import UTC, datetime
import json
import os
from pathlib import Path
import random
import time
import uuid
from typing import Any, NoReturn

from openpilot.cereal import messaging
from openpilot.common.hardware import HARDWARE, PC
from openpilot.common.swaglog import cloudlog
from openpilot.common.utils import atomic_write
from openpilot.common.version import get_build_metadata


SCHEMA_VERSION = 1
FLUSH_INTERVAL_S = 60.0
IO_INTERVAL_S = 10.0
RESERVOIR_SIZE = 256
MAX_SESSIONS = 10


def _percentile(values: list[float], percentile: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  return ordered[round(percentile * (len(ordered) - 1))]


@dataclass
class Metric:
  count: int = 0
  total: float = 0.0
  minimum: float | None = None
  maximum: float | None = None
  reservoir: list[float] = field(default_factory=list)

  def add(self, value: float) -> None:
    if not isinstance(value, (int, float)):
      return
    value = float(value)
    self.count += 1
    self.total += value
    self.minimum = value if self.minimum is None else min(self.minimum, value)
    self.maximum = value if self.maximum is None else max(self.maximum, value)
    if len(self.reservoir) < RESERVOIR_SIZE:
      self.reservoir.append(value)
    else:
      index = random.randrange(self.count)
      if index < RESERVOIR_SIZE:
        self.reservoir[index] = value

  def as_dict(self) -> dict[str, float | int | None]:
    return {
      "count": self.count,
      "mean": self.total / self.count if self.count else None,
      "min": self.minimum,
      "p50_approx": _percentile(self.reservoir, 0.50),
      "p95_approx": _percentile(self.reservoir, 0.95),
      "max": self.maximum,
    }


@dataclass
class ServiceMetrics:
  cpu_percent_one_core: Metric = field(default_factory=Metric)
  rss_mib: Metric = field(default_factory=Metric)
  pss_mib: Metric = field(default_factory=Metric)
  disk_read_mib_s: Metric = field(default_factory=Metric)
  disk_write_mib_s: Metric = field(default_factory=Metric)
  threads: Metric = field(default_factory=Metric)

  def as_dict(self) -> dict[str, dict[str, float | int | None]]:
    return {name: metric.as_dict() for name, metric in vars(self).items()}


def _read_proc_io(pid: int) -> tuple[int, int] | None:
  try:
    values: dict[str, int] = {}
    with open(f"/proc/{pid}/io") as f:
      for line in f:
        key, value = line.split(":", 1)
        if key in ("read_bytes", "write_bytes"):
          values[key] = int(value.strip())
    if "read_bytes" in values and "write_bytes" in values:
      return values["read_bytes"], values["write_bytes"]
  except (FileNotFoundError, PermissionError, ProcessLookupError, OSError, ValueError):
    pass
  return None


class SystemStatsCollector:
  def __init__(self, io_reader: Callable[[int], tuple[int, int] | None] = _read_proc_io):
    self.services: dict[str, ServiceMetrics] = {}
    self.system = {
      "cpu_mean_percent": Metric(),
      "cpu_max_core_percent": Metric(),
      "memory_used_percent": Metric(),
      "gpu_total_percent": Metric(),
      "disk_used_percent": Metric(),
      "power_draw_w": Metric(),
      "max_temp_c": Metric(),
    }
    self.previous_cpu: dict[tuple[int, float], tuple[float, float]] = {}
    self.previous_io: dict[tuple[int, float], tuple[int, int, float]] = {}
    self.io_reader = io_reader
    self.next_io_sample = 0.0
    self.proc_samples = 0
    self.device_samples = 0

  def ingest_proc_log(self, proc_log: Any, manager_state: Any, monotonic_s: float) -> None:
    managed = {int(p.pid): str(p.name) for p in manager_state.processes if p.running and int(p.pid) > 0}
    proc_by_pid = {int(p.pid): p for p in proc_log.procs}
    sample_io = monotonic_s >= self.next_io_sample
    if sample_io:
      self.next_io_sample = monotonic_s + IO_INTERVAL_S

    active_cpu: set[tuple[int, float]] = set()
    active_io: set[tuple[int, float]] = set()
    for pid, service_name in managed.items():
      proc = proc_by_pid.get(pid)
      if proc is None:
        continue
      start_time = float(proc.startTime)
      identity = (pid, start_time)
      metrics = self.services.setdefault(service_name, ServiceMetrics())
      cpu_total = float(proc.cpuUser) + float(proc.cpuSystem)
      previous_cpu = self.previous_cpu.get(identity)
      if previous_cpu is not None:
        elapsed = monotonic_s - previous_cpu[1]
        delta_cpu = cpu_total - previous_cpu[0]
        if elapsed > 0.0 and delta_cpu >= 0.0:
          metrics.cpu_percent_one_core.add(100.0 * delta_cpu / elapsed)
      self.previous_cpu[identity] = (cpu_total, monotonic_s)
      active_cpu.add(identity)

      metrics.rss_mib.add(int(proc.memRss) / (1024 * 1024))
      if int(proc.memPss) > 0:
        metrics.pss_mib.add(int(proc.memPss) / (1024 * 1024))
      metrics.threads.add(int(proc.numThreads))

      if sample_io:
        io = self.io_reader(pid)
        previous_io = self.previous_io.get(identity)
        if io is not None and previous_io is not None:
          elapsed = monotonic_s - previous_io[2]
          if elapsed > 0.0:
            read_delta = io[0] - previous_io[0]
            write_delta = io[1] - previous_io[1]
            if read_delta >= 0:
              metrics.disk_read_mib_s.add(read_delta / elapsed / (1024 * 1024))
            if write_delta >= 0:
              metrics.disk_write_mib_s.add(write_delta / elapsed / (1024 * 1024))
        if io is not None:
          self.previous_io[identity] = (io[0], io[1], monotonic_s)
          active_io.add(identity)

    self.previous_cpu = {key: value for key, value in self.previous_cpu.items() if key in active_cpu}
    if sample_io:
      self.previous_io = {key: value for key, value in self.previous_io.items() if key in active_io}
    self.proc_samples += 1

  def ingest_device_state(self, device_state: Any) -> None:
    cpu = [float(value) for value in device_state.cpuUsagePercent if value >= 0]
    if cpu:
      self.system["cpu_mean_percent"].add(sum(cpu) / len(cpu))
      self.system["cpu_max_core_percent"].add(max(cpu))
    memory = float(device_state.memoryUsagePercent)
    gpu = float(device_state.gpuUsagePercent)
    free_space = float(device_state.freeSpacePercent)
    if memory >= 0:
      self.system["memory_used_percent"].add(memory)
    if gpu >= 0:
      self.system["gpu_total_percent"].add(gpu)
    if 0.0 <= free_space <= 100.0:
      self.system["disk_used_percent"].add(100.0 - free_space)
    self.system["power_draw_w"].add(float(device_state.powerDrawW))
    self.system["max_temp_c"].add(float(device_state.maxTempC))
    self.device_samples += 1

  def recommendations(self) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for name, service in sorted(self.services.items()):
      cpu = service.cpu_percent_one_core.as_dict()
      pss = service.pss_mib.as_dict()
      rss = service.rss_mib.as_dict()
      write = service.disk_write_mib_s.as_dict()
      memory_p95 = pss["p95_approx"] if pss["count"] else rss["p95_approx"]
      if cpu["p95_approx"] is not None and cpu["p95_approx"] >= 50.0:
        result.append(
          {
            "service": name,
            "kind": "high_cpu",
            "evidence": {"cpu_p95_one_core_percent": cpu["p95_approx"]},
            "next_step": "profile hot paths and verify service rate/necessity",
          }
        )
      if memory_p95 is not None and memory_p95 >= 300.0:
        result.append(
          {
            "service": name,
            "kind": "high_memory",
            "evidence": {"memory_p95_mib": memory_p95},
            "next_step": "inspect allocations, model buffers, and cache lifetime",
          }
        )
      if write["p95_approx"] is not None and write["p95_approx"] >= 1.0:
        result.append(
          {
            "service": name,
            "kind": "high_disk_write",
            "evidence": {"write_p95_mib_s": write["p95_approx"]},
            "next_step": "inspect logging frequency, retention, and compression",
          }
        )
      if cpu["count"] >= 30 and cpu["mean"] is not None and cpu["mean"] < 0.2 and rss["mean"] is not None and rss["mean"] < 20.0:
        result.append(
          {
            "service": name,
            "kind": "low_activity_review_only",
            "evidence": {"cpu_mean_one_core_percent": cpu["mean"], "rss_mean_mib": rss["mean"]},
            "next_step": "check dependencies and safety role; low activity does not prove the service is unnecessary",
          }
        )
    return result

  def as_dict(self) -> dict[str, Any]:
    return {
      "sample_counts": {"proc_log": self.proc_samples, "device_state": self.device_samples},
      "system": {name: metric.as_dict() for name, metric in self.system.items()},
      "services": {name: metrics.as_dict() for name, metrics in sorted(self.services.items())},
      "recommendations": self.recommendations(),
    }


class SessionWriter:
  def __init__(self, root: Path):
    self.root = root
    self.root.mkdir(parents=True, exist_ok=True)
    self.session_id = uuid.uuid4().hex[:12]
    self.started_at = datetime.now(UTC).isoformat()
    metadata = get_build_metadata()
    self.metadata = {
      "branch": metadata.channel,
      "commit": metadata.openpilot.git_commit,
      "version": metadata.openpilot.version,
      "device_type": HARDWARE.get_device_type(),
    }

  def payload(self, collector: SystemStatsCollector, final: bool) -> dict[str, Any]:
    return {
      "schema_version": SCHEMA_VERSION,
      "session_id": self.session_id,
      "started_at": self.started_at,
      "updated_at": datetime.now(UTC).isoformat(),
      "final": final,
      "metadata": self.metadata,
      "definitions": {
        "cpu_percent_one_core": "100% means one logical CPU core fully occupied",
        "gpu_total_percent": "whole-device GPU load; per-service GPU attribution is unavailable",
        "disk_io": "physical storage bytes from /proc/<pid>/io, sampled every 10 seconds",
        "p95_approx": f"reservoir estimate bounded to {RESERVOIR_SIZE} values",
      },
      "safety_note": "Recommendations are evidence for review only. No service is disabled automatically.",
      **collector.as_dict(),
    }

  def write(self, collector: SystemStatsCollector, final: bool = False) -> Path:
    payload = self.payload(collector, final)
    latest = self.root / "latest.json"
    with atomic_write(str(latest), overwrite=True) as f:
      json.dump(payload, f, separators=(",", ":"), sort_keys=True)
    if final:
      archive = self.root / f"session-{self.session_id}.json"
      with atomic_write(str(archive), overwrite=True) as f:
        json.dump(payload, f, separators=(",", ":"), sort_keys=True)
      archives = sorted(self.root.glob("session-*.json"), key=lambda path: path.stat().st_mtime)
      for stale in archives[:-MAX_SESSIONS]:
        stale.unlink(missing_ok=True)
      return archive
    return latest


def stats_root() -> Path:
  override = os.getenv("SYSTEM_STATS_ROOT")
  if override:
    return Path(override)
  return Path.home() / ".comma" / "system_stats" if PC else Path("/data/system_stats")


def run_collector() -> None:
  collector = SystemStatsCollector()
  writer = SessionWriter(stats_root())
  sm = messaging.SubMaster(["procLog", "managerState", "deviceState"])
  next_flush = time.monotonic() + FLUSH_INTERVAL_S
  try:
    while True:
      sm.update(1000)
      now = time.monotonic()
      if sm.updated["procLog"] and sm.valid["procLog"] and sm.valid["managerState"]:
        collector.ingest_proc_log(sm["procLog"], sm["managerState"], sm.logMonoTime["procLog"] / 1e9)
      if sm.updated["deviceState"] and sm.valid["deviceState"]:
        collector.ingest_device_state(sm["deviceState"])
      if now >= next_flush:
        writer.write(collector)
        next_flush = now + FLUSH_INTERVAL_S
  except KeyboardInterrupt:
    pass
  finally:
    try:
      writer.write(collector, final=True)
    except Exception:
      cloudlog.exception("failed to write final system stats session")


def print_report(path: Path) -> None:
  with open(path) as f:
    report = json.load(f)
  print(f"session={report['session_id']} samples={report['sample_counts']} final={report['final']}")
  print(f"{'service':28} {'cpu mean/p95 %':>18} {'pss/rss mean MiB':>19} {'write p95 MiB/s':>17}")
  for name, metrics in report["services"].items():
    cpu = metrics["cpu_percent_one_core"]
    memory = metrics["pss_mib"] if metrics["pss_mib"]["count"] else metrics["rss_mib"]
    write = metrics["disk_write_mib_s"]
    print(f"{name[:28]:28} {cpu['mean'] or 0:7.2f}/{cpu['p95_approx'] or 0:7.2f} {memory['mean'] or 0:19.1f} {write['p95_approx'] or 0:17.3f}")
  print("\nreview candidates:")
  for item in report["recommendations"]:
    print(f"- {item['service']}: {item['kind']} {item['evidence']}")


def main() -> NoReturn:
  run_collector()
  raise SystemExit(0)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--report", type=Path, nargs="?", const=stats_root() / "latest.json", help="print a saved report (default: latest)")
  args = parser.parse_args()
  if args.report:
    print_report(args.report)
  else:
    run_collector()
