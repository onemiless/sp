import json
from types import SimpleNamespace

from openpilot.system.manager.process_config import system_stats_collection
from openpilot.system.system_statsd import MAX_SESSIONS, Metric, SessionWriter, SystemStatsCollector


def proc(pid=10, start=1.0, cpu=0.0, rss=100 * 1024 * 1024, pss=80 * 1024 * 1024):
  return SimpleNamespace(pid=pid, startTime=start, cpuUser=cpu, cpuSystem=0.0, memRss=rss, memPss=pss, numThreads=3)


def manager(pid=10, name="modeld", running=True):
  return SimpleNamespace(processes=[SimpleNamespace(pid=pid, name=name, running=running)])


def proc_log(*procs):
  return SimpleNamespace(procs=procs)


def test_metric_summary():
  metric = Metric()
  for value in [1.0, 2.0, 3.0, 4.0]:
    metric.add(value)
  summary = metric.as_dict()
  assert summary["count"] == 4
  assert summary["mean"] == 2.5
  assert summary["min"] == 1.0
  assert summary["max"] == 4.0


def test_collects_only_manager_owned_processes_and_cpu_delta():
  collector = SystemStatsCollector(io_reader=lambda pid: None)
  collector.ingest_proc_log(proc_log(proc(cpu=1.0), proc(pid=11, cpu=50.0)), manager(), 100.0)
  collector.ingest_proc_log(proc_log(proc(cpu=2.0), proc(pid=11, cpu=60.0)), manager(), 102.0)
  assert set(collector.services) == {"modeld"}
  metrics = collector.services["modeld"]
  assert metrics.cpu_percent_one_core.as_dict()["mean"] == 50.0
  assert metrics.pss_mib.as_dict()["mean"] == 80.0


def test_pid_reuse_does_not_create_cpu_spike():
  collector = SystemStatsCollector(io_reader=lambda pid: None)
  collector.ingest_proc_log(proc_log(proc(start=1.0, cpu=100.0)), manager(), 100.0)
  collector.ingest_proc_log(proc_log(proc(start=2.0, cpu=1.0)), manager(), 102.0)
  assert collector.services["modeld"].cpu_percent_one_core.count == 0


def test_disk_io_is_low_rate_and_uses_deltas():
  reads = iter([(10 * 1024 * 1024, 20 * 1024 * 1024), (30 * 1024 * 1024, 50 * 1024 * 1024)])
  collector = SystemStatsCollector(io_reader=lambda pid: next(reads))
  collector.ingest_proc_log(proc_log(proc()), manager(), 100.0)
  collector.ingest_proc_log(proc_log(proc()), manager(), 105.0)
  assert collector.services["modeld"].disk_write_mib_s.count == 0
  collector.ingest_proc_log(proc_log(proc()), manager(), 110.0)
  metrics = collector.services["modeld"]
  assert metrics.disk_read_mib_s.as_dict()["mean"] == 2.0
  assert metrics.disk_write_mib_s.as_dict()["mean"] == 3.0


def test_device_metrics_are_aggregate():
  collector = SystemStatsCollector()
  state = SimpleNamespace(cpuUsagePercent=[10, 30], memoryUsagePercent=40, gpuUsagePercent=50, freeSpacePercent=75, powerDrawW=8.5, maxTempC=61.0)
  collector.ingest_device_state(state)
  assert collector.system["cpu_mean_percent"].as_dict()["mean"] == 20.0
  assert collector.system["gpu_total_percent"].as_dict()["mean"] == 50.0
  assert collector.system["disk_used_percent"].as_dict()["mean"] == 25.0


def test_recommendations_never_claim_service_is_unnecessary():
  collector = SystemStatsCollector(io_reader=lambda pid: None)
  for index in range(31):
    collector.ingest_proc_log(proc_log(proc(cpu=index * 0.001, rss=10 * 1024 * 1024, pss=0)), manager(name="idle_service"), 100.0 + index)
  recommendations = collector.recommendations()
  assert recommendations[0]["kind"] == "low_activity_review_only"
  assert "does not prove" in recommendations[0]["next_step"]


def test_manager_switch_is_onroad_and_opt_in():
  params = SimpleNamespace(get_bool=lambda key: key == "SystemStatsCollectionEnabled")
  assert system_stats_collection(True, params, SimpleNamespace())
  assert not system_stats_collection(False, params, SimpleNamespace())
  assert not system_stats_collection(True, SimpleNamespace(get_bool=lambda key: False), SimpleNamespace())


def test_session_files_are_bounded_and_describe_gpu_limit(tmp_path, monkeypatch):
  monkeypatch.setattr("openpilot.system.system_statsd.get_branch", lambda: "test")
  monkeypatch.setattr("openpilot.system.system_statsd.get_commit", lambda: "abc")
  monkeypatch.setattr("openpilot.system.system_statsd.get_version", lambda: "1")
  monkeypatch.setattr("openpilot.system.system_statsd.HARDWARE.get_device_type", lambda: "test-device")
  collector = SystemStatsCollector()
  for _ in range(MAX_SESSIONS + 2):
    writer = SessionWriter(tmp_path)
    writer.write(collector, final=True)
  assert len(list(tmp_path.glob("session-*.json"))) == MAX_SESSIONS
  report = json.loads((tmp_path / "latest.json").read_text())
  assert "per-service GPU attribution is unavailable" in report["definitions"]["gpu_total_percent"]
