# Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Measure order throughput of adapter_example (C++ and Python) and save it to CSV.

For every adapter implementation and robot count, this starts N simulated
robots (NUM_ROBOTS=N), then runs the C++ master_benchmark (which keeps every robot
busy and reports orders completed per second) and appends the result to a CSV.
Plot it with plot_order_benchmark.py. Requires a running MQTT broker.

    python run_order_benchmark.py --robots 10 50 100 200 --repeats 3
        [--adapters cpp py] [--duration 10]
        [--broker tcp://localhost:1883] [--output order_throughput.csv]
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXAMPLES = HERE.parent
WORKSPACE = HERE.parents[4]
DEFAULT_BIN_DIR = WORKSPACE / "install" / "vda5050_core" / "lib" / "vda5050_core"

PERFORMANCE_TAG = "=== Performance ==="
FIELDS = [
    "adapter", "robots", "repeat", "started", "run_s", "orders",
    "orders_per_s", "robot_mean", "robot_sd", "robot_min", "robot_max",
    "latency_ms", "rejected", "cpu_pct", "rss_mb",
]
# master_benchmark's "key=value" names -> CSV columns
KEYS = {
    "robots": "robots", "started": "started", "run": "run_s", "orders": "orders",
    "orders/s": "orders_per_s", "mean": "robot_mean", "sd": "robot_sd",
    "min": "robot_min", "max": "robot_max", "latency": "latency_ms",
    "rejected": "rejected", "cpu": "cpu_pct", "rss": "rss_mb",
}


def adapter_command(kind: str, bin_dir: Path) -> list[str]:
    if kind == "cpp":
        return [str(bin_dir / "adapter_example")]
    return [sys.executable, str(EXAMPLES / "python" / "adapter_example.py")]


def parse_performance(output: str) -> dict[str, str] | None:
    """Pull the key=value pairs out of master_benchmark's summary line."""
    for line in reversed(output.splitlines()):
        if PERFORMANCE_TAG not in line:
            continue
        row = {}
        for key, value in re.findall(r"([\w/]+)=([\d.]+)", line.split(PERFORMANCE_TAG)[1]):
            if key in KEYS:
                row.setdefault(KEYS[key], value)  # first "mean/sd/min/max" only
        return row
    return None


def stop(proc: subprocess.Popen) -> None:
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def run_once(args, adapter: str, n: int, repeat: int) -> dict[str, str] | None:
    env = {**os.environ, "NUM_ROBOTS": str(n), "MQTT_BROKER": args.broker}
    if args.drive_time is not None:
        env["DRIVE_TIME_S"] = str(args.drive_time)
    if args.action_time is not None:
        env["ACTION_TIME_S"] = str(args.action_time)
    adapter_proc = subprocess.Popen(
        adapter_command(adapter, args.bin_dir), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    try:
        time.sleep(args.settle)
        cmd = [str(args.bin_dir / "master_benchmark")] + [
            "-n", str(n), "--broker", args.broker, "--duration", str(args.duration),
            "--warmup-timeout", str(args.warmup_timeout), "--bench-period", "0",
            "--summary-only",
        ]
        result = subprocess.run(
            cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=args.warmup_timeout + args.duration + 60,
        )
    except subprocess.TimeoutExpired:
        print("  master_benchmark timed out", file=sys.stderr)
        return None
    finally:
        stop(adapter_proc)
        time.sleep(args.cooldown)

    row = parse_performance(result.stdout)
    if row is None:
        print(f"  no result line; master output:\n{result.stdout[-2000:]}", file=sys.stderr)
        return None
    return {"adapter": adapter, "repeat": str(repeat), **row}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--robots", type=int, nargs="+", default=[200])
    p.add_argument("--adapters", nargs="+", choices=["cpp", "py"], default=["cpp", "py"])
    p.add_argument("--repeats", type=int, default=3)
    p.add_argument("--duration", type=float, default=10, help="measured seconds per run")
    p.add_argument("--warmup-timeout", type=float, default=30)
    p.add_argument("--drive-time", type=float, help="adapter navigation time in s (default: adapter's own, 2)")
    p.add_argument("--action-time", type=float, help="adapter action time in s (default: adapter's own, 1)")
    p.add_argument("--settle", type=float, default=3, help="seconds to let adapters connect")
    p.add_argument("--cooldown", type=float, default=2, help="seconds between runs")
    p.add_argument("--broker", default=os.environ.get("MQTT_BROKER", "tcp://localhost:1883"))
    p.add_argument("--bin-dir", type=Path, default=DEFAULT_BIN_DIR,
                   help="directory holding the adapter_example/master_benchmark binaries")
    p.add_argument("--output", type=Path, default=Path("order_throughput.csv"))
    args = p.parse_args()

    new_file = not args.output.exists()
    with args.output.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        if new_file:
            writer.writeheader()
        for adapter in args.adapters:
            for n in args.robots:
                for repeat in range(args.repeats):
                    print(f"[{adapter}] robots={n} repeat={repeat + 1}/{args.repeats}", flush=True)
                    row = run_once(args, adapter, n, repeat)
                    if row is None:
                        continue
                    writer.writerow(row)
                    f.flush()
                    print(f"  {row['orders_per_s']} orders/s", flush=True)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
