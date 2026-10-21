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

"""Order-throughput benchmark master (Python).

Run adapter_example (C++ or Python, N robots) first, then this with the same
-n. It keeps every robot busy: it assigns a one-edge order and, the moment the
robot reports it complete, assigns the next. Reports orders completed per
second, in total and per robot. Usage:

    python master_benchmark.py -n 100 [--broker tcp://localhost:1883]
        [--duration 10] [--warmup-timeout 30] [--bench-period 5]
        [--summary-only]
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import resource
import signal
import threading
import time

from vda5050_core.master import VDA5050Master
from vda5050_core.transport import create_default_client_shared
from vda5050_core.types import (
    Action,
    ActionParameter,
    BlockingType,
    Edge,
    InstantActions,
    Node,
    Order,
)

INTERFACE_NAME = "uagv"
MAP_ID = "map_1"
LOGGER = logging.getLogger(__name__)

running = threading.Event()
running.set()


def signal_handler(signum, _frame) -> None:
    LOGGER.info("System Signal [%d] received. Shutting down ...", signum)
    running.clear()


def serial_for(index: int) -> str:
    return f"S{index + 1:03d}"


def index_of(agv_id: str) -> int:
    """The master reports AGVs as "<manufacturer>/<serial>"; "S007" -> 6."""
    return int(agv_id.split("/")[-1][1:]) - 1


def rss_mb() -> float:
    with open("/proc/self/statm") as f:
        return int(f.read().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 1e6


def cpu_seconds() -> float:
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return usage.ru_utime + usage.ru_stime


def make_node(node_id: str, seq: int) -> Node:
    n = Node()
    n.node_id = node_id
    n.sequence_id = seq
    n.released = True
    return n


def make_order(order_id: str, start: str, end: str) -> Order:
    """A minimal released route start -> end (header auto-filled by master)."""
    o = Order()
    o.order_id = order_id
    o.order_update_id = 0
    o.nodes = [make_node(start, 0), make_node(end, 2)]
    e = Edge()
    e.edge_id = "E0"
    e.sequence_id = 1
    e.start_node_id = start
    e.end_node_id = end
    e.released = True
    o.edges = [e]
    return o


def make_action(action_type: str, params: dict[str, str]) -> Action:
    a = Action()
    a.action_id = f"{action_type}-{time.monotonic_ns()}"
    a.action_type = action_type
    a.blocking_type = BlockingType.NONE
    parameters = []
    for k, v in params.items():
        p = ActionParameter()
        p.key = k
        p.value = v
        parameters.append(p)
    a.action_parameters = parameters
    return a


class Robot:
    """Per-robot benchmark state; each robot has its own lock."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.init_sent = False
        self.started = False
        self.orders_sent = 0
        self.sent_at = 0.0
        # Measured window only:
        self.completed = 0
        self.latency_sum = 0.0  # seconds, assign -> complete
        self.window_completed = 0  # for the progress lines


def stats_of(values: list[float]) -> tuple[float, float, float, float]:
    if not values:
        return 0.0, 0.0, 0.0, 0.0
    mean = sum(values) / len(values)
    sd = math.sqrt(sum((x - mean) ** 2 for x in values) / len(values))
    return mean, sd, min(values), max(values)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("-n", "--n-robots", type=int, default=10)
    parser.add_argument(
        "--broker", default=os.environ.get("MQTT_BROKER", "tcp://localhost:1883")
    )
    parser.add_argument(
        "--manufacturer", default=os.environ.get("VDA5050_MANUFACTURER", "Manufacturer")
    )
    parser.add_argument(
        "--duration", type=float, default=10, help="measured seconds; 0 = until Ctrl-C"
    )
    parser.add_argument(
        "--warmup-timeout",
        type=float,
        default=30,
        help="max wait for every robot to get its first order",
    )
    parser.add_argument(
        "--bench-period", type=float, default=5.0, help="seconds between progress lines; 0 = off"
    )
    parser.add_argument(
        "--summary-only", action="store_true", help="skip the per-robot table"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    n = args.n_robots
    mfr = args.manufacturer

    # --- Master that keeps every robot busy --------------------------------
    robots = [Robot() for _ in range(n)]
    state = {"measuring": False, "stopping": False, "started": 0, "rejected": 0}
    counter_lock = threading.Lock()

    master = VDA5050Master.make(
        create_default_client_shared(args.broker, "master_benchmark")
    )

    def next_order(idx: int, r: Robot) -> bool:
        """Alternate N0->N1 / N1->N0. Caller must hold r.lock."""
        serial = serial_for(idx)
        forward = r.orders_sent % 2 == 0
        r.sent_at = time.perf_counter()
        res = master.assign_order(
            mfr,
            serial,
            make_order(
                f"order-{serial}-{r.orders_sent}",
                "N0" if forward else "N1",
                "N1" if forward else "N0",
            ),
        )
        if res:
            r.orders_sent += 1
            return True
        with counter_lock:
            state["rejected"] += 1
        return False

    def send_actions(idx: int, action: Action) -> None:
        ia = InstantActions()
        ia.actions = [action]
        master.assign_instant_actions(mfr, serial_for(idx), ia)

    def on_connect(agv_id: str) -> None:
        send_actions(index_of(agv_id), make_action("stateRequest", {}))

    def on_state(agv_id: str, st) -> None:
        idx = index_of(agv_id)
        if not 0 <= idx < n:
            return
        pos = st.agv_position
        initialized = pos is not None and pos.position_initialized
        r = robots[idx]
        with r.lock:
            if not initialized and not r.init_sent:
                send_actions(
                    idx,
                    make_action(
                        "initPosition",
                        {"x": "0", "y": "0", "theta": "0", "mapId": MAP_ID, "lastNodeId": "N0"},
                    ),
                )
                r.init_sent = True
                return
            if initialized and not r.started and not state["stopping"]:
                r.started = next_order(idx, r)
                if r.started:
                    with counter_lock:
                        state["started"] += 1

    def on_order_complete(agv_id: str, _order_id: str) -> None:
        idx = index_of(agv_id)
        if not 0 <= idx < n:
            return
        r = robots[idx]
        with r.lock:
            if state["measuring"]:
                r.completed += 1
                r.window_completed += 1
                r.latency_sum += time.perf_counter() - r.sent_at
            if not state["stopping"]:
                next_order(idx, r)

    def on_order_rejected(agv_id: str, _order_id: str, _errors) -> None:
        idx = index_of(agv_id)
        if not 0 <= idx < n:
            return
        with counter_lock:
            state["rejected"] += 1
        r = robots[idx]
        with r.lock:
            if not state["stopping"]:
                next_order(idx, r)

    master.on_connect(on_connect)
    master.on_state(on_state)
    master.on_order_complete(on_order_complete)
    master.on_order_rejected(on_order_rejected)

    master.connect()
    for i in range(n):
        master.onboard_agv_with_interface(INTERFACE_NAME, mfr, serial_for(i))

    # --- Warm-up: wait until every robot has been given its first order ----
    warm_start = time.perf_counter()
    while (
        running.is_set()
        and state["started"] < n
        and time.perf_counter() - warm_start < args.warmup_timeout
    ):
        time.sleep(0.05)
    started = state["started"]
    if started < n:
        LOGGER.warning(
            "only %d/%d robots started within %.0fs", started, n, args.warmup_timeout
        )
    LOGGER.info(
        "[bench] %d/%d robots dispatching; measuring for %s",
        started, n, f"{args.duration:g}s" if args.duration > 0 else "until Ctrl-C",
    )

    # --- Measurement window ------------------------------------------------
    cpu_run0 = cpu_seconds()
    run_start = win_start = time.perf_counter()
    state["rejected"] = 0  # report only rejections inside the window
    state["measuring"] = True
    while running.is_set():
        time.sleep(0.1)
        now = time.perf_counter()
        if args.duration > 0 and now - run_start >= args.duration:
            break
        wall = now - win_start
        if args.bench_period > 0 and wall >= args.bench_period:
            done = 0
            for r in robots:
                with r.lock:
                    done += r.window_completed
                    r.window_completed = 0
            LOGGER.info(
                "[bench] t=%.0fs orders/s=%.0f (%d orders) rss=%.0fMB",
                now - run_start, done / wall, done, rss_mb(),
            )
            win_start = now
    state["measuring"] = False
    run = time.perf_counter() - run_start
    cpu_run = cpu_seconds() - cpu_run0
    state["stopping"] = True

    # --- Report ------------------------------------------------------------
    rates = []
    total = 0
    latency_sum = 0.0
    if not args.summary_only:
        print(f"\n{'robot':<8} {'orders':>10} {'orders/s':>12} {'latency_ms':>12}")
    for i, r in enumerate(robots):
        with r.lock:
            total += r.completed
            latency_sum += r.latency_sum
            rates.append(r.completed / run)
            if not args.summary_only:
                lat = 1e3 * r.latency_sum / r.completed if r.completed else 0.0
                print(
                    f"{serial_for(i):<8} {r.completed:>10d} {r.completed / run:>12.2f} {lat:>12.3f}"
                )
    mean, sd, lo, hi = stats_of(rates)
    LOGGER.info(
        "=== Performance === robots=%d started=%d run=%.2fs orders=%d orders/s=%.1f "
        "per-robot orders/s: mean=%.2f sd=%.2f min=%.2f max=%.2f "
        "latency=%.3fms rejected=%d cpu=%.1f%% rss=%.0fMB",
        n, started, run, total, total / run, mean, sd, lo, hi,
        1e3 * latency_sum / total if total else 0.0, state["rejected"],
        100.0 * cpu_run / run, rss_mb(),
    )

    # --- Shutdown ----------------------------------------------------------
    for i in range(n):
        master.offboard_agv(mfr, serial_for(i))
    master.disconnect()


if __name__ == "__main__":
    main()
