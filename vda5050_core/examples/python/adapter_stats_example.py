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

"""Simulated VDA5050 adapters that count completed orders per robot and in total.

The adapter API has no end-of-order callback, so an order counts as completed
when the last node the robot navigates to for that order_id finishes and no
further node for the same order arrives within ORDER_GRACE_S. The clock starts
when the first order is received. On shutdown, a per-robot summary and the
cumulative orders completed over time are logged.
"""

from __future__ import annotations

import logging
import os
import signal
import threading
import time

from vda5050_core.client import Adapter, ProtocolAdapter
from vda5050_core.transport import create_default_client_shared

BROKER_URI = os.environ.get("MQTT_BROKER", "tcp://localhost:1883")
INTERFACE_NAME = "uagv"
VERSION = "2.0.0"
MANUFACTURER = os.environ.get("VDA5050_MANUFACTURER", "Manufacturer")
NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS", "200"))
ORDER_GRACE_S = float(os.environ.get("ORDER_GRACE_S", "0.5"))
BUCKET_S = float(os.environ.get("STATS_BUCKET_S", "10"))
LOGGER = logging.getLogger(__name__)

running = threading.Event()
running.set()


def signal_handler(signum, _frame) -> None:
    LOGGER.info("System Signal [%d] received. Shutting down ...", signum)
    running.clear()


def serial_for(index: int) -> str:
    return f"S{index + 1:03d}"


class OrderStats:
    """Thread-safe order completion tracker."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._start: float | None = None
        self._pending: dict[int, tuple[str, float]] = {}  # robot -> (order, t)
        self._counts: dict[int, int] = {}
        self._completions: list[float] = []  # seconds since start

    def node_started(self, robot: int, order_id: str) -> None:
        with self._lock:
            if self._start is None:
                self._start = time.monotonic()
                LOGGER.info("First order [%s] received; stats clock started", order_id)
            pending = self._pending.get(robot)
            if pending is not None and pending[0] != order_id:
                self._commit(robot)
            elif pending is not None:
                del self._pending[robot]

    def node_finished(self, robot: int, order_id: str) -> None:
        with self._lock:
            self._pending[robot] = (order_id, time.monotonic())

    def sweep(self, flush: bool = False) -> None:
        now = time.monotonic()
        with self._lock:
            for robot, (_, t) in list(self._pending.items()):
                if flush or now - t >= ORDER_GRACE_S:
                    self._commit(robot)

    def _commit(self, robot: int) -> None:
        order_id, t = self._pending.pop(robot)
        self._counts[robot] = self._counts.get(robot, 0) + 1
        self._completions.append(t - self._start)
        LOGGER.info(
            "Robot [%s] completed order [%s] (robot total %d, overall %d)",
            serial_for(robot), order_id, self._counts[robot], len(self._completions),
        )

    def report(self) -> None:
        self.sweep(flush=True)
        with self._lock:
            total = len(self._completions)
            if self._start is None or total == 0:
                LOGGER.info("No orders were completed")
                return
            LOGGER.info("==== Orders completed per robot ====")
            for robot in sorted(self._counts):
                LOGGER.info("  %s: %d", serial_for(robot), self._counts[robot])
            elapsed = max(self._completions)
            LOGGER.info("  TOTAL: %d", total)
            LOGGER.info(
                "==== Orders completed over time (%.0fs buckets) ====", BUCKET_S
            )
            cumulative = 0
            nbuckets = int(elapsed // BUCKET_S) + 1
            for b in range(nbuckets):
                lo, hi = b * BUCKET_S, (b + 1) * BUCKET_S
                n = sum(1 for c in self._completions if lo <= c < hi)
                cumulative += n
                LOGGER.info(
                    "  %6.0f-%-6.0fs: +%-5d cumulative %d", lo, hi, n, cumulative
                )
            LOGGER.info(
                "Total %d orders in %.1fs since first order (%.2f orders/s)",
                total, elapsed, total / elapsed if elapsed > 0 else 0.0,
            )


def make_numbered_adapter(num: int, stats: OrderStats) -> Adapter:
    mqtt_client = create_default_client_shared(
        BROKER_URI, f"adapter_stats_example_{num}"
    )
    protocol_adapter = ProtocolAdapter.make(
        mqtt_client, INTERFACE_NAME, VERSION, MANUFACTURER, serial_for(num)
    )
    adapter = Adapter.make(protocol_adapter)
    state_manager = adapter.state_manager()

    def on_navigate(node_request, _edge_request, execution) -> None:
        LOGGER.info("Navigating to node [%s]", node_request.node_id())
        order_id = execution.order_id()
        stats.node_started(num, order_id)

        def drive() -> None:
            state_manager.set_driving(True)
            time.sleep(0.1)
            state_manager.set_driving(False)

            execution.finished()
            stats.node_finished(num, order_id)
            LOGGER.info(
                "AMR[%d]: Reached node [%s]", num, node_request.node_id()
            )

            position = node_request.node_position()
            if position is not None:
                state_manager.set_position(
                    position.x,
                    position.y,
                    0.0 if position.theta is None else position.theta,
                    position.map_id,
                )

        threading.Thread(target=drive, daemon=True).start()

    def on_action(request, execution) -> None:
        LOGGER.info("Action of type [%s] requested", request.action_type())

        execution.running()
        time.sleep(0.2)
        execution.finished()

    def on_localize(request, execution) -> None:
        LOGGER.info(
            "Received localization request with point [%s, %s, %s] on map: %s",
            request.x(),
            request.y(),
            request.theta(),
            request.map_id(),
        )

        state_manager.initialize_position(
            request.x(), request.y(), request.theta(), request.map_id()
        )
        execution.finished()

    adapter.on_navigate(on_navigate)
    adapter.on_action(on_action)
    adapter.on_localize(on_localize)
    return adapter


def main() -> None:
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    stats = OrderStats()
    adapters = [make_numbered_adapter(i, stats) for i in range(NUM_ROBOTS)]
    for i, adapter in enumerate(adapters):
        adapter.start()
        LOGGER.info("Adapter [%d]: started ...", i)

    try:
        while running.is_set():
            time.sleep(0.2)
            stats.sweep()
    finally:
        for adapter in adapters:
            threading.Thread(target=adapter.stop, daemon=True).start()
        stats.report()


if __name__ == "__main__":
    main()
