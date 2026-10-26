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

"""Plot the CSV written by run_order_benchmark.py.

Left: total orders/s vs robot count. Right: per-robot orders/s vs robot count.
Lines are the mean over repeats, error bars one standard deviation.

    python plot_order_benchmark.py [order_throughput.csv] [-o order_throughput.png]
"""

from __future__ import annotations

import argparse
import csv
import statistics
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

LABELS = {"cpp": "adapter_example (C++)", "py": "adapter_example (Python)"}
COLORS = {"cpp": "#1f77b4", "py": "#ff7f0e"}
MARKERS = {"cpp": "o", "py": "s"}


def load(path: Path) -> dict[str, dict[int, list[tuple[float, float]]]]:
    """adapter -> robots -> [(orders/s, orders/s per robot)] over repeats."""
    data: dict[str, dict[int, list[tuple[float, float]]]] = defaultdict(lambda: defaultdict(list))
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            n = int(row["robots"])
            total = float(row["orders_per_s"])
            data[row["adapter"]][n].append((total, total / n))
    return data


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("csv", nargs="?", type=Path, default=Path("order_throughput.csv"))
    p.add_argument("-o", "--output", type=Path, default=Path("order_throughput.png"))
    args = p.parse_args()

    data = load(args.csv)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    for ax, idx, title, ylabel in (
        (axes[0], 0, "Total order throughput", "orders / s"),
        (axes[1], 1, "Per-robot order throughput", "orders / s / robot"),
    ):
        for adapter, by_n in sorted(data.items()):
            ns = sorted(by_n)
            mean = [statistics.fmean(v[idx] for v in by_n[n]) for n in ns]
            sd = [
                statistics.pstdev([v[idx] for v in by_n[n]]) if len(by_n[n]) > 1 else 0.0
                for n in ns
            ]
            ax.errorbar(
                ns, mean, yerr=sd, label=LABELS.get(adapter, adapter),
                color=COLORS.get(adapter), marker=MARKERS.get(adapter, "o"),
                capsize=3, linewidth=1.8,
            )
        ax.set_title(title)
        ax.set_xlabel("robots")
        ax.set_ylabel(ylabel)
        ax.set_ylim(bottom=0)
        ax.grid(alpha=0.3)
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].legend(frameon=False)
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
