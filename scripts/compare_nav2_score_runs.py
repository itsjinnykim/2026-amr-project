#!/usr/bin/env python3
"""Compare two CSV files produced by nav2_score_monitor.py."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from statistics import mean
from typing import Iterable, Optional


@dataclass
class RunSummary:
    name: str
    samples: int
    final_score: float
    mean_score: float
    final_path_m: float
    min_obstacle_m: Optional[float]
    mean_inflated_cells: float
    mean_high_cost_cells: float
    mean_cost: float
    final_rotation_rad: float
    final_energy_cost: float
    final_velocity_change: float
    final_acceleration_change: float
    stop_count: int
    restart_count: int
    blocked_samples: int


def _to_float(value: str) -> Optional[float]:
    if value is None or value == "":
        return None
    return float(value)


def summarize(name: str, path: str) -> RunSummary:
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise ValueError(f"No rows in {path}")

    scores = [float(row["score"]) for row in rows]
    obstacles = [
        value
        for value in (_to_float(row.get("nearest_obstacle_m", "")) for row in rows)
        if value is not None
    ]
    blocked_samples = sum(1 for row in rows if row.get("blocked") == "true")
    inflated_cells = [float(row.get("inflated_cells", 0.0) or 0.0) for row in rows]
    high_cost_cells = [float(row.get("high_cost_cells", 0.0) or 0.0) for row in rows]
    mean_costs = [float(row.get("mean_cost", 0.0) or 0.0) for row in rows]
    last = rows[-1]

    return RunSummary(
        name=name,
        samples=len(rows),
        final_score=float(last["score"]),
        mean_score=mean(scores),
        final_path_m=float(last["path_length_m"]),
        min_obstacle_m=min(obstacles) if obstacles else None,
        mean_inflated_cells=mean(inflated_cells),
        mean_high_cost_cells=mean(high_cost_cells),
        mean_cost=mean(mean_costs),
        final_rotation_rad=float(last["accumulated_rotation_rad"]),
        final_energy_cost=float(last.get("estimated_energy_cost", 0.0) or 0.0),
        final_velocity_change=float(
            last.get("accumulated_velocity_change", 0.0) or 0.0
        ),
        final_acceleration_change=float(
            last.get("accumulated_acceleration_change", 0.0) or 0.0
        ),
        stop_count=int(last.get("stop_count", 0) or 0),
        restart_count=int(last.get("restart_count", 0) or 0),
        blocked_samples=blocked_samples,
    )


def format_obstacle(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value:.3f}"


def print_summary(summary: RunSummary) -> None:
    print(f"[{summary.name}]")
    print(f"  samples: {summary.samples}")
    print(f"  final_score: {summary.final_score:.3f}")
    print(f"  mean_score: {summary.mean_score:.3f}")
    print(f"  final_path_m: {summary.final_path_m:.3f}")
    print(f"  min_obstacle_m: {format_obstacle(summary.min_obstacle_m)}")
    print(f"  mean_inflated_cells: {summary.mean_inflated_cells:.1f}")
    print(f"  mean_high_cost_cells: {summary.mean_high_cost_cells:.1f}")
    print(f"  mean_cost: {summary.mean_cost:.3f}")
    print(f"  final_rotation_rad: {summary.final_rotation_rad:.3f}")
    print(f"  final_energy_cost: {summary.final_energy_cost:.3f}")
    print(f"  final_velocity_change: {summary.final_velocity_change:.3f}")
    print(f"  final_acceleration_change: {summary.final_acceleration_change:.3f}")
    print(f"  stop_count: {summary.stop_count}")
    print(f"  restart_count: {summary.restart_count}")
    print(f"  blocked_samples: {summary.blocked_samples}")


def delta(tuned: RunSummary, baseline: RunSummary) -> Iterable[str]:
    yield f"score_delta(final): {tuned.final_score - baseline.final_score:+.3f}"
    yield f"score_delta(mean): {tuned.mean_score - baseline.mean_score:+.3f}"
    yield f"path_delta_m(final): {tuned.final_path_m - baseline.final_path_m:+.3f}"
    yield f"rotation_delta_rad(final): {tuned.final_rotation_rad - baseline.final_rotation_rad:+.3f}"
    yield f"energy_delta(final): {tuned.final_energy_cost - baseline.final_energy_cost:+.3f}"
    yield f"velocity_change_delta(final): {tuned.final_velocity_change - baseline.final_velocity_change:+.3f}"
    yield f"acceleration_change_delta(final): {tuned.final_acceleration_change - baseline.final_acceleration_change:+.3f}"
    yield f"stop_delta_count: {tuned.stop_count - baseline.stop_count:+d}"
    yield f"restart_delta_count: {tuned.restart_count - baseline.restart_count:+d}"
    yield f"blocked_delta_samples: {tuned.blocked_samples - baseline.blocked_samples:+d}"
    yield f"mean_inflated_cells_delta: {tuned.mean_inflated_cells - baseline.mean_inflated_cells:+.1f}"
    yield f"mean_high_cost_cells_delta: {tuned.mean_high_cost_cells - baseline.mean_high_cost_cells:+.1f}"
    yield f"mean_cost_delta: {tuned.mean_cost - baseline.mean_cost:+.3f}"
    if tuned.min_obstacle_m is not None and baseline.min_obstacle_m is not None:
        yield f"min_obstacle_delta_m: {tuned.min_obstacle_m - baseline.min_obstacle_m:+.3f}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", required=True, help="Baseline monitor CSV")
    parser.add_argument("--tuned", required=True, help="Tuned monitor CSV")
    args = parser.parse_args()

    baseline = summarize("baseline", args.baseline)
    tuned = summarize("tuned", args.tuned)
    print_summary(baseline)
    print()
    print_summary(tuned)
    print()
    print("[delta tuned - baseline]")
    for line in delta(tuned, baseline):
        print(f"  {line}")


if __name__ == "__main__":
    main()
