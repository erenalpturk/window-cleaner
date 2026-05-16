#!/usr/bin/env python3
"""Plot benchmark metrics (Phase 4, task 4.3).

Reads ``results/metrics.csv`` (written by metrics_node, one row per run) and
writes four PNGs into ``media/plots/``:

  * coverage_by_run.png  — coverage % per world/run, coloured by outcome
  * duration_box.png     — mission-duration distribution per world (box)
  * collisions_bar.png   — collision count per world/run
  * summary_grouped.png  — per-world grouped summary (mean coverage / duration)

Pure offline tool: standard-library ``csv`` + matplotlib only (no ROS, no
pandas, no numpy). Headless ``Agg`` backend so it runs inside the container.
``run_benchmark.sh`` calls it automatically at the end of a sweep; it can
also be run by hand:

    python3 plot_results.py [--csv PATH] [--outdir DIR]
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")  # headless — must precede pyplot import
import matplotlib.pyplot as plt  # noqa: E402


# Outcome -> bar colour (honest: ABORTED/TIMEOUT visually distinct from DONE).
OUTCOME_COLOR = {
    "DONE": "#2e7d32",       # green
    "ABORTED": "#c62828",    # red
    "TIMEOUT": "#ef6c00",    # orange
    "INTERRUPTED": "#757575",  # grey
    "HARD_TIMEOUT": "#4a148c",  # purple
    "NO_MISSION": "#9e9d24",   # olive (pipeline never reached RUNNING)
}
DEFAULT_COLOR = "#1565c0"


def load_rows(csv_path):
    if not os.path.isfile(csv_path):
        print(f"[plot_results] CSV not found: {csv_path}", file=sys.stderr)
        return []
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append({
                    "world": r["world"],
                    "run": int(r["run_index"]),
                    "result": r["mission_result"],
                    "coverage": float(r["coverage_pct"]),
                    "collisions": int(r["collisions"]),
                    "duration": float(r["duration_s"]),
                    "distance": float(r["distance_m"]),
                })
            except (KeyError, ValueError) as exc:
                print(f"[plot_results] skipping bad row {r}: {exc}",
                      file=sys.stderr)
    return rows


def _labels(rows):
    return [f"{r['world']}\n#{r['run']}" for r in rows]


def plot_coverage_by_run(rows, outdir):
    rows = sorted(rows, key=lambda r: (r["world"], r["run"]))
    colors = [OUTCOME_COLOR.get(r["result"], DEFAULT_COLOR) for r in rows]
    fig, ax = plt.subplots(figsize=(max(6, len(rows) * 0.9), 4.5))
    ax.bar(range(len(rows)), [r["coverage"] for r in rows], color=colors)
    ax.set_xticks(range(len(rows)))
    ax.set_xticklabels(_labels(rows))
    ax.set_ylabel("Coverage (%)")
    ax.set_ylim(0, 100)
    ax.set_title("Coverage per run (bar colour = mission outcome)")
    handles = [plt.Rectangle((0, 0), 1, 1, color=c)
               for c in OUTCOME_COLOR.values()]
    ax.legend(handles, list(OUTCOME_COLOR.keys()), fontsize=8, ncol=3)
    ax.grid(axis="y", alpha=0.3)
    _save(fig, outdir, "coverage_by_run.png")


def plot_duration_box(rows, outdir):
    by_world = defaultdict(list)
    for r in rows:
        by_world[r["world"]].append(r["duration"])
    worlds = sorted(by_world)
    fig, ax = plt.subplots(figsize=(max(5, len(worlds) * 1.6), 4.5))
    ax.boxplot([by_world[w] for w in worlds], labels=worlds)
    ax.set_ylabel("Mission duration (s, sim clock)")
    ax.set_title("Duration distribution per world")
    ax.grid(axis="y", alpha=0.3)
    _save(fig, outdir, "duration_box.png")


def plot_collisions_bar(rows, outdir):
    rows = sorted(rows, key=lambda r: (r["world"], r["run"]))
    fig, ax = plt.subplots(figsize=(max(6, len(rows) * 0.9), 4.0))
    ax.bar(range(len(rows)), [r["collisions"] for r in rows],
           color=DEFAULT_COLOR)
    ax.set_xticks(range(len(rows)))
    ax.set_xticklabels(_labels(rows))
    ax.set_ylabel("Collision periods (< 0.05 m)")
    ax.set_title("Collisions per run")
    ax.grid(axis="y", alpha=0.3)
    _save(fig, outdir, "collisions_bar.png")


def plot_summary_grouped(rows, outdir):
    by_world = defaultdict(list)
    for r in rows:
        by_world[r["world"]].append(r)
    worlds = sorted(by_world)

    def mean(vals):
        return sum(vals) / len(vals) if vals else 0.0

    cov = [mean([r["coverage"] for r in by_world[w]]) for w in worlds]
    dur = [mean([r["duration"] for r in by_world[w]]) for w in worlds]
    dist = [mean([r["distance"] for r in by_world[w]]) for w in worlds]

    x = range(len(worlds))
    w = 0.27
    fig, ax = plt.subplots(figsize=(max(5, len(worlds) * 2.0), 4.5))
    ax.bar([i - w for i in x], cov, w, label="mean coverage (%)",
           color="#2e7d32")
    ax.bar(list(x), dur, w, label="mean duration (s)", color="#1565c0")
    ax.bar([i + w for i in x], dist, w, label="mean distance (m)",
           color="#ef6c00")
    ax.set_xticks(list(x))
    ax.set_xticklabels(worlds)
    ax.set_title("Per-world summary (means)")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    _save(fig, outdir, "summary_grouped.png")


def _save(fig, outdir, name):
    fig.tight_layout()
    path = os.path.join(outdir, name)
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print(f"[plot_results] wrote {path}")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Plot benchmark metrics.")
    ap.add_argument("--csv", default="/workspace/results/metrics.csv")
    ap.add_argument("--outdir", default="/workspace/media/plots")
    args = ap.parse_args(argv)

    rows = load_rows(args.csv)
    if not rows:
        print("[plot_results] no rows — nothing to plot", file=sys.stderr)
        return 1
    os.makedirs(args.outdir, exist_ok=True)
    plot_coverage_by_run(rows, args.outdir)
    plot_duration_box(rows, args.outdir)
    plot_collisions_bar(rows, args.outdir)
    plot_summary_grouped(rows, args.outdir)
    print(f"[plot_results] done — {len(rows)} runs, plots in {args.outdir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
