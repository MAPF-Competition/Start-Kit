#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot cumulative and per-interval completed-task trends."
    )
    parser.add_argument("input", type=Path, help="Task trend text file produced by lifelong")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output image path. Defaults to <input_stem>.png",
    )
    parser.add_argument(
        "--title",
        default="Task Completion Trend",
        help="Plot title",
    )
    return parser.parse_args()


def read_trend_file(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"Input file does not exist: {path}")

    timesteps = []
    cumulative = []
    interval = []

    with path.open("r", encoding="utf-8") as handle:
        header = handle.readline().strip().split()
        expected = ["timestep", "cumulative_finished", "interval_finished"]
        if header != expected:
            raise ValueError(
                f"Unexpected header in {path}. Expected: {' '.join(expected)}"
            )

        for line_number, raw_line in enumerate(handle, start=2):
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 3:
                raise ValueError(
                    f"Invalid row at line {line_number}: expected 3 columns, got {len(parts)}"
                )
            t, c, i = map(int, parts)
            timesteps.append(t)
            cumulative.append(c)
            interval.append(i)

    if not timesteps:
        raise ValueError(f"No data rows found in {path}")

    return timesteps, cumulative, interval


def main():
    args = parse_args()
    output_path = args.output or args.input.with_suffix(".png")

    timesteps, cumulative, interval = read_trend_file(args.input)

    fig, ax1 = plt.subplots(figsize=(10, 5.5))
    ax2 = ax1.twinx()

    cumulative_line = ax1.plot(
        timesteps,
        cumulative,
        color="#0b6e4f",
        linewidth=2.5,
        marker="o",
        label="Cumulative finished",
    )
    interval_bars = ax2.bar(
        timesteps,
        interval,
        width=max(1, int(0.6 * min([b - a for a, b in zip(timesteps, timesteps[1:])] or [10]))),
        color="#f4a261",
        alpha=0.45,
        label="Finished in interval",
    )

    ax1.set_title(args.title)
    ax1.set_xlabel("Timestep")
    ax1.set_ylabel("Cumulative tasks finished", color="#0b6e4f")
    ax2.set_ylabel("Tasks finished in interval", color="#9c4f00")
    ax1.grid(True, linestyle="--", alpha=0.3)

    handles = cumulative_line + [interval_bars]
    labels = [handle.get_label() for handle in handles]
    ax1.legend(handles, labels, loc="upper left")

    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    print(f"Saved plot to {output_path}")


if __name__ == "__main__":
    main()
