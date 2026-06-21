#!/usr/bin/env python3
import argparse
import csv
import sys
from pathlib import Path
import matplotlib.pyplot as plt


def read_csv_data(filepath: Path) -> tuple[list[float], list[float]]:
    """Reads a single CSV and returns (times, cells) as lists of floats."""
    if not filepath.exists():
        sys.exit(f"[ERROR] File not found: {filepath}")

    times = []
    cells = []

    with open(filepath, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f, delimiter=",", skipinitialspace=True)

        if reader.fieldnames is None:
            sys.exit(f"[ERROR] The CSV file is empty or has no headers: {filepath}")

        headers = [h.strip() for h in reader.fieldnames if h]
        if "time" not in headers or "cells" not in headers:
            sys.exit(
                f"[ERROR] Columns 'time' and 'cells' not found.\n"
                f"        Available columns: {headers}\n"
                f"        File: {filepath}"
            )

        for i, row in enumerate(reader, start=2):
            try:
                clean_row = {k.strip(): v.strip() for k, v in row.items() if k and v}

                if "time" not in clean_row or "cells" not in clean_row:
                    continue  # Skip rows without time or cells

                times.append(float(clean_row["time"]))
                cells.append(float(clean_row["cells"]))

            except ValueError:
                print(f"[WARNING] Ignoring row {i} due to non-numeric values: {dict(row)}")
                continue

    if len(times) < 2:
        sys.exit(f"[ERROR] At least 2 valid rows are required to plot: {filepath}")

    return times, cells


def plot_data(all_data: dict[str, tuple[list[float], list[float]]]) -> None:
    """Generates one plot per CSV and a final combined plot."""

    # Calculate global t0 across all CSVs
    global_t0 = min(times[0] for times, _ in all_data.values())

    # 1. Generate one plot per CSV
    for name, (times, cells) in all_data.items():
        # Sort by time
        sorted_pairs = sorted(zip(times, cells), key=lambda p: p[0])
        sorted_times, sorted_cells = zip(*sorted_pairs)

        # Relativize to the first time of this CSV
        t0 = sorted_times[0]
        rel_times = [t - t0 for t in sorted_times]

        plt.figure(figsize=(10, 6))
        plt.plot(rel_times, sorted_cells, marker='o', linestyle='-', label=name)
        plt.title(f"Explored Cells over Time - {name}")
        plt.xlabel("Time (s) [Relative to start]")
        plt.ylabel("Explored Cells")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

    # 2. Generate combined plot with all CSVs and total
    plt.figure(figsize=(10, 6))

    all_sorted = {}
    for name, (times, cells) in all_data.items():
        sorted_pairs = sorted(zip(times, cells), key=lambda p: p[0])
        sorted_times, sorted_cells = zip(*sorted_pairs)
        rel_times = [t - global_t0 for t in sorted_times]
        all_sorted[name] = (rel_times, sorted_cells)
        plt.plot(rel_times, sorted_cells, marker='o', linestyle='-', label=name)

    # Calculate total explored cells at each unique timestamp
    all_time_points = sorted(set(t for rel_times, _ in all_sorted.values() for t in rel_times))
    totals = []
    for t in all_time_points:
        total = 0
        for rel_times, sorted_cells in all_sorted.values():
            # Forward-fill: take the last known value at or before t
            last = 0.0
            for rt, c in zip(rel_times, sorted_cells):
                if rt <= t:
                    last = c
            total += last
        totals.append(total)

    plt.plot(all_time_points, totals, marker='o', linestyle='-', color='black', linewidth=2, label='Total')
    plt.title("Explored Cells over Time (All Robots)")
    plt.xlabel("Time (s) [Relative to global start]")
    plt.ylabel("Explored Cells")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # Display all generated plots
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Plot exploration progress from multiple CSVs, one per robot."
    )
    parser.add_argument("csv_dir", help="Path to the directory containing the CSV files")
    args = parser.parse_args()

    csv_dir = Path(args.csv_dir)
    if not csv_dir.is_dir():
        sys.exit(f"[ERROR] Not a directory: {csv_dir}")

    csv_files = sorted(csv_dir.glob("*.csv"))
    if not csv_files:
        sys.exit(f"[ERROR] No CSV files found in: {csv_dir}")

    # Read all CSVs — key is the filename stem (e.g. "robot1_exploration_data")
    all_data = {}
    for filepath in csv_files:
        times, cells = read_csv_data(filepath)
        all_data[filepath.stem] = (times, cells)

    plot_data(all_data)


if __name__ == "__main__":
    main()
