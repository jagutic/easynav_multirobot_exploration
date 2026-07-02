#!/usr/bin/env python3
import argparse
import csv
import sys
from pathlib import Path
import matplotlib.pyplot as plt


# ── I/O ─────────────────────────────────────────────────────────────────────

def read_csv_data(filepath: Path) -> tuple[list[float], list[float]]:
    """Reads a single CSV and returns (times, cells) as lists of floats."""
    if not filepath.exists():
        sys.exit(f"[ERROR] File not found: {filepath}")

    times, cells = [], []
    with open(filepath, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f, delimiter=",", skipinitialspace=True)
        if reader.fieldnames is None:
            sys.exit(f"[ERROR] Empty or headerless CSV: {filepath}")

        headers = [h.strip() for h in reader.fieldnames if h]
        if "time" not in headers or "cells" not in headers:
            sys.exit(
                f"[ERROR] Columns 'time' and 'cells' not found.\n"
                f"        Available: {headers}\n"
                f"        File: {filepath}"
            )

        for i, row in enumerate(reader, start=2):
            try:
                clean = {k.strip(): v.strip() for k, v in row.items() if k and v}
                if "time" not in clean or "cells" not in clean:
                    continue
                times.append(float(clean["time"]))
                cells.append(float(clean["cells"]))
            except ValueError:
                print(f"[WARNING] Skipping row {i} (non-numeric): {dict(row)}")

    if len(times) < 2:
        sys.exit(f"[ERROR] Need at least 2 valid rows: {filepath}")

    return times, cells


# ── Data preparation ─────────────────────────────────────────────────────────

def load_experiment(csv_dir: Path) -> dict[str, tuple[list[float], list[float]]]:
    """Read all CSVs in csv_dir; return {stem: (times, cells)}."""
    csv_files = sorted(csv_dir.glob("*.csv"))
    data = {}
    for fp in csv_files:
        times, cells = read_csv_data(fp)
        data[fp.stem] = (times, cells)
    return data


def relativize(
    raw: dict[str, tuple[list[float], list[float]]],
    total_cells: int | None,
) -> tuple[dict[str, tuple[list[float], list[float]]], float]:
    """
    Sort by time, relativize timestamps to this experiment's own t0,
    optionally convert cells to %. Returns (rel_data, max_relative_time).
    """
    exp_t0 = min(min(times) for times, _ in raw.values())
    rel_data: dict[str, tuple[list[float], list[float]]] = {}
    max_t = 0.0

    for name, (times, cells) in raw.items():
        pairs = sorted(zip(times, cells), key=lambda p: p[0])
        s_times, s_cells = zip(*pairs)
        rel_times = [t - exp_t0 for t in s_times]
        y = [c / total_cells * 100.0 for c in s_cells] if total_cells else list(s_cells)
        rel_data[name] = (rel_times, y)
        max_t = max(max_t, rel_times[-1])

    return rel_data, max_t


# ── Plotting ─────────────────────────────────────────────────────────────────

def plot_experiment(
    rel_data: dict[str, tuple[list[float], list[float]]],
    output_dir: Path,
    dir_name: str,
    xlim: float,
    show: bool,
    relative: bool,
) -> None:
    """Save one PNG per CSV (one per robot)."""

    y_label = "Explored (%)" if relative else "Explored Cells"
    pct_fmt = plt.FuncFormatter(lambda v, _: f"{v:.1f}%")

    for name, (rel_times, y_values) in rel_data.items():
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(rel_times, y_values, marker="o", linestyle="-", label=name)
        ax.set_title(f"Exploration over Time — {dir_name} / {name}")
        ax.set_xlabel("Time (s) [Relative to experiment start]")
        ax.set_ylabel(y_label)
        ax.set_xlim(0, xlim)
        if relative:
            ax.set_ylim(0, 105)
            ax.yaxis.set_major_formatter(pct_fmt)
        ax.grid(True)
        ax.legend()
        fig.tight_layout()

        save_path = output_dir / f"{dir_name}_{name}.png"
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  [OK] {save_path.name}")
        plt.close(fig)

    if show:
        plt.show()


# ── Directory helpers ────────────────────────────────────────────────────────

def build_dir_name(csv_dir: Path, root: Path) -> str:
    try:
        return "_".join(csv_dir.relative_to(root).parts)
    except ValueError:
        return csv_dir.name


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Plot exploration progress from CSVs (one per robot).\n\n"
            "Time is relative to each experiment's own start, but the X axis\n"
            "range is shared across all experiments for easy comparison.\n\n"
            "Use --relative N to express the Y axis as %% of N total map cells."
        )
    )
    parser.add_argument(
        "root_dir",
        help="Experiment directory, or parent directory when --recursive is used",
    )
    parser.add_argument(
        "-r", "--recursive",
        action="store_true",
        help="Process every directory containing CSVs under root_dir, at any depth",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display plots interactively in addition to saving them",
    )
    parser.add_argument(
        "--relative",
        type=int,
        default=None,
        metavar="N",
        help="Express explored cells as %% of N total free cells of the map",
    )
    args = parser.parse_args()

    if args.relative is not None and args.relative <= 0:
        parser.error("--relative N requires a positive integer")

    root = Path(args.root_dir).resolve()
    if not root.is_dir():
        sys.exit(f"[ERROR] Not a directory: {root}")

    # Collect directories to process
    if args.recursive:
        csv_dirs = sorted(set(p.parent for p in root.rglob("*.csv")))
        if not csv_dirs:
            sys.exit(f"[ERROR] No CSV files found under: {root}")
        effective_root = root.parent  # include root folder name in dir_name
    else:
        csv_dirs = [root]
        effective_root = root.parent

    # ── Pass 1: load & relativize all experiments, find shared xlim ──────────
    print("[1/2] Loading data …")
    experiments: list[tuple[Path, str, dict]] = []
    global_max_t = 0.0

    for csv_dir in csv_dirs:
        csv_files = sorted(csv_dir.glob("*.csv"))
        if not csv_files:
            print(f"  [SKIP] No CSVs in {csv_dir}")
            continue

        dir_name = build_dir_name(csv_dir, effective_root)
        raw = load_experiment(csv_dir)
        rel_data, max_t = relativize(raw, args.relative)
        global_max_t = max(global_max_t, max_t)
        experiments.append((csv_dir, dir_name, rel_data))
        print(f"  [loaded] {dir_name}  (duration: {max_t:.1f} s)")

    if not experiments:
        sys.exit("[ERROR] No valid experiments found.")

    print(f"\n  Shared X range: 0 – {global_max_t:.1f} s")

    # ── Pass 2: plot everything with the shared xlim ─────────────────────────
    print("\n[2/2] Saving plots …")
    for csv_dir, dir_name, rel_data in experiments:
        print(f"\n[>>] {dir_name}")
        plot_experiment(
            rel_data=rel_data,
            output_dir=csv_dir,
            dir_name=dir_name,
            xlim=global_max_t,
            show=args.show,
            relative=args.relative is not None,
        )

    print("\n[DONE] All figures saved.")


if __name__ == "__main__":
    main()