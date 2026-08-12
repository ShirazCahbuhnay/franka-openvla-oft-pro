"""Convert an offline W&B run into a local CSV file and loss-chart PNG.

Run this with the same Python environment used for ``finetune.py``.  It reads
the local ``run-<id>.wandb`` history directly and never logs in to, uploads to,
or contacts the W&B service.

Example:
    python vla-scripts/plot_wandb_offline_loss.py \
        ./runs/wandb/wandb/offline-run-YYYYMMDD_HHMMSS-<id>
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections.abc import Iterable
from pathlib import Path
from typing import Any


DEFAULT_METRIC = "VLA Train/Loss"
DEFAULT_SMOOTH_WINDOW = 25


def _remove_repo_root_from_import_path() -> None:
    """Avoid importing a sibling ``wandb/`` run-data directory as a package."""

    repo_root = Path(__file__).resolve().parent.parent
    for entry in list(sys.path):
        entry_path = Path(entry or ".").resolve()
        if entry_path == repo_root:
            sys.path.remove(entry)


def _find_run_file(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.is_file() and path.suffix == ".wandb":
        return path
    if path.is_dir():
        run_files = sorted(path.glob("run-*.wandb"))
        if len(run_files) == 1:
            return run_files[0]
        if not run_files:
            raise FileNotFoundError(
                f"No run-<id>.wandb file found in offline run directory: {path}"
            )
        raise ValueError(f"More than one .wandb file found in: {path}")
    raise FileNotFoundError(f"Offline W&B run path does not exist: {path}")


def _item_key(item: Any) -> str:
    key = str(getattr(item, "key", ""))
    if key:
        return key
    nested_key = list(getattr(item, "nested_key", []))
    return ".".join(str(part) for part in nested_key)


def _item_value(item: Any) -> Any:
    value_json = getattr(item, "value_json", "")
    if not value_json:
        return None
    try:
        return json.loads(value_json)
    except json.JSONDecodeError:
        return value_json


def _numeric_value(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _history_rows(run_file: Path) -> list[dict[str, Any]]:
    """Read W&B history protobuf records from one offline ``.wandb`` file."""

    _remove_repo_root_from_import_path()
    try:
        from wandb.proto import wandb_internal_pb2  # type: ignore
        from wandb.sdk.internal import datastore  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "The W&B package is unavailable. Run this script with the same "
            "Python environment used for finetune.py."
        ) from exc

    store = datastore.DataStore()
    store.open_for_scan(str(run_file))
    rows: list[dict[str, Any]] = []

    while True:
        scanned = store.scan_record()
        if scanned is None:
            break
        serialized_record = scanned[1] if isinstance(scanned, tuple) else scanned
        record = wandb_internal_pb2.Record()
        record.ParseFromString(serialized_record)
        if record.WhichOneof("record_type") != "history":
            continue

        row = {
            _item_key(item): _item_value(item)
            for item in record.history.item
            if _item_key(item)
        }
        if row:
            rows.append(row)

    return rows


def _available_metrics(rows: Iterable[dict[str, Any]]) -> list[str]:
    metrics: set[str] = set()
    for row in rows:
        for key, value in row.items():
            if _numeric_value(value) is not None and not key.startswith("_"):
                metrics.add(key)
    return sorted(metrics)


def _points_for_metric(
    rows: Iterable[dict[str, Any]], metric: str
) -> list[tuple[int, float]]:
    """Return one final value per W&B step, sorted by optimizer step."""

    values_by_step: dict[int, float] = {}
    fallback_step = 0
    for row in rows:
        value = _numeric_value(row.get(metric))
        if value is None:
            continue
        raw_step = _numeric_value(row.get("_step"))
        step = int(raw_step) if raw_step is not None else fallback_step
        values_by_step[step] = value
        fallback_step += 1
    return sorted(values_by_step.items())


def _moving_average(values: list[float], window: int) -> list[float]:
    if window <= 1:
        return values
    averages: list[float] = []
    running_total = 0.0
    for index, value in enumerate(values):
        running_total += value
        if index >= window:
            running_total -= values[index - window]
        averages.append(running_total / min(index + 1, window))
    return averages


def _write_csv(points: list[tuple[int, float]], csv_path: Path) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["optimizer_step", "loss"])
        writer.writerows(points)


def _write_plot(
    *,
    points: list[tuple[int, float]],
    metric: str,
    run_file: Path,
    output_path: Path,
    smooth_window: int,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is unavailable. Install the OpenVLA-OFT environment "
            "dependencies before creating a local loss graph."
        ) from exc

    steps = [step for step, _ in points]
    losses = [loss for _, loss in points]
    smoothed = _moving_average(losses, smooth_window)

    figure, axis = plt.subplots(figsize=(10, 6), layout="constrained")
    axis.plot(steps, losses, color="#4C78A8", alpha=0.28, linewidth=1.0, label="Loss")
    axis.plot(
        steps,
        smoothed,
        color="#F58518",
        linewidth=2.0,
        label=f"Moving average ({smooth_window} points)",
    )
    axis.set_title(f"{metric} - {run_file.parent.name}")
    axis.set_xlabel("Optimizer step")
    axis.set_ylabel("Loss")
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert an offline W&B run into a local loss CSV and PNG without "
            "logging in or syncing data."
        )
    )
    parser.add_argument(
        "run_path",
        type=Path,
        help="Offline run directory or its run-<id>.wandb file.",
    )
    parser.add_argument(
        "--metric",
        default=DEFAULT_METRIC,
        help=f"Numeric W&B metric to plot. Default: {DEFAULT_METRIC!r}.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PNG path. Defaults to training_loss.png beside the .wandb file.",
    )
    parser.add_argument(
        "--csv-output",
        type=Path,
        default=None,
        help="Output CSV path. Defaults to training_loss.csv beside the .wandb file.",
    )
    parser.add_argument(
        "--smooth-window",
        type=int,
        default=DEFAULT_SMOOTH_WINDOW,
        help=f"Moving-average window in logged points. Default: {DEFAULT_SMOOTH_WINDOW}.",
    )
    parser.add_argument(
        "--list-metrics",
        action="store_true",
        help="Print all numeric metrics found in the run, then exit.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.smooth_window < 1:
        raise SystemExit("--smooth-window must be at least 1")

    run_file = _find_run_file(args.run_path)
    rows = _history_rows(run_file)
    metrics = _available_metrics(rows)
    if args.list_metrics:
        if not metrics:
            raise SystemExit(f"No numeric history metrics found in: {run_file}")
        print("Numeric metrics:")
        for metric in metrics:
            print(f"  {metric}")
        return

    points = _points_for_metric(rows, args.metric)
    if not points:
        available = ", ".join(metrics) if metrics else "none"
        raise SystemExit(
            f"Metric {args.metric!r} was not found in {run_file}. "
            f"Available numeric metrics: {available}"
        )

    output_path = (args.output or run_file.parent / "training_loss.png").expanduser()
    csv_path = (args.csv_output or run_file.parent / "training_loss.csv").expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    _write_csv(points, csv_path)
    _write_plot(
        points=points,
        metric=args.metric,
        run_file=run_file,
        output_path=output_path,
        smooth_window=args.smooth_window,
    )

    print(f"Read {len(points)} unique optimizer-step value(s) from: {run_file}")
    print(f"Wrote local loss CSV: {csv_path}")
    print(f"Wrote local loss graph: {output_path}")


if __name__ == "__main__":
    main()
