"""Merge multiple compatible RLDS/TFDS datasets into one new dataset."""

from __future__ import annotations

import argparse
import copy
import re
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
import tensorflow_datasets as tfds  # type: ignore


DEFAULT_VERSION = "1.0.0"


def _resolve_builder_dir(path: Path) -> Path:
    path = path.expanduser().resolve()

    if (path / "dataset_info.json").is_file():
        return path

    version_dir = path / DEFAULT_VERSION
    if (version_dir / "dataset_info.json").is_file():
        return version_dir

    matches = sorted(path.rglob("dataset_info.json"))
    if len(matches) == 1:
        return matches[0].parent

    raise FileNotFoundError(
        "Could not resolve a TFDS builder directory from %s. Pass a path like "
        "~/tensorflow_datasets/franka_vla_200/1.0.0." % path
    )


def _dataset_name_to_class_name(dataset_name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", dataset_name).strip("_")
    parts = [part for part in cleaned.split("_") if part]
    class_name = "".join(part[:1].upper() + part[1:] for part in parts) or "MergedDataset"
    if class_name[0].isdigit():
        class_name = "Dataset" + class_name
    return class_name


def _to_example_value(value: Any) -> Any:
    if hasattr(value, "numpy"):
        value = value.numpy()

    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")

    if isinstance(value, np.ndarray):
        if value.shape == ():
            return value.item()
        return value

    if isinstance(value, dict):
        return {key: _to_example_value(item) for key, item in value.items()}

    if isinstance(value, (list, tuple)):
        return [_to_example_value(item) for item in value]

    if hasattr(value, "item"):
        try:
            return value.item()
        except ValueError:
            pass

    return value


def _iter_step_dicts(steps: Any) -> Iterable[dict[str, Any]]:
    if hasattr(steps, "as_numpy_iterator"):
        yield from steps.as_numpy_iterator()
        return

    try:
        yield from tfds.as_numpy(steps)
        return
    except TypeError:
        pass

    if isinstance(steps, dict) and "timestamp_ns" in steps:
        timestamps = _to_example_value(steps["timestamp_ns"])
        if hasattr(timestamps, "tolist"):
            timestamps = timestamps.tolist()
        for index, _ in enumerate(timestamps):
            yield {
                key: _to_example_value(value)[index]
                for key, value in steps.items()
                if key != "observation"
            }


def _materialize_episode(raw_episode: dict[str, Any]) -> dict[str, Any]:
    return {
        "steps": [
            _to_example_value(step)
            for step in _iter_step_dicts(raw_episode["steps"])
        ],
        "episode_metadata": _to_example_value(raw_episode["episode_metadata"]),
    }


def _safe_key(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", text).strip("_") or "episode"


def _unique_key(candidate: str, used_keys: set[str]) -> str:
    candidate = _safe_key(candidate)
    if candidate not in used_keys:
        used_keys.add(candidate)
        return candidate

    index = 1
    while f"{candidate}_{index}" in used_keys:
        index += 1
    unique = f"{candidate}_{index}"
    used_keys.add(unique)
    return unique


def _make_builder_class(
    *,
    source_dirs: Sequence[Path],
    dataset_name: str,
    split: str,
    rewrite_duplicate_episode_ids: bool,
):
    source_builders = [tfds.builder_from_directory(str(path)) for path in source_dirs]
    reference_features = source_builders[0].info.features
    reference_features_repr = repr(reference_features)

    for builder in source_builders[1:]:
        if repr(builder.info.features) != reference_features_repr:
            raise ValueError(
                "Source datasets do not have identical TFDS features. "
                "Merge only datasets converted with the same image/action/state settings."
            )

    class MergedFrankaVla(tfds.core.GeneratorBasedBuilder):
        VERSION = tfds.core.Version(DEFAULT_VERSION)
        RELEASE_NOTES = {DEFAULT_VERSION: "Merged compatible Franka VLA RLDS datasets."}

        def _info(self):
            return tfds.core.DatasetInfo(
                builder=self,
                description="Merged Franka VLA demonstrations in RLDS/TFDS format.",
                features=copy.deepcopy(reference_features),
            )

        def _split_generators(self, dl_manager):
            return {"train": self._generate_examples()}

        def _generate_examples(self):
            used_keys: set[str] = set()
            used_episode_ids: set[str] = set()

            for source_index, source_builder in enumerate(source_builders):
                dataset = source_builder.as_dataset(split=split, shuffle_files=False)

                for episode_index, raw_episode in enumerate(dataset):
                    episode = _materialize_episode(raw_episode)
                    metadata = episode["episode_metadata"]
                    episode_id = str(metadata.get("episode_id", f"episode_{episode_index}"))

                    if rewrite_duplicate_episode_ids and episode_id in used_episode_ids:
                        episode_id = f"source_{source_index}_{episode_id}"
                        metadata["episode_id"] = episode_id
                    used_episode_ids.add(episode_id)

                    key = _unique_key(
                        f"source_{source_index}_{episode_id}_{episode_index}",
                        used_keys,
                    )
                    yield key, episode

    class_name = _dataset_name_to_class_name(dataset_name)
    MergedFrankaVla.__name__ = class_name
    MergedFrankaVla.__qualname__ = class_name
    return MergedFrankaVla


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Merge multiple compatible RLDS/TFDS datasets into one new dataset."
    )
    parser.add_argument(
        "sources",
        nargs="+",
        type=Path,
        help=(
            "Source TFDS builder directories, for example "
            "~/tensorflow_datasets/franka_vla_200/1.0.0."
        ),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        required=True,
        help="TFDS root directory for the merged dataset, for example ~/tensorflow_datasets.",
    )
    parser.add_argument(
        "--dataset-name",
        required=True,
        help="Merged dataset name, for example franka_vla_300.",
    )
    parser.add_argument(
        "--split",
        default="train",
        help="Source split to merge. Defaults to train.",
    )
    parser.add_argument(
        "--no-rewrite-duplicate-episode-ids",
        action="store_true",
        help="Preserve duplicate episode_metadata.episode_id values instead of making them unique.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite the merged TFDS dataset if it already exists.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    args = _parse_args(argv)

    source_dirs = [_resolve_builder_dir(path) for path in args.sources]
    output_root = args.output_root.expanduser().resolve()

    builder_class = _make_builder_class(
        source_dirs=source_dirs,
        dataset_name=args.dataset_name,
        split=args.split,
        rewrite_duplicate_episode_ids=not args.no_rewrite_duplicate_episode_ids,
    )
    builder = builder_class(data_dir=str(output_root))

    output_dir = Path(builder.data_dir)
    if output_dir.exists() and not args.overwrite:
        raise SystemExit(
            f"Output already exists: {output_dir}. Use --overwrite to rebuild it."
        )

    mode = (
        tfds.GenerateMode.FORCE_REDOWNLOAD
        if args.overwrite
        else tfds.GenerateMode.REUSE_DATASET_IF_EXISTS
    )
    builder.download_and_prepare(
        download_config=tfds.download.DownloadConfig(download_mode=mode)
    )

    print(f"Wrote merged RLDS/TFDS dataset to: {builder.data_dir}")
    print("Load it with:")
    print(f"  tfds.builder_from_directory({str(builder.data_dir)!r}).as_dataset(split='train')")


if __name__ == "__main__":
    main()
