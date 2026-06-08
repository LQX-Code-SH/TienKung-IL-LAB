# -*- coding: utf-8 -*-
import argparse
import json
import logging
import shutil
from pathlib import Path

import cv2
import h5py
import numpy as np
from tqdm import tqdm

from lerobot.datasets import LeRobotDataset

# lerobot 0.5.x validate_feature_numpy_array compares numpy .shape (tuple)
# against feature spec shape (list) directly with !=, which always fails.
# Patch to normalize both sides to tuple before comparison.
try:
    from lerobot.datasets import feature_utils as _fu

    _orig_validate = _fu.validate_feature_numpy_array

    def _patched_validate(name, expected_dtype, expected_shape, value):
        if isinstance(value, np.ndarray) and isinstance(expected_shape, list):
            expected_shape = tuple(expected_shape)
        return _orig_validate(name, expected_dtype, expected_shape, value)

    _fu.validate_feature_numpy_array = _patched_validate
except Exception:
    pass


def load_config(config_path: str) -> tuple[dict, dict]:
    """Load config JSON, extracting hdf5_mapping and returning (features, mapping)."""
    with open(config_path, "r") as f:
        config = json.load(f)

    mapping = config.pop("hdf5_mapping", None)
    features = config  # remaining keys are the LeRobot feature schema

    if mapping is None:
        raise ValueError(
            f"Config '{config_path}' is missing 'hdf5_mapping'. "
            "Add an hdf5_mapping section or use an updated config file."
        )

    logging.info(f"Loaded features config with {len(features)} feature keys")
    logging.info(f"Loaded HDF5 mapping for {len(mapping)} fields")
    return features, mapping


def _read_hdf5_part(file, spec) -> np.ndarray:
    """Read a single part from HDF5 according to a mapping spec element.

    spec can be:
      - str: direct HDF5 key, read full array
      - dict: {"hdf5_key": ..., "repeat": N, "pad": [v1, ...], "invert": true}
        "repeat" (optional): repeat the value N times along last axis
        "pad" (optional): append constant values along last axis
        "invert" (optional): if true, apply 1 - value to flip the range
    """
    if isinstance(spec, str):
        return np.array(file[spec])

    hdf5_key = spec["hdf5_key"]
    data = np.array(file[hdf5_key])

    if "invert" in spec and spec["invert"]:
        data = 1.0 - data

    if "repeat" in spec:
        n = spec["repeat"]
        data = np.repeat(data, n, axis=-1)

    if "pad" in spec:
        pad_vals = np.array(spec["pad"], dtype=data.dtype)
        pad_shape = list(data.shape)
        pad_shape[-1] = len(pad_vals)
        pad_arr = np.broadcast_to(pad_vals, pad_shape)
        data = np.concatenate([data, pad_arr], axis=-1)

    return data


def validate_mapping(mapping: dict, features: dict) -> None:
    """Validate hdf5_mapping against the feature schema."""
    for lerobot_key, field_spec in mapping.items():
        if lerobot_key not in features:
            raise ValueError(
                f"hdf5_mapping key '{lerobot_key}' not found in feature schema"
            )
        if isinstance(field_spec, list):
            if "shape" not in features[lerobot_key]:
                raise ValueError(
                    f"Feature '{lerobot_key}' missing 'shape' in feature schema"
                )
            for item in field_spec:
                if isinstance(item, dict):
                    if "hdf5_key" not in item:
                        raise ValueError(
                            f"Dict entry in '{lerobot_key}' missing 'hdf5_key'"
                        )
        elif isinstance(field_spec, dict):
            required = {"hdf5_key", "encoding", "image_size"}
            missing = required - set(field_spec.keys())
            if missing:
                raise ValueError(
                    f"Image mapping for '{lerobot_key}' missing keys: {missing}"
                )
        else:
            raise TypeError(
                f"hdf5_mapping['{lerobot_key}'] must be a list or dict, "
                f"got {type(field_spec).__name__}"
            )


def initialize_dataset(
    repo_id: str, tgt_path: str, fps: int, robot_type: str, features: dict,
    image_writer_processes: int = 4, image_writer_threads: int = 4,
) -> LeRobotDataset:
    """Initialize dataset instance, removing existing data if present."""
    dataset_path = Path(tgt_path) / repo_id

    if dataset_path.exists():
        shutil.rmtree(dataset_path)
        logging.warning(f"Removed existing dataset: {dataset_path}")

    logging.info(f"Creating new dataset: {dataset_path}")
    return LeRobotDataset.create(
        repo_id=repo_id,
        root=str(dataset_path),
        fps=fps,
        robot_type=robot_type,
        features=features,
        image_writer_processes=image_writer_processes,
        image_writer_threads=image_writer_threads,
    )


def process_episode(
    episode_path: Path,
    dataset: LeRobotDataset,
    task_name: str,
    mapping: dict,
    features: dict,
) -> bool:
    """Process single episode from HDF5 into LeRobot dataset frames."""
    try:
        with h5py.File(episode_path, "r") as file:
            compose_fields = {}   # lerobot_key -> (parts_list, dtype)
            image_fields = {}     # lerobot_key -> numpy array (T, H, W, C)

            for lerobot_key, field_spec in mapping.items():
                if isinstance(field_spec, list):
                    # Numeric compose field: read and concatenate per-list-order
                    parts = [_read_hdf5_part(file, item) for item in field_spec]
                    dtype = np.dtype(features[lerobot_key]["dtype"])
                    compose_fields[lerobot_key] = (parts, dtype)
                elif isinstance(field_spec, dict):
                    # Image field
                    hdf5_key = field_spec["hdf5_key"]
                    encoding = field_spec["encoding"]
                    image_size = tuple(field_spec["image_size"])  # (W, H)
                    raw = file[hdf5_key]

                    if encoding == "jpeg":
                        images = [
                            cv2.resize(cv2.imdecode(img, cv2.IMREAD_COLOR_RGB), image_size)
                            for img in raw
                        ]
                    elif encoding == "png_depth":
                        # uint16 millimeter depth -> 3-channel uint8 video frame
                        # (Pick_up_the_apple_all 约定：复制到 3 通道，走 mp4 编码，is_depth_map=false)
                        depth_clip_mm = field_spec.get("depth_clip_mm", 8000)
                        images = []
                        for buf in raw:
                            d16 = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_UNCHANGED)
                            d16 = cv2.resize(d16, image_size, interpolation=cv2.INTER_NEAREST)
                            # 量化：[0, clip_mm] -> [0, 255]，超出 clip 的远处截断到 255
                            d8 = np.clip(d16, 0, depth_clip_mm).astype(np.float32)
                            d8 = (d8 * (255.0 / depth_clip_mm)).astype(np.uint8)
                            images.append(np.stack([d8, d8, d8], axis=-1))  # (H, W, 3)
                    elif encoding == "raw":
                        images = [
                            cv2.resize(img, image_size)
                            for img in raw
                        ]
                    else:
                        raise ValueError(
                            f"Unknown encoding '{encoding}' for '{lerobot_key}'"
                        )

                    image_fields[lerobot_key] = np.stack(images)

        num_frames = None
        for parts, _ in compose_fields.values():
            for p in parts:
                if num_frames is None:
                    num_frames = len(p)
                elif len(p) != num_frames:
                    logging.error(
                        f"Frame count mismatch in {episode_path}: "
                        f"expected {num_frames}, got {len(p)}"
                    )
                    return False

        for arr in image_fields.values():
            if num_frames is None:
                num_frames = len(arr)
            elif len(arr) != num_frames:
                logging.error(
                    f"Frame count mismatch in {episode_path}: "
                    f"expected {num_frames}, got {len(arr)} for image"
                )
                return False

    except (FileNotFoundError, OSError, KeyError) as e:
        logging.error(f"Skipped {episode_path}: {e}")
        return False

    try:
        for i in tqdm(range(num_frames), desc=f"Processing {episode_path.name}"):
            frame = {}
            for lerobot_key, (parts, dtype) in compose_fields.items():
                frame[lerobot_key] = np.concatenate(
                    [p[i] for p in parts]
                ).astype(dtype)
            for lerobot_key, arr in image_fields.items():
                frame[lerobot_key] = arr[i]
            frame["task"] = task_name
            dataset.add_frame(frame)
    except Exception as e:
        logging.error(f"Skipped {episode_path} during frame processing: {e}")
        return False

    return True


def str2bool(v: str) -> bool:
    """Parse boolean from string, replacing deprecated distutils.util.strtobool."""
    if v.lower() in ("yes", "true", "t", "1"):
        return True
    elif v.lower() in ("no", "false", "f", "0"):
        return False
    raise argparse.ArgumentTypeError(f"Boolean value expected, got '{v}'")


def main():
    parser = argparse.ArgumentParser(description="HDF5 -> LeRobot Dataset Conversion Tool")
    parser.add_argument("--config", type=str, required=True, help="Path to config JSON file")
    parser.add_argument("--repo_id", type=str, required=True, help="Dataset repository ID")
    parser.add_argument("--src_root", type=str, required=True, help="Source data directory")
    parser.add_argument("--tgt_path", type=str, required=True, help="Target output directory")
    parser.add_argument("--task_name", type=str, default="default_task", help="Task name identifier")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    parser.add_argument("--robot_type", type=str, default="tiangong", help="Robot type identifier")
    parser.add_argument("--save_one", type=str2bool, default=False, help="Save only one episode for testing")
    parser.add_argument("--hdf5_rel_path", type=str, default="trajectory.hdf5", help="Relative path to HDF5 file within each episode dir")
    parser.add_argument("--image_writer_processes", type=int, default=4, help="Number of image writer processes")
    parser.add_argument("--image_writer_threads", type=int, default=4, help="Number of image writer threads")
    args = parser.parse_args()

    # Load configuration
    features, mapping = load_config(args.config)
    validate_mapping(mapping, features)

    # Initialize dataset
    dataset = initialize_dataset(
        repo_id=args.repo_id,
        tgt_path=args.tgt_path,
        fps=args.fps,
        robot_type=args.robot_type,
        features=features,
        image_writer_processes=args.image_writer_processes,
        image_writer_threads=args.image_writer_threads,
    )

    # Process all episodes
    src_root = Path(args.src_root)
    episodes = sorted([ep for ep in src_root.iterdir() if ep.is_dir()])

    success_count = 0
    logging.info(f"Found {len(episodes)} episodes to process...")
    for ep_dir in episodes:
        ep_path = ep_dir / args.hdf5_rel_path
        if process_episode(ep_path, dataset, args.task_name, mapping, features):
            dataset.save_episode()
            success_count += 1
            logging.info(f"Saved episode: {ep_dir.name} ({success_count}/{len(episodes)})")
        else:
            dataset.clear_episode_buffer()

        if args.save_one:
            break

    dataset.finalize()
    logging.info(f"Conversion complete: {success_count}/{len(episodes)} episodes saved.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    main()
