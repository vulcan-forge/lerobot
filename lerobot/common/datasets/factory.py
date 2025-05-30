#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import logging
from pprint import pformat
import shutil

import torch

from lerobot.common.datasets.lerobot_dataset import (
    LeRobotDataset,
    LeRobotDatasetMetadata,
    MultiLeRobotDataset,
)
from lerobot.common.datasets.transforms import ImageTransforms
from lerobot.common.datasets.utils import (
    get_episode_data_index,
    write_info,
    write_jsonlines,
    serialize_dict,
)
from lerobot.configs.policies import PreTrainedConfig
from lerobot.configs.train import TrainPipelineConfig

IMAGENET_STATS = {
    "mean": [[[0.485]], [[0.456]], [[0.406]]],  # (c,1,1)
    "std": [[[0.229]], [[0.224]], [[0.225]]],  # (c,1,1)
}


def resolve_delta_timestamps(
    cfg: PreTrainedConfig, ds_meta: LeRobotDatasetMetadata
) -> dict[str, list] | None:
    """Resolves delta_timestamps by reading from the 'delta_indices' properties of the PreTrainedConfig.

    Args:
        cfg (PreTrainedConfig): The PreTrainedConfig to read delta_indices from.
        ds_meta (LeRobotDatasetMetadata): The dataset from which features and fps are used to build
            delta_timestamps against.

    Returns:
        dict[str, list] | None: A dictionary of delta_timestamps, e.g.:
            {
                "observation.state": [-0.04, -0.02, 0]
                "observation.action": [-0.02, 0, 0.02]
            }
            returns `None` if the resulting dict is empty.
    """
    delta_timestamps = {}
    for key in ds_meta.features:
        if key == "next.reward" and cfg.reward_delta_indices is not None:
            delta_timestamps[key] = [i / ds_meta.fps for i in cfg.reward_delta_indices]
        if key == "action" and cfg.action_delta_indices is not None:
            delta_timestamps[key] = [i / ds_meta.fps for i in cfg.action_delta_indices]
        if key.startswith("observation.") and cfg.observation_delta_indices is not None:
            delta_timestamps[key] = [i / ds_meta.fps for i in cfg.observation_delta_indices]

    if len(delta_timestamps) == 0:
        delta_timestamps = None

    return delta_timestamps


def make_dataset(cfg: TrainPipelineConfig) -> LeRobotDataset | MultiLeRobotDataset:
    """Handles the logic of setting up delta timestamps and image transforms before creating a dataset.

    Args:
        cfg (TrainPipelineConfig): A TrainPipelineConfig config which contains a DatasetConfig and a PreTrainedConfig.

    Raises:
        NotImplementedError: The MultiLeRobotDataset is currently deactivated.

    Returns:
        LeRobotDataset | MultiLeRobotDataset
    """
    image_transforms = (
        ImageTransforms(cfg.dataset.image_transforms) if cfg.dataset.image_transforms.enable else None
    )

    if isinstance(cfg.dataset.repo_id, str):
        ds_meta = LeRobotDatasetMetadata(
            cfg.dataset.repo_id, root=cfg.dataset.root, revision=cfg.dataset.revision
        )
        delta_timestamps = resolve_delta_timestamps(cfg.policy, ds_meta)
        dataset = LeRobotDataset(
            cfg.dataset.repo_id,
            root=cfg.dataset.root,
            episodes=cfg.dataset.episodes,
            delta_timestamps=delta_timestamps,
            image_transforms=image_transforms,
            revision=cfg.dataset.revision,
            video_backend=cfg.dataset.video_backend,
        )
    else:
        raise NotImplementedError("The MultiLeRobotDataset isn't supported for now.")
        dataset = MultiLeRobotDataset(
            cfg.dataset.repo_id,
            # TODO(aliberts): add proper support for multi dataset
            # delta_timestamps=delta_timestamps,
            image_transforms=image_transforms,
            video_backend=cfg.dataset.video_backend,
        )
        logging.info(
            "Multiple datasets were provided. Applied the following index mapping to the provided datasets: "
            f"{pformat(dataset.repo_id_to_index, indent=2)}"
        )

    if cfg.dataset.use_imagenet_stats:
        for key in dataset.meta.camera_keys:
            for stats_type, stats in IMAGENET_STATS.items():
                dataset.meta.stats[key][stats_type] = torch.tensor(stats, dtype=torch.float32)

    return dataset


def combine_datasets(datasets: list[LeRobotDataset], root: str, repo_id: str) -> LeRobotDataset:
    """
    Combines multiple LeRobotDatasets into a single dataset.
    Uses the chunk size and structure from the first dataset as the template.
    """
    if not datasets:
        raise ValueError("No datasets provided to combine")

    # Validate that all datasets have compatible configurations
    first_dataset = datasets[0]
    for dataset in datasets[1:]:
        # Check FPS compatibility
        if dataset.fps != first_dataset.fps:
            raise ValueError(f"Incompatible FPS: {dataset.fps} vs {first_dataset.fps}")

        # Check feature compatibility
        if dataset.features != first_dataset.features:
            raise ValueError("Datasets have incompatible features")

        # Check video backend compatibility
        if dataset.video_backend != first_dataset.video_backend:
            raise ValueError("Datasets have incompatible video backends")

    # Create the combined dataset using create()
    combined_dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=first_dataset.fps,
        root=root,
        robot_type=first_dataset.meta.robot_type,
        features=first_dataset.features,
        use_videos=len(first_dataset.meta.video_keys) > 0,
        video_backend=first_dataset.video_backend
    )

    # Use first dataset's chunk size as the template
    chunk_size = first_dataset.meta.chunks_size
    total_episodes = sum(dataset.meta.total_episodes for dataset in datasets)
    total_chunks = (total_episodes + chunk_size - 1) // chunk_size

    # Update metadata with correct total episodes and chunks
    combined_dataset.meta.info["total_episodes"] = total_episodes
    combined_dataset.meta.info["total_chunks"] = total_chunks
    combined_dataset.meta.info["chunks_size"] = chunk_size

    # Create meta directory
    meta_dir = combined_dataset.meta.root / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    # Create data directory
    data_dir = combined_dataset.meta.root / "data"
    data_dir.mkdir(parents=True, exist_ok=True)

    # Combine episodes and stats
    episode_offset = 0
    for dataset in datasets:
        logging.info(f"Processing dataset with root: {dataset.meta.root}")
        # Update episode indices
        for ep_idx in dataset.meta.episodes:
            new_ep_idx = ep_idx + episode_offset
            combined_dataset.meta.episodes[new_ep_idx] = dataset.meta.episodes[ep_idx]
            combined_dataset.meta.episodes_stats[new_ep_idx] = dataset.meta.episodes_stats[ep_idx]

            # Calculate new chunk and episode numbers
            new_chunk = new_ep_idx // chunk_size
            new_ep_in_chunk = new_ep_idx % chunk_size

            # Copy data files with new sequential naming
            src_data_path = dataset.meta.root / dataset.meta.get_data_file_path(ep_idx)
            if src_data_path.exists():
                dst_data_path = combined_dataset.meta.root / f"data/chunk-{new_chunk:03d}/episode_{new_ep_idx:06d}.parquet"
                dst_data_path.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src_data_path, dst_data_path)

            # Copy video files if they exist
            for vid_key in dataset.meta.video_keys:
                src_video_path = dataset.meta.root / dataset.meta.get_video_file_path(ep_idx, vid_key)
                if src_video_path.exists():
                    dst_video_path = combined_dataset.meta.root / f"videos/chunk-{new_chunk:03d}/{vid_key}/episode_{new_ep_idx:06d}.mp4"
                    dst_video_path.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(src_video_path, dst_video_path)

        episode_offset += dataset.meta.total_episodes

    # Save all metadata files
    write_info(combined_dataset.meta.info, combined_dataset.meta.root)

    # Save tasks.jsonl
    tasks_data = [{"task_index": idx, "task": task} for idx, task in combined_dataset.meta.tasks.items()]
    write_jsonlines(tasks_data, meta_dir / "tasks.jsonl")

    # Save episodes.jsonl
    episodes_data = list(combined_dataset.meta.episodes.values())
    write_jsonlines(episodes_data, meta_dir / "episodes.jsonl")

    # Save episodes_stats.jsonl
    episodes_stats_data = [
        {"episode_index": idx, "stats": serialize_dict(stats)}
        for idx, stats in combined_dataset.meta.episodes_stats.items()
    ]
    write_jsonlines(episodes_stats_data, meta_dir / "episodes_stats.jsonl")

    # Verify that data files exist before loading
    if not data_dir.exists():
        raise ValueError(f"Data directory {data_dir} does not exist")

    parquet_files = list(data_dir.rglob("*.parquet"))
    if not parquet_files:
        raise ValueError(f"No parquet files found in {data_dir}")

    # Load the combined dataset
    combined_dataset.hf_dataset = combined_dataset.load_hf_dataset()
    combined_dataset.episode_data_index = get_episode_data_index(
        combined_dataset.meta.episodes,
        list(combined_dataset.meta.episodes.keys())
    )

    return combined_dataset
