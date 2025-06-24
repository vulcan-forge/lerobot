#!/usr/bin/env python

"""
Camera detection utility for LeRobot.

This module provides functions to find and list available cameras on the system,
including OpenCV-compatible cameras and Intel RealSense cameras.
"""

import logging
import sys
from typing import Any, Dict, List, Optional

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def find_cameras(camera_type: Optional[str] = None, verbose: bool = True) -> List[Dict[str, Any]]:
    """
    Find available cameras on the system.

    This function detects both OpenCV-compatible cameras and Intel RealSense cameras.
    It leverages the existing camera detection infrastructure from the LeRobot project.

    Args:
        camera_type: Optional filter for camera type. Can be "opencv", "realsense", or None for all.
        verbose: If True, prints camera information to console. Defaults to True.

    Returns:
        List of dictionaries containing camera information. Each dictionary includes:
        - name: Human-readable camera name
        - type: Camera type ("OpenCV" or "RealSense")
        - id: Camera identifier (index/path for OpenCV, serial number for RealSense)
        - backend_api: Backend API name (for OpenCV cameras)
        - default_stream_profile: Dictionary with format, width, height, fps

    Example:
        # Find all cameras
        cameras = find_cameras()

        # Find only OpenCV cameras
        opencv_cameras = find_cameras(camera_type="opencv")

        # Find only RealSense cameras
        realsense_cameras = find_cameras(camera_type="realsense")

        # Find cameras without printing to console
        cameras = find_cameras(verbose=False)
    """
    all_cameras_info: List[Dict[str, Any]] = []

    # Normalize camera type filter
    if camera_type:
        camera_type = camera_type.lower()

    # Find OpenCV cameras
    if camera_type is None or camera_type == "opencv":
        try:
            from lerobot.common.cameras.opencv.camera_opencv import OpenCVCamera
            opencv_cameras = OpenCVCamera.find_cameras()
            all_cameras_info.extend(opencv_cameras)
            if verbose:
                logger.info(f"Found {len(opencv_cameras)} OpenCV camera(s)")
        except ImportError:
            if verbose:
                logger.warning("OpenCV not available - skipping OpenCV camera detection")
        except Exception as e:
            if verbose:
                logger.error(f"Error detecting OpenCV cameras: {e}")

    # Find RealSense cameras
    if camera_type is None or camera_type == "realsense":
        try:
            from lerobot.common.cameras.realsense.camera_realsense import RealSenseCamera
            realsense_cameras = RealSenseCamera.find_cameras()
            all_cameras_info.extend(realsense_cameras)
            if verbose:
                logger.info(f"Found {len(realsense_cameras)} RealSense camera(s)")
        except ImportError:
            if verbose:
                logger.warning("pyrealsense2 not available - skipping RealSense camera detection")
        except Exception as e:
            if verbose:
                logger.error(f"Error detecting RealSense cameras: {e}")

    # Print results if verbose
    if verbose:
        if not all_cameras_info:
            if camera_type:
                logger.warning(f"No {camera_type} cameras detected")
            else:
                logger.warning("No cameras detected")
        else:
            print("\n" + "="*50)
            print("DETECTED CAMERAS")
            print("="*50)

            for i, cam_info in enumerate(all_cameras_info):
                print(f"\nCamera #{i+1}:")
                print(f"  Name: {cam_info.get('name', 'Unknown')}")
                print(f"  Type: {cam_info.get('type', 'Unknown')}")
                print(f"  ID: {cam_info.get('id', 'Unknown')}")

                if 'backend_api' in cam_info:
                    print(f"  Backend API: {cam_info['backend_api']}")

                if 'default_stream_profile' in cam_info:
                    profile = cam_info['default_stream_profile']
                    print(f"  Default Profile:")
                    print(f"    Resolution: {profile.get('width', 'Unknown')}x{profile.get('height', 'Unknown')}")
                    print(f"    FPS: {profile.get('fps', 'Unknown')}")
                    print(f"    Format: {profile.get('format', 'Unknown')}")

                print("-" * 30)

    return all_cameras_info


def find_camera_by_id(camera_id: str, camera_type: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """
    Find a specific camera by its ID.

    Args:
        camera_id: The camera identifier to search for
        camera_type: Optional camera type filter ("opencv", "realsense", or None)

    Returns:
        Camera information dictionary if found, None otherwise
    """
    cameras = find_cameras(camera_type=camera_type, verbose=False)

    for camera in cameras:
        if str(camera.get('id')) == str(camera_id):
            return camera

    return None


def list_camera_types() -> List[str]:
    """
    List available camera types that can be detected.

    Returns:
        List of available camera type strings
    """
    available_types = []

    try:
        from lerobot.common.cameras.opencv.camera_opencv import OpenCVCamera
        available_types.append("opencv")
    except ImportError:
        pass

    try:
        from lerobot.common.cameras.realsense.camera_realsense import RealSenseCamera
        available_types.append("realsense")
    except ImportError:
        pass

    return available_types


def get_camera_summary() -> Dict[str, Any]:
    """
    Get a summary of all detected cameras.

    Returns:
        Dictionary with summary information including:
        - total_cameras: Total number of cameras found
        - camera_types: Dictionary with counts by type
        - available_types: List of camera types that can be detected
    """
    cameras = find_cameras(verbose=False)
    available_types = list_camera_types()

    # Count cameras by type
    camera_types = {}
    for camera in cameras:
        cam_type = camera.get('type', 'Unknown').lower()
        camera_types[cam_type] = camera_types.get(cam_type, 0) + 1

    return {
        'total_cameras': len(cameras),
        'camera_types': camera_types,
        'available_types': available_types
    }


if __name__ == "__main__":
    # Command line interface
    import argparse

    parser = argparse.ArgumentParser(description="Find available cameras on the system")
    parser.add_argument(
        "--type",
        choices=["opencv", "realsense"],
        help="Filter by camera type"
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        help="Show summary only"
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress verbose output"
    )

    args = parser.parse_args()

    if args.summary:
        summary = get_camera_summary()
        print("\nCamera Detection Summary:")
        print(f"Total cameras: {summary['total_cameras']}")
        print(f"Available types: {', '.join(summary['available_types'])}")
        print("By type:")
        for cam_type, count in summary['camera_types'].items():
            print(f"  {cam_type}: {count}")
    else:
        find_cameras(camera_type=args.type, verbose=not args.quiet)
