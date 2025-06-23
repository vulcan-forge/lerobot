#!/usr/bin/env python

import cv2
import numpy as np
from typing import List, Dict, Any, Optional
import time


def fourcc_to_string(fourcc: int) -> str:
    """Convert FOURCC integer to string representation."""
    return "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])


def string_to_fourcc(fourcc_str: str) -> int:
    """Convert string to FOURCC integer."""
    return cv2.VideoWriter_fourcc(*fourcc_str)


def get_camera_info(camera_index: int = 0) -> Dict[str, Any]:
    """Get current camera information and capabilities."""
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera at index {camera_index}")

    info = {
        "camera_index": camera_index,
        "backend": cap.getBackendName(),
        "current_format": fourcc_to_string(int(cap.get(cv2.CAP_PROP_FOURCC))),
        "current_width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "current_height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "current_fps": cap.get(cv2.CAP_PROP_FPS),
        "format_property": cap.get(cv2.CAP_PROP_FORMAT),
    }

    cap.release()
    return info


def test_camera_format(
    camera_index: int = 0,
    target_format: str = "MJPG",
    width: int = 1280,
    height: int = 720,
    fps: int = 30,
    test_duration: float = 2.0
) -> Dict[str, Any]:
    """
    Test setting a specific camera format and capture frames.

    Args:
        camera_index: Camera device index
        target_format: FOURCC format string (e.g., "MJPG", "YUYV", "RGB3")
        width: Target width
        height: Target height
        fps: Target FPS
        test_duration: How long to capture frames (seconds)

    Returns:
        Dictionary with test results
    """
    print(f"Testing camera {camera_index} with format {target_format}")
    print(f"Target: {width}x{height} @ {fps}fps")

    # Open camera
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera at index {camera_index}")

    # Get initial state
    initial_format = fourcc_to_string(int(cap.get(cv2.CAP_PROP_FOURCC)))
    print(f"Initial format: {initial_format}")

    # Set format
    fourcc = string_to_fourcc(target_format)
    format_success = cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    # Set other properties
    width_success = cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    height_success = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    fps_success = cap.set(cv2.CAP_PROP_FPS, fps)

    # Get actual values after setting
    actual_format = fourcc_to_string(int(cap.get(cv2.CAP_PROP_FOURCC)))
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"Format set success: {format_success}")
    print(f"Actual format: {actual_format}")
    print(f"Actual resolution: {actual_width}x{actual_height}")
    print(f"Actual FPS: {actual_fps}")

    # Test frame capture
    frames_captured = 0
    start_time = time.time()
    frame_times = []

    try:
        while time.time() - start_time < test_duration:
            frame_start = time.time()
            ret, frame = cap.read()
            frame_end = time.time()

            if ret:
                frames_captured += 1
                frame_times.append(frame_end - frame_start)

                # Print first frame info
                if frames_captured == 1:
                    print(f"First frame shape: {frame.shape}")
                    print(f"First frame dtype: {frame.dtype}")
            else:
                print("Failed to read frame")
                break

    except KeyboardInterrupt:
        print("Test interrupted by user")

    cap.release()

    # Calculate statistics
    total_time = time.time() - start_time
    actual_fps_measured = frames_captured / total_time if total_time > 0 else 0
    avg_frame_time = np.mean(frame_times) if frame_times else 0

    results = {
        "target_format": target_format,
        "initial_format": initial_format,
        "actual_format": actual_format,
        "format_set_success": format_success,
        "target_resolution": (width, height),
        "actual_resolution": (actual_width, actual_height),
        "target_fps": fps,
        "actual_fps": actual_fps,
        "measured_fps": actual_fps_measured,
        "frames_captured": frames_captured,
        "test_duration": total_time,
        "avg_frame_time_ms": avg_frame_time * 1000,
        "width_set_success": width_success,
        "height_set_success": height_success,
        "fps_set_success": fps_success,
    }

    print(f"\nTest Results:")
    print(f"  Frames captured: {frames_captured}")
    print(f"  Measured FPS: {actual_fps_measured:.1f}")
    print(f"  Avg frame time: {avg_frame_time*1000:.1f}ms")
    print(f"  Format changed: {initial_format} -> {actual_format}")

    return results


def test_multiple_formats(
    camera_index: int = 0,
    formats: List[str] = ["YUYV", "MJPG", "RGB3", "BGR3"],
    width: int = 1280,
    height: int = 720,
    fps: int = 30,
    test_duration: float = 1.0
) -> Dict[str, Dict[str, Any]]:
    """
    Test multiple camera formats and compare results.

    Args:
        camera_index: Camera device index
        formats: List of FOURCC format strings to test
        width: Target width
        height: Target height
        fps: Target FPS
        test_duration: Test duration per format (seconds)

    Returns:
        Dictionary with results for each format
    """
    print(f"Testing multiple formats on camera {camera_index}")
    print(f"Formats to test: {formats}")
    print(f"Target: {width}x{height} @ {fps}fps")
    print("=" * 50)

    results = {}

    for format_name in formats:
        try:
            print(f"\nTesting format: {format_name}")
            result = test_camera_format(
                camera_index=camera_index,
                target_format=format_name,
                width=width,
                height=height,
                fps=fps,
                test_duration=test_duration
            )
            results[format_name] = result
        except Exception as e:
            print(f"Error testing {format_name}: {e}")
            results[format_name] = {"error": str(e)}

    # Print summary
    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)

    for format_name, result in results.items():
        if "error" in result:
            print(f"{format_name}: ERROR - {result['error']}")
        else:
            print(f"{format_name}: {result['measured_fps']:.1f} FPS, "
                  f"{result['avg_frame_time_ms']:.1f}ms avg frame time")

    return results


def list_available_cameras() -> List[Dict[str, Any]]:
    """List all available cameras and their current formats."""
    cameras = []
    max_index = 10  # Check first 10 indices

    for i in range(max_index):
        try:
            info = get_camera_info(i)
            cameras.append(info)
            print(f"Camera {i}: {info['current_format']} @ "
                  f"{info['current_width']}x{info['current_height']} "
                  f"({info['current_fps']:.1f} FPS)")
        except RuntimeError:
            continue

    return cameras


if __name__ == "__main__":
    # Example usage
    print("Camera Format Testing Tool")
    print("=" * 30)

    # List available cameras
    print("\nAvailable cameras:")
    cameras = list_available_cameras()

    if not cameras:
        print("No cameras found!")
        exit(1)

    # Test single format
    print(f"\nTesting MJPG format on camera 0:")
    try:
        result = test_camera_format(
            camera_index=0,
            target_format="MJPG",
            width=1280,
            height=720,
            fps=30,
            test_duration=2.0
        )
    except Exception as e:
        print(f"Error: {e}")

    # Test multiple formats
    print(f"\nTesting multiple formats on camera 0:")
    try:
        results = test_multiple_formats(
            camera_index=0,
            formats=["YUYV", "MJPG", "RGB3"],
            width=1280,
            height=720,
            fps=30,
            test_duration=1.0
        )
    except Exception as e:
        print(f"Error: {e}")
