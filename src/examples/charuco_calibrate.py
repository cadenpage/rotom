#!/usr/bin/env python3
"""Interactive ChArUco camera calibration for Rotom.

Board (must match your printed target):
  squaresX=11, squaresY=8, DICT_5X5_100
  checker_size=20 mm, marker_size=15 mm

Prerequisites:
  Terminal 1:  just ros2-vision-camera
  Terminal 2:  just charuco-calibrate

Controls:
  SPACE  — capture the current frame (aim for 20+ frames)
  d      — discard the most recently captured frame
  c      — run calibration with captured frames and save YAML
  q      — quit (without saving if you haven't pressed c)

Output:
  src/ros2_ws/src/rotom_vision/config/camera_calibration.yaml

After calibrating, rebuild rotom_vision once so the saved YAML is installed.
The vision launch will then auto-load config/camera_calibration.yaml.
"""

import argparse
import datetime
import os
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

_HERE = Path(__file__).resolve().parent
# Make sure rclpy / cv_bridge are importable when run from the repo's ROS environment
_INSTALL = _HERE.parent / "ros2_ws" / "install"

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
except ImportError:
    print("rclpy not found — source ros2_ws/install/setup.bash first.")
    sys.exit(1)


def _imgmsg_to_bgr(msg: "Image") -> np.ndarray:
    """Convert a sensor_msgs/Image to a BGR uint8 numpy array without cv_bridge."""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("bgr8", "rgb8", "bgra8", "rgba8"):
        channels = 4 if enc in ("bgra8", "rgba8") else 3
        frame = data.reshape(msg.height, msg.width, channels)
        if enc in ("rgb8", "rgba8"):
            frame = frame[:, :, [2, 1, 0]]  # RGB→BGR
        return np.ascontiguousarray(frame[:, :, :3])
    elif enc in ("mono8",):
        gray = data.reshape(msg.height, msg.width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    elif enc == "yuyv":
        yuv = data.reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
    else:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

# ---------------------------------------------------------------------------
# Board configuration
# ---------------------------------------------------------------------------
SQUARES_X = 11
SQUARES_Y = 8
SQUARE_LEN_M = 0.020   # 20 mm
MARKER_LEN_M = 0.015   # 15 mm
ARUCO_DICT_ID = cv2.aruco.DICT_5X5_100
MIN_FRAMES = 15
MIN_CHARUCO_CORNERS = 4

DEFAULT_TOPIC = "/camera/selected/image_raw"
DEFAULT_OUTPUT = str(
    _HERE.parent / "ros2_ws" / "src" / "rotom_vision" / "config" / "camera_calibration.yaml"
)


# ---------------------------------------------------------------------------
# ROS subscriber node
# ---------------------------------------------------------------------------

class _ImageSub(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("charuco_calibrator")
        self._latest: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self.create_subscription(Image, topic, self._cb, 10)
        self.get_logger().info(f"Listening on {topic}")

    def _cb(self, msg: Image) -> None:
        try:
            frame = _imgmsg_to_bgr(msg)
        except Exception:
            return
        with self._lock:
            self._latest = frame

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._latest is None else self._latest.copy()


# ---------------------------------------------------------------------------
# ChArUco helpers
# ---------------------------------------------------------------------------

def _make_board():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    # OpenCV 4.7+ API: CharucoBoard((cols, rows), square_len, marker_len, dict)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_LEN_M, MARKER_LEN_M, aruco_dict
    )
    return aruco_dict, board


def _make_board_variants():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    variants = []
    for cols, rows in ((SQUARES_X, SQUARES_Y), (SQUARES_Y, SQUARES_X)):
        for legacy in (False, True):
            board = cv2.aruco.CharucoBoard((cols, rows), SQUARE_LEN_M, MARKER_LEN_M, aruco_dict)
            if legacy and hasattr(board, "setLegacyPattern"):
                board.setLegacyPattern(True)
            label = f"{cols}x{rows}" + (" legacy" if legacy else "")
            variants.append({
                "label": label,
                "cols": cols,
                "rows": rows,
                "legacy": legacy,
                "board": board,
            })
    return aruco_dict, variants


def _detect(frame: np.ndarray, aruco_dict, board_variants, _params):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    display = frame.copy()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, _params)
        marker_corners, marker_ids, _rejected = detector.detectMarkers(gray)
    else:
        marker_corners, marker_ids, _rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=_params)

    marker_count = 0 if marker_ids is None else len(marker_ids)
    if marker_count > 0:
        cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)

    best_variant = None
    best_corners = None
    best_ids = None
    best_count = 0

    for variant in board_variants:
        board = variant["board"]
        variant_corners = None
        variant_ids = None

        if marker_count > 0:
            try:
                _count, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, gray, board
                )
                if ch_ids is not None and len(ch_ids) > 0:
                    variant_corners, variant_ids = ch_corners, ch_ids
            except Exception:
                pass

        if hasattr(cv2.aruco, "CharucoDetector"):
            try:
                ch_detector = cv2.aruco.CharucoDetector(board)
                ch_corners, ch_ids, _marker_corners, _marker_ids = ch_detector.detectBoard(gray)
                if ch_ids is not None and len(ch_ids) > (0 if variant_ids is None else len(variant_ids)):
                    variant_corners, variant_ids = ch_corners, ch_ids
            except Exception:
                pass

        variant_count = 0 if variant_ids is None else len(variant_ids)
        if variant_count > best_count:
            best_variant = variant
            best_corners = variant_corners
            best_ids = variant_ids
            best_count = variant_count

    charuco_count = best_count
    if charuco_count >= MIN_CHARUCO_CORNERS:
        cv2.aruco.drawDetectedCornersCharuco(display, best_corners, best_ids)
        return display, best_corners, best_ids, marker_count, charuco_count, best_variant
    return display, None, None, marker_count, charuco_count, best_variant


# ---------------------------------------------------------------------------
# YAML output  (ROS camera_calibration compatible)
# ---------------------------------------------------------------------------

def _save_yaml(path: str, K: np.ndarray, D: np.ndarray, size: Tuple[int, int], board_label: str) -> None:
    w, h = size
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    d = [float(v) for v in D.ravel()]

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(f"# ChArUco calibration — {datetime.datetime.now().isoformat()}\n")
        f.write(
            f"# Board: {board_label}, "
            f"checker={SQUARE_LEN_M * 1000:.0f}mm, marker={MARKER_LEN_M * 1000:.0f}mm\n"
        )
        f.write(f"image_width: {w}\n")
        f.write(f"image_height: {h}\n")
        f.write(f"camera_name: rotom_camera\n")
        f.write(f"camera_matrix:\n  rows: 3\n  cols: 3\n")
        f.write(f"  data: [{fx:.6f}, 0.0, {cx:.6f}, 0.0, {fy:.6f}, {cy:.6f}, 0.0, 0.0, 1.0]\n")
        f.write(f"distortion_model: plumb_bob\n")
        f.write(f"distortion_coefficients:\n  rows: 1\n  cols: {len(d)}\n")
        f.write(f"  data: [{', '.join(f'{v:.8f}' for v in d)}]\n")
        f.write(f"rectification_matrix:\n  rows: 3\n  cols: 3\n")
        f.write(f"  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n")
        f.write(f"projection_matrix:\n  rows: 3\n  cols: 4\n")
        f.write(
            f"  data: [{fx:.6f}, 0.0, {cx:.6f}, 0.0, "
            f"0.0, {fy:.6f}, {cy:.6f}, 0.0, "
            f"0.0, 0.0, 1.0, 0.0]\n"
        )
    print(f"\nSaved to {path}")


# ---------------------------------------------------------------------------
# Calibration solver
# ---------------------------------------------------------------------------

def _calibrate(
    all_corners: List,
    all_ids: List,
    board,
    image_size: Tuple[int, int],
    output_path: str,
    board_label: str,
) -> bool:
    if len(all_corners) < MIN_FRAMES:
        print(f"Need at least {MIN_FRAMES} frames, only have {len(all_corners)}.")
        return False

    print(f"\nRunning calibration on {len(all_corners)} frames …")
    rms, K, D, _rvecs, _tvecs = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, image_size, None, None
    )

    print(f"RMS reprojection error: {rms:.4f} px")
    if rms > 1.5:
        print("  WARNING: RMS > 1.5 — consider recapturing with better coverage.")
    print(f"fx={K[0,0]:.2f}  fy={K[1,1]:.2f}  cx={K[0,2]:.2f}  cy={K[1,2]:.2f}")
    print(f"distortion: {D.ravel()}")

    _save_yaml(output_path, K, D, image_size, board_label)
    return True


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Image topic to subscribe to")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Output YAML path")
    args = parser.parse_args()

    rclpy.init()
    node = _ImageSub(args.topic)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    aruco_dict, board_variants = _make_board_variants()
    detector_params = cv2.aruco.DetectorParameters()  # OpenCV 4.7+ API
    expected_markers = len(board_variants[0]["board"].getIds()) if hasattr(board_variants[0]["board"], "getIds") else (SQUARES_X * SQUARES_Y) // 2
    active_variant = None

    all_corners: List = []
    all_ids: List = []
    image_size: Optional[Tuple[int, int]] = None

    cv2.namedWindow("ChArUco Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ChArUco Calibration", 800, 600)

    print("\n=== ChArUco Calibration ===")
    print(f"  Board: {SQUARES_X}x{SQUARES_Y} squares, checker={SQUARE_LEN_M*1000:.0f}mm, marker={MARKER_LEN_M*1000:.0f}mm")
    print(f"  Topic: {args.topic}")
    print(f"  Output: {args.output}")
    print("\nSPACE  — capture frame")
    print("d      — discard last frame")
    print("c      — calibrate & save")
    print("q      — quit\n")

    flash_msg = ""
    flash_until = 0.0

    while rclpy.ok():
        frame = node.get_frame()
        if frame is None:
            # Show waiting screen
            blank = np.zeros((300, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for image…", (20, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
            cv2.imshow("ChArUco Calibration", blank)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break
            continue

        if image_size is None:
            h, w = frame.shape[:2]
            image_size = (w, h)

        active_variants = board_variants if active_variant is None else [active_variant]
        display, corners, ids, marker_count, charuco_count, best_variant = _detect(
            frame, aruco_dict, active_variants, detector_params
        )
        variant_label = "auto" if best_variant is None else best_variant["label"]
        if active_variant is not None:
            variant_label = active_variant["label"]

        # Status bar
        n = len(all_corners)
        board_ok = corners is not None
        bar_color = (0, 200, 0) if board_ok else (0, 80, 200)
        board_txt = f"Board: {'OK  ' if board_ok else 'LOST'}  |  Captured: {n}"
        cv2.putText(display, board_txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, bar_color, 2)
        detail_txt = (
            f"Markers: {marker_count}/{expected_markers}  |  "
            f"Variant: {variant_label}  |  "
            f"ChArUco corners: {charuco_count}"
        )
        cv2.putText(display, detail_txt, (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 2)
        cv2.putText(display, "SPC=capture  d=discard  c=calibrate  q=quit",
                    (10, display.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

        # Flash message
        if time.time() < flash_until:
            cv2.putText(display, flash_msg, (10, 82),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("ChArUco Calibration", display)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' '):
            if not board_ok:
                if marker_count > 0:
                    flash_msg = (
                        f"Markers detected ({marker_count}), but only {charuco_count} corners. "
                        "Try flatter lighting, better focus, or a different board variant."
                    )
                else:
                    flash_msg = "Board not detected — move it into view"
                flash_until = time.time() + 2.0
            else:
                if active_variant is None and best_variant is not None:
                    active_variant = best_variant
                all_corners.append(corners)
                all_ids.append(ids)
                flash_msg = f"Captured! ({len(all_corners)} frames) using {active_variant['label']}"
                flash_until = time.time() + 1.5
                print(f"  Captured frame {len(all_corners)}")

        elif key == ord('d'):
            if all_corners:
                all_corners.pop()
                all_ids.pop()
                if not all_corners:
                    active_variant = None
                flash_msg = f"Discarded last ({len(all_corners)} remain)"
                flash_until = time.time() + 1.5
                print(f"  Discarded — {len(all_corners)} frames remain")
            else:
                flash_msg = "Nothing to discard"
                flash_until = time.time() + 1.0

        elif key == ord('c'):
            if image_size is not None:
                calibration_variant = active_variant if active_variant is not None else board_variants[0]
                success = _calibrate(
                    all_corners,
                    all_ids,
                    calibration_variant["board"],
                    image_size,
                    args.output,
                    calibration_variant["label"],
                )
                if success:
                    flash_msg = "Calibration saved!"
                    flash_until = time.time() + 3.0
                else:
                    flash_msg = f"Need {MIN_FRAMES} frames (have {len(all_corners)})"
                    flash_until = time.time() + 2.0

    cv2.destroyAllWindows()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
