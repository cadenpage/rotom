#!/usr/bin/env python3
"""Interactive ChArUco camera calibration for Rotom.

Board (must match your printed target):
  squaresX=11, squaresY=8, DICT_5X5_100
  checker_size=20 mm, marker_size=15 mm

Prerequisites:
  Terminal 1:  pixi run ros2-vision-camera
  Terminal 2:  pixi run calibrate-camera

Controls:
  SPACE  — capture the current frame (aim for 20+ frames)
  d      — discard the most recently captured frame
  c      — run calibration with captured frames and save YAML
  q      — quit (without saving if you haven't pressed c)

Output:
  src/ros2_ws/src/rotom_vision/config/camera_calibration.yaml

After calibrating, set calibration_file in vision_pipeline.yaml to
that path so the stereo_splitter publishes real intrinsics.
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
# Make sure rclpy / cv_bridge are importable when run via pixi task
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


def _detect(frame: np.ndarray, aruco_dict, board, _params):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # OpenCV 4.7+ CharucoDetector
    detector = cv2.aruco.CharucoDetector(board)
    ch_corners, ch_ids, corners, ids = detector.detectBoard(gray)

    display = frame.copy()
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(display, corners, ids)
    if ch_corners is not None and len(ch_corners) >= 4:
        cv2.aruco.drawDetectedCornersCharuco(display, ch_corners, ch_ids)
        return display, ch_corners, ch_ids
    return display, None, None


# ---------------------------------------------------------------------------
# YAML output  (ROS camera_calibration compatible)
# ---------------------------------------------------------------------------

def _save_yaml(path: str, K: np.ndarray, D: np.ndarray, size: Tuple[int, int]) -> None:
    w, h = size
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    d = [float(v) for v in D.ravel()]

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(f"# ChArUco calibration — {datetime.datetime.now().isoformat()}\n")
        f.write(
            f"# Board: {SQUARES_X}x{SQUARES_Y} squares, "
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

    _save_yaml(output_path, K, D, image_size)
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

    aruco_dict, board = _make_board()
    detector_params = cv2.aruco.DetectorParameters()  # OpenCV 4.7+ API

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

        display, corners, ids = _detect(frame, aruco_dict, board, detector_params)

        # Status bar
        n = len(all_corners)
        board_ok = corners is not None
        bar_color = (0, 200, 0) if board_ok else (0, 80, 200)
        board_txt = f"Board: {'OK  ' if board_ok else 'LOST'}  |  Captured: {n}"
        cv2.putText(display, board_txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, bar_color, 2)
        cv2.putText(display, "SPC=capture  d=discard  c=calibrate  q=quit",
                    (10, display.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

        # Flash message
        if time.time() < flash_until:
            cv2.putText(display, flash_msg, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("ChArUco Calibration", display)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' '):
            if not board_ok:
                flash_msg = "Board not detected — move it into view"
                flash_until = time.time() + 2.0
            else:
                all_corners.append(corners)
                all_ids.append(ids)
                flash_msg = f"Captured! ({len(all_corners)} frames)"
                flash_until = time.time() + 1.5
                print(f"  Captured frame {len(all_corners)}")

        elif key == ord('d'):
            if all_corners:
                all_corners.pop()
                all_ids.pop()
                flash_msg = f"Discarded last ({len(all_corners)} remain)"
                flash_until = time.time() + 1.5
                print(f"  Discarded — {len(all_corners)} frames remain")
            else:
                flash_msg = "Nothing to discard"
                flash_until = time.time() + 1.0

        elif key == ord('c'):
            if image_size is not None:
                success = _calibrate(all_corners, all_ids, board, image_size, args.output)
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
