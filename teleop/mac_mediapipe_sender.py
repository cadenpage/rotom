#!/usr/bin/env python3
"""Phase-one MediaPipe teleop sender for the Mac webcam.

This script tracks one hand, uses a pinch gesture as a clutch, and sends a
relative hand pose stream over UDP to the Jetson. The robot-side ROS node maps
that hand motion into `/rotom_servo/cartesian_cmd`.
"""

import argparse
import json
import math
import socket
import time

import cv2
import mediapipe as mp


def _mean_landmark(landmarks, indices):
    count = float(len(indices))
    return (
        sum(landmarks[i].x for i in indices) / count,
        sum(landmarks[i].y for i in indices) / count,
        sum(landmarks[i].z for i in indices) / count,
    )


def _distance_2d(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _palm_yaw(landmarks):
    index_mcp = landmarks[5]
    pinky_mcp = landmarks[17]
    return math.atan2(index_mcp.y - pinky_mcp.y, index_mcp.x - pinky_mcp.x)


def parse_args():
    parser = argparse.ArgumentParser(description="Send MediaPipe hand teleop packets over UDP.")
    parser.add_argument("--host", default="127.0.0.1", help="Jetson IP or hostname")
    parser.add_argument("--port", type=int, default=8765, help="Jetson UDP port")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--send-rate", type=float, default=25.0, help="Packet send rate in Hz")
    parser.add_argument("--pinch-threshold", type=float, default=0.045, help="Thumb-index pinch threshold")
    parser.add_argument("--min-detection-confidence", type=float, default=0.6)
    parser.add_argument("--min-tracking-confidence", type=float, default=0.6)
    parser.add_argument("--no-display", action="store_true", help="Disable the preview window")
    return parser.parse_args()


def main():
    args = parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    hands = mp.solutions.hands.Hands(
        max_num_hands=1,
        min_detection_confidence=args.min_detection_confidence,
        min_tracking_confidence=args.min_tracking_confidence,
    )
    drawing = mp.solutions.drawing_utils

    last_send_time = 0.0
    latest_packet = {
        "seq": 0,
        "stamp_ns": time.monotonic_ns(),
        "tracked": False,
        "clutch": False,
        "confidence": 0.0,
        "hand_xyz": [0.0, 0.0, 0.0],
        "yaw": 0.0,
    }

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Failed to read frame from webcam")

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(rgb)

            tracked = False
            clutch = False
            confidence = 0.0
            hand_xyz = latest_packet["hand_xyz"]
            yaw = latest_packet["yaw"]

            if result.multi_hand_landmarks:
                landmarks = result.multi_hand_landmarks[0].landmark
                palm_center = _mean_landmark(landmarks, [0, 5, 9, 17])
                pinch_distance = _distance_2d(landmarks[4], landmarks[8])

                tracked = True
                clutch = pinch_distance < args.pinch_threshold
                confidence = 1.0
                hand_xyz = [float(palm_center[0]), float(palm_center[1]), float(palm_center[2])]
                yaw = float(_palm_yaw(landmarks))

                if not args.no_display:
                    drawing.draw_landmarks(frame, result.multi_hand_landmarks[0], mp.solutions.hands.HAND_CONNECTIONS)
                    cv2.putText(
                        frame,
                        f"pinch={pinch_distance:.3f} clutch={'ON' if clutch else 'OFF'}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0) if clutch else (0, 200, 255),
                        2,
                    )
            elif not args.no_display:
                cv2.putText(
                    frame,
                    "No hand detected",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

            now = time.monotonic()
            if now - last_send_time >= 1.0 / max(args.send_rate, 1e-3):
                latest_packet = {
                    "seq": seq,
                    "stamp_ns": time.monotonic_ns(),
                    "tracked": tracked,
                    "clutch": clutch,
                    "confidence": confidence,
                    "hand_xyz": hand_xyz,
                    "yaw": yaw,
                }
                sock.sendto(json.dumps(latest_packet).encode("utf-8"), (args.host, args.port))
                seq += 1
                last_send_time = now

            if not args.no_display:
                cv2.putText(
                    frame,
                    f"UDP -> {args.host}:{args.port} seq={latest_packet['seq']}",
                    (20, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )
                cv2.imshow("Rotom MediaPipe Teleop", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        hands.close()
        cap.release()
        sock.close()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
