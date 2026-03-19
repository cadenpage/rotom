"""Nudge the current task-space pitch target by a small delta."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class TaskPitchStepClient(Node):
    def __init__(self, delta_pitch: float, timeout_s: float) -> None:
        super().__init__("task_pitch_step_client")
        self.delta_pitch = delta_pitch
        self.timeout_s = timeout_s
        self.current_target: float | None = None

        self.target_sub = self.create_subscription(Float64, "/rotom_task/target_pitch", self._target_cb, 10)
        self.target_pub = self.create_publisher(Float64, "/rotom_task/pitch_target", 10)

    def _target_cb(self, msg: Float64) -> None:
        self.current_target = float(msg.data)

    def run(self) -> int:
        deadline = self.get_clock().now().nanoseconds + int(self.timeout_s * 1e9)
        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_target is not None:
                break

        if self.current_target is None:
            self.get_logger().error("No /rotom_task/target_pitch received; is task-core running?")
            return 1

        msg = Float64()
        msg.data = self.current_target + self.delta_pitch
        for _ in range(3):
            self.target_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(f"task pitch step sent: {self.current_target:.3f} -> {msg.data:.3f} rad")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("delta_pitch", type=float)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = TaskPitchStepClient(args.delta_pitch, args.timeout)
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
