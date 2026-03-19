"""Nudge the current task-space target by a small delta.

This helper reads the currently published task target (falling back to the
current task pose if needed), adds a delta, and republishes the result as a new
absolute target. It is more reliable for manual testing than short-lived
timeout-based delta publishers.
"""

from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class TaskStepClient(Node):
    def __init__(self, dx: float, dy: float, dz: float, timeout_s: float) -> None:
        super().__init__("task_step_client")
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.timeout_s = timeout_s

        self.target_msg: PointStamped | None = None
        self.current_msg: PointStamped | None = None

        self.target_sub = self.create_subscription(
            PointStamped,
            "/rotom_task/target_pose",
            self._target_cb,
            10,
        )
        self.current_sub = self.create_subscription(
            PointStamped,
            "/rotom_task/current_pose",
            self._current_cb,
            10,
        )
        self.target_pub = self.create_publisher(PointStamped, "/rotom_task/target", 10)

    def _target_cb(self, msg: PointStamped) -> None:
        self.target_msg = msg

    def _current_cb(self, msg: PointStamped) -> None:
        self.current_msg = msg

    def run(self) -> int:
        deadline = self.get_clock().now().nanoseconds + int(self.timeout_s * 1e9)
        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.target_msg is not None:
                break

        base_msg = self.target_msg if self.target_msg is not None else self.current_msg
        if base_msg is None:
            self.get_logger().error(
                "No /rotom_task/target_pose or /rotom_task/current_pose received; is task-core running?"
            )
            return 1

        new_target = PointStamped()
        new_target.header.stamp = self.get_clock().now().to_msg()
        new_target.header.frame_id = "ground"
        new_target.point.x = float(base_msg.point.x + self.dx)
        new_target.point.y = float(base_msg.point.y + self.dy)
        new_target.point.z = float(base_msg.point.z + self.dz)

        for _ in range(3):
            self.target_pub.publish(new_target)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(
            "task step sent: "
            f"({base_msg.point.x:.3f}, {base_msg.point.y:.3f}, {base_msg.point.z:.3f}) -> "
            f"({new_target.point.x:.3f}, {new_target.point.y:.3f}, {new_target.point.z:.3f})"
        )
        return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dx", type=float)
    parser.add_argument("dy", type=float)
    parser.add_argument("dz", type=float, nargs="?", default=0.0)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = TaskStepClient(args.dx, args.dy, args.dz, args.timeout)
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
