from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("image_monitor")

        self.declare_parameter("image_topic", "/camera/selected/image_raw")
        self.declare_parameter("window_name", "Rotom Image Monitor")
        self.declare_parameter("max_width", 0)
        self.declare_parameter("reliability", "best_effort")

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.max_width = int(self.get_parameter("max_width").value)
        self.reliability = str(self.get_parameter("reliability").value).strip().lower()
        self._last_log_ns = 0
        self._log_period_ns = 2_000_000_000

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.create_subscription(Image, self.image_topic, self._image_cb, self._make_qos())

        self.get_logger().info(
            f"image_monitor viewing {self.image_topic} in window '{self.window_name}' "
            f"with reliability={self.reliability}"
        )

    def _make_qos(self):
        if self.reliability == "reliable":
            return QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
            )
        return qos_profile_sensor_data

    @staticmethod
    def _imgmsg_to_bgr(msg: Image) -> Optional[np.ndarray]:
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ("bgr8", "rgb8"):
            frame = data.reshape(msg.height, msg.width, 3)
            if enc == "rgb8":
                frame = frame[:, :, ::-1]
            return np.ascontiguousarray(frame)
        if enc in ("bgra8", "rgba8"):
            frame = data.reshape(msg.height, msg.width, 4)
            if enc == "rgba8":
                frame = frame[:, :, [2, 1, 0, 3]]
            return np.ascontiguousarray(frame[:, :, :3])
        if enc == "mono8":
            gray = data.reshape(msg.height, msg.width)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return None

    def _image_cb(self, msg: Image) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns > self._log_period_ns:
            self.get_logger().info(
                f"received image {msg.width}x{msg.height} encoding={msg.encoding} from {self.image_topic}"
            )
            self._last_log_ns = now_ns

        frame = self._imgmsg_to_bgr(msg)
        if frame is None:
            if now_ns - self._last_log_ns > self._log_period_ns:
                self.get_logger().warn(f"Unsupported image encoding for viewer: {msg.encoding}")
                self._last_log_ns = now_ns
            return

        if self.max_width > 0 and frame.shape[1] > self.max_width:
            scale = self.max_width / float(frame.shape[1])
            frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyWindow(self.window_name)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ImageMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
