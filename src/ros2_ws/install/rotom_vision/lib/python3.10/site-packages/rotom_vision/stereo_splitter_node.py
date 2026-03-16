import copy
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml


class StereoSplitterNode(Node):
    def __init__(self) -> None:
        super().__init__("stereo_splitter")

        self.declare_parameter("input_image_topic", "/image_raw")
        self.declare_parameter("input_camera_info_topic", "/camera_info")
        self.declare_parameter("output_image_topic", "/camera/selected/image_raw")
        self.declare_parameter("output_camera_info_topic", "/camera/selected/camera_info")
        self.declare_parameter("eye", "left")
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("output_encoding", "bgr8")
        self.declare_parameter("output_reliability", "reliable")
        self.declare_parameter("publish_synthetic_camera_info_if_missing", True)
        self.declare_parameter("synthetic_focal_length_px", -1.0)
        self.declare_parameter("synthetic_distortion_model", "plumb_bob")
        self.declare_parameter("calibration_file", "")

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.input_camera_info_topic = str(self.get_parameter("input_camera_info_topic").value)
        self.output_image_topic = str(self.get_parameter("output_image_topic").value)
        self.output_camera_info_topic = str(self.get_parameter("output_camera_info_topic").value)
        self.eye = str(self.get_parameter("eye").value).strip().lower()
        self.queue_size = int(self.get_parameter("queue_size").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.output_encoding = str(self.get_parameter("output_encoding").value).strip()
        self.output_reliability = str(self.get_parameter("output_reliability").value).strip().lower()
        self.publish_synthetic_camera_info_if_missing = bool(
            self.get_parameter("publish_synthetic_camera_info_if_missing").value
        )
        self.synthetic_focal_length_px = float(self.get_parameter("synthetic_focal_length_px").value)
        self.synthetic_distortion_model = str(self.get_parameter("synthetic_distortion_model").value)
        calibration_file = str(self.get_parameter("calibration_file").value).strip()

        self._loaded_calibration: Optional[CameraInfo] = None
        self._calibration_size_warned = False
        if calibration_file:
            self._loaded_calibration = self._load_calibration_yaml(calibration_file)
            if self._loaded_calibration:
                self.get_logger().info(f"Loaded calibration from {calibration_file}")
            else:
                self.get_logger().error(f"Failed to load calibration from {calibration_file}")

        if self.eye not in {"left", "right"}:
            raise ValueError(f"Unsupported eye='{self.eye}'. Use 'left' or 'right'.")
        if self.output_reliability not in {"reliable", "best_effort"}:
            raise ValueError(
                f"Unsupported output_reliability='{self.output_reliability}'. "
                "Use 'reliable' or 'best_effort'."
            )

        self.latest_camera_info: Optional[CameraInfo] = None
        self._last_warn_ns = 0
        self._warn_period_ns = 2_000_000_000
        self._last_image_rx_ns = 0
        self._last_stall_warn_ns = 0
        self._stall_warn_period_ns = 2_000_000_000

        reliability = (
            ReliabilityPolicy.RELIABLE
            if self.output_reliability == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        )
        publisher_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(self.queue_size, 1),
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.image_pub = self.create_publisher(Image, self.output_image_topic, publisher_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.output_camera_info_topic, publisher_qos)

        self.create_subscription(CameraInfo, self.input_camera_info_topic, self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.input_image_topic, self._image_cb, qos_profile_sensor_data)
        self.create_timer(1.0, self._watchdog_cb)

        self.get_logger().info(
            "Stereo splitter active. "
            f"input={self.input_image_topic}, eye={self.eye}, output={self.output_image_topic}, "
            f"output_reliability={self.output_reliability}"
        )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _image_cb(self, msg: Image) -> None:
        self._last_image_rx_ns = self.get_clock().now().nanoseconds

        # Determine channels from encoding
        enc = msg.encoding.lower()
        if enc in ("rgb8", "bgr8"):
            channels = 3
        elif enc in ("rgba8", "bgra8"):
            channels = 4
        elif enc in ("mono8",):
            channels = 1
        else:
            self.get_logger().error(f"Unsupported encoding: {msg.encoding}")
            return

        data = np.frombuffer(msg.data, dtype=np.uint8)
        if channels == 1:
            frame = data.reshape(msg.height, msg.width)
        else:
            frame = data.reshape(msg.height, msg.width, channels)

        height, width = frame.shape[:2]
        if width < 2:
            self.get_logger().error(f"Image width too small for stereo split: width={width}")
            return

        half_width = width // 2
        x_offset = 0 if self.eye == "left" else width - half_width
        cropped = np.ascontiguousarray(frame[:, x_offset : x_offset + half_width])

        # Convert to requested output encoding (only rgb8<->bgr8 flip needed)
        out_enc = self.output_encoding if self.output_encoding else enc
        if out_enc != enc:
            if enc in ("rgb8", "bgr8") and out_enc in ("rgb8", "bgr8"):
                cropped = cropped[:, :, ::-1]  # flip R<->B
            else:
                self.get_logger().error(f"Cannot convert {enc} -> {out_enc}")
                return

        out_image = Image()
        out_image.header = msg.header
        if self.output_frame_id:
            out_image.header.frame_id = self.output_frame_id
        out_image.height = cropped.shape[0]
        out_image.width = cropped.shape[1]
        out_image.encoding = out_enc
        out_image.is_bigendian = 0
        out_image.step = cropped.strides[0]
        out_image.data = cropped.tobytes()
        self.image_pub.publish(out_image)

        # Use file-loaded calibration if available (highest priority)
        if self._loaded_calibration is not None:
            if (
                not self._calibration_size_warned
                and (
                    self._loaded_calibration.width != out_image.width
                    or self._loaded_calibration.height != out_image.height
                )
            ):
                self.get_logger().warn(
                    "Loaded calibration resolution "
                    f"{self._loaded_calibration.width}x{self._loaded_calibration.height} "
                    f"does not match the current split image {out_image.width}x{out_image.height}. "
                    "Marker pose may be inaccurate until the camera mode matches the calibration."
                )
                self._calibration_size_warned = True
            cal = copy.deepcopy(self._loaded_calibration)
            cal.header = out_image.header
            self.camera_info_pub.publish(cal)
            return

        if self.latest_camera_info is None or not self._is_camera_info_valid(self.latest_camera_info):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_warn_ns > self._warn_period_ns:
                if self.publish_synthetic_camera_info_if_missing:
                    self.get_logger().warn(
                        "Camera info missing or invalid; publishing synthetic intrinsics. "
                        "Run camera calibration for accurate marker pose."
                    )
                else:
                    self.get_logger().warn("Camera info missing or invalid; publishing image only.")
                self._last_warn_ns = now_ns
            if self.publish_synthetic_camera_info_if_missing:
                synthetic_info = self._build_synthetic_camera_info(half_width, height, out_image.header)
                self.camera_info_pub.publish(synthetic_info)
            return

        cropped_info = self._crop_camera_info(self.latest_camera_info, x_offset, half_width, height, out_image.header)
        self.camera_info_pub.publish(cropped_info)

    def _load_calibration_yaml(self, path: str) -> Optional[CameraInfo]:
        """Load a ROS camera_calibration YAML file and return a CameraInfo message."""
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
        except Exception as exc:
            self.get_logger().error(f"Cannot read calibration file '{path}': {exc}")
            return None

        try:
            msg = CameraInfo()
            msg.width = int(data["image_width"])
            msg.height = int(data["image_height"])
            msg.distortion_model = str(data.get("distortion_model", "plumb_bob"))
            msg.k = [float(v) for v in data["camera_matrix"]["data"]]
            msg.d = [float(v) for v in data["distortion_coefficients"]["data"]]
            msg.r = [float(v) for v in data["rectification_matrix"]["data"]]
            msg.p = [float(v) for v in data["projection_matrix"]["data"]]
            return msg
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Malformed calibration YAML '{path}': {exc}")
            return None

    def _crop_camera_info(
        self,
        source_info: CameraInfo,
        x_offset: int,
        width: int,
        height: int,
        header,
    ) -> CameraInfo:
        out = copy.deepcopy(source_info)
        out.header = header
        out.width = int(width)
        out.height = int(height)

        if len(out.k) == 9:
            out.k[2] = float(out.k[2] - x_offset)

        if len(out.p) == 12:
            out.p[2] = float(out.p[2] - x_offset)

        out.roi.x_offset = 0
        out.roi.y_offset = 0
        out.roi.width = int(width)
        out.roi.height = int(height)
        out.roi.do_rectify = False

        return out

    @staticmethod
    def _is_camera_info_valid(camera_info: CameraInfo) -> bool:
        if len(camera_info.k) != 9:
            return False
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        return fx > 0.0 and fy > 0.0

    def _build_synthetic_camera_info(self, width: int, height: int, header) -> CameraInfo:
        fx_fy = self.synthetic_focal_length_px if self.synthetic_focal_length_px > 0.0 else float(max(width, height))
        cx = (float(width) - 1.0) * 0.5
        cy = (float(height) - 1.0) * 0.5

        out = CameraInfo()
        out.header = header
        out.width = int(width)
        out.height = int(height)
        out.distortion_model = self.synthetic_distortion_model
        out.d = [0.0] * 5
        out.k = [fx_fy, 0.0, cx, 0.0, fx_fy, cy, 0.0, 0.0, 1.0]
        out.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        out.p = [fx_fy, 0.0, cx, 0.0, 0.0, fx_fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        out.roi.x_offset = 0
        out.roi.y_offset = 0
        out.roi.width = int(width)
        out.roi.height = int(height)
        out.roi.do_rectify = False
        return out

    def _watchdog_cb(self) -> None:
        if self._last_image_rx_ns == 0:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_image_rx_ns <= self._stall_warn_period_ns:
            return
        if now_ns - self._last_stall_warn_ns <= self._stall_warn_period_ns:
            return
        self._last_stall_warn_ns = now_ns
        idle_s = (now_ns - self._last_image_rx_ns) / 1e9
        self.get_logger().warn(
            f"No input image received for {idle_s:.1f}s on {self.input_image_topic}. "
            "Camera stream may be stalled."
        )


def main() -> None:
    rclpy.init()
    node = StereoSplitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
