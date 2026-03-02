import copy
from typing import Optional

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


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
        self.declare_parameter("publish_synthetic_camera_info_if_missing", True)
        self.declare_parameter("synthetic_focal_length_px", -1.0)
        self.declare_parameter("synthetic_distortion_model", "plumb_bob")

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.input_camera_info_topic = str(self.get_parameter("input_camera_info_topic").value)
        self.output_image_topic = str(self.get_parameter("output_image_topic").value)
        self.output_camera_info_topic = str(self.get_parameter("output_camera_info_topic").value)
        self.eye = str(self.get_parameter("eye").value).strip().lower()
        self.queue_size = int(self.get_parameter("queue_size").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.publish_synthetic_camera_info_if_missing = bool(
            self.get_parameter("publish_synthetic_camera_info_if_missing").value
        )
        self.synthetic_focal_length_px = float(self.get_parameter("synthetic_focal_length_px").value)
        self.synthetic_distortion_model = str(self.get_parameter("synthetic_distortion_model").value)

        if self.eye not in {"left", "right"}:
            raise ValueError(f"Unsupported eye='{self.eye}'. Use 'left' or 'right'.")

        self.bridge = CvBridge()
        self.latest_camera_info: Optional[CameraInfo] = None
        self._last_warn_ns = 0
        self._warn_period_ns = 2_000_000_000

        self.image_pub = self.create_publisher(Image, self.output_image_topic, qos_profile_sensor_data)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.output_camera_info_topic, qos_profile_sensor_data)

        self.create_subscription(CameraInfo, self.input_camera_info_topic, self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.input_image_topic, self._image_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"Stereo splitter active. input={self.input_image_topic}, eye={self.eye}, output={self.output_image_topic}"
        )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        if frame.ndim < 2:
            self.get_logger().error("Unexpected image shape with fewer than 2 dimensions.")
            return

        height, width = frame.shape[:2]
        if width < 2:
            self.get_logger().error(f"Image width too small for stereo split: width={width}")
            return

        half_width = width // 2
        if self.eye == "left":
            x_offset = 0
        else:
            x_offset = width - half_width

        cropped = frame[:, x_offset : x_offset + half_width]

        encoding = msg.encoding if msg.encoding else "passthrough"
        out_image = self.bridge.cv2_to_imgmsg(cropped, encoding=encoding)
        out_image.header = msg.header
        if self.output_frame_id:
            out_image.header.frame_id = self.output_frame_id
        self.image_pub.publish(out_image)

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
