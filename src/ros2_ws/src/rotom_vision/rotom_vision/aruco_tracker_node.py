from typing import Dict, Optional, Set

import cv2
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from .math_utils import matrix_to_quaternion


ARUCO_DICT_BY_NAME: Dict[str, int] = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}


def _make_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def _make_detector(aruco_dict, detector_params):
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(aruco_dict, detector_params)
    return None


class ArucoTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_tracker")

        self.declare_parameter("image_topic", "/camera/selected/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/selected/camera_info")
        self.declare_parameter("dictionary", "DICT_5X5_100")
        self.declare_parameter("marker_size_m", 0.011)
        self.declare_parameter("frame_prefix", "aruco_marker_")
        self.declare_parameter("tracked_ids", [1, 2])
        self.declare_parameter("camera_frame_id", "")
        self.declare_parameter("tf_use_current_time", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/aruco/debug_image")

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.dict_name = str(self.get_parameter("dictionary").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.frame_prefix = str(self.get_parameter("frame_prefix").value)
        tracked_ids = list(self.get_parameter("tracked_ids").value)
        self.tracked_ids: Set[int] = {int(marker_id) for marker_id in tracked_ids}
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.tf_use_current_time = bool(self.get_parameter("tf_use_current_time").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        if self.marker_size_m <= 0.0:
            raise ValueError(f"marker_size_m must be positive. Got {self.marker_size_m}")
        if self.dict_name not in ARUCO_DICT_BY_NAME:
            supported = ", ".join(sorted(ARUCO_DICT_BY_NAME))
            raise ValueError(f"Unsupported dictionary '{self.dict_name}'. Supported: {supported}")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.debug_pub = None
        if self.publish_debug_image:
            debug_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, debug_qos)
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self._last_no_camera_info_warn_ns = 0
        self._warn_period_ns = 2_000_000_000
        self._last_image_log_ns = 0
        self._last_detection_log_ns = 0
        self._log_period_ns = 2_000_000_000

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_BY_NAME[self.dict_name])
        detector_params = _make_detector_parameters()
        self.aruco_dict = aruco_dict
        self.detector_params = detector_params
        self.detector = _make_detector(aruco_dict, detector_params)

        half_size = self.marker_size_m / 2.0
        self._marker_obj_pts = np.array(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float64,
        )

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"aruco_tracker active. image={self.image_topic}, dict={self.dict_name}, "
            f"marker_size={self.marker_size_m:.4f}m, tracked_ids={sorted(self.tracked_ids)}"
        )
        if self.publish_debug_image:
            self.get_logger().info(f"Publishing annotated debug images on {self.debug_image_topic}")

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if len(msg.k) != 9:
            self.get_logger().error("CameraInfo.k must have 9 entries.")
            return

        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d if msg.d else [0.0] * 5, dtype=np.float64)

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
        if now_ns - self._last_image_log_ns > self._log_period_ns:
            self.get_logger().info(
                f"received image {msg.width}x{msg.height} encoding={msg.encoding} frame_id={msg.header.frame_id}"
            )
            self._last_image_log_ns = now_ns

        if self.camera_matrix is None or self.dist_coeffs is None:
            if now_ns - self._last_no_camera_info_warn_ns > self._warn_period_ns:
                self.get_logger().warn("Waiting for valid camera_info before running marker detection.")
                self._last_no_camera_info_warn_ns = now_ns
            return

        frame = self._imgmsg_to_bgr(msg)
        if frame is None:
            self.get_logger().error(f"Unsupported image encoding: {msg.encoding}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)

        debug_frame = frame.copy() if self.publish_debug_image else None
        if ids is None or len(ids) == 0:
            if now_ns - self._last_detection_log_ns > self._log_period_ns:
                self.get_logger().info("no tracked ArUco markers detected in current frame")
                self._last_detection_log_ns = now_ns
            if debug_frame is not None:
                self._publish_debug_image(debug_frame, msg.header)
            return

        ids_flat = ids.flatten().astype(int).tolist()
        if now_ns - self._last_detection_log_ns > self._log_period_ns:
            self.get_logger().info(f"detected marker ids: {ids_flat}")
            self._last_detection_log_ns = now_ns
        frame_id = self.camera_frame_id if self.camera_frame_id else msg.header.frame_id

        for i, marker_id in enumerate(ids_flat):
            if self.tracked_ids and marker_id not in self.tracked_ids:
                continue

            img_pts = corners[i][0].astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                self._marker_obj_pts,
                img_pts,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue

            rot_mtx, _ = cv2.Rodrigues(rvec.flatten())
            quat = matrix_to_quaternion(rot_mtx)
            tvec = tvec.flatten()

            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg() if self.tf_use_current_time else msg.header.stamp
            tf_msg.header.frame_id = frame_id
            tf_msg.child_frame_id = f"{self.frame_prefix}{marker_id}"
            tf_msg.transform.translation.x = float(tvec[0])
            tf_msg.transform.translation.y = float(tvec[1])
            tf_msg.transform.translation.z = float(tvec[2])
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(tf_msg)

            if debug_frame is not None:
                cv2.drawFrameAxes(
                    debug_frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size_m * 0.6,
                )

        if debug_frame is not None:
            cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)
            self._publish_debug_image(debug_frame, msg.header)

    def _publish_debug_image(self, frame: np.ndarray, header) -> None:
        if self.debug_pub is None:
            return

        msg = Image()
        msg.header = header
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = frame.strides[0]
        msg.data = np.ascontiguousarray(frame).tobytes()
        self.debug_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ArucoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
