from typing import Dict, Optional, Set

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
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


class ArucoTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_tracker")

        self.declare_parameter("image_topic", "/camera/selected/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/selected/camera_info")
        self.declare_parameter("dictionary", "DICT_5X5_100")
        self.declare_parameter("marker_size_m", 0.03)
        self.declare_parameter("frame_prefix", "aruco_marker_")
        self.declare_parameter("tracked_ids", Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("camera_frame_id", "")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_image_topic", "/aruco/debug_image")
        self.declare_parameter("queue_size", 10)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.dict_name = str(self.get_parameter("dictionary").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.frame_prefix = str(self.get_parameter("frame_prefix").value)
        tracked_ids = list(self.get_parameter("tracked_ids").value)
        self.tracked_ids: Set[int] = {int(marker_id) for marker_id in tracked_ids}
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.queue_size = int(self.get_parameter("queue_size").value)

        if self.marker_size_m <= 0.0:
            raise ValueError(f"marker_size_m must be positive. Got {self.marker_size_m}")

        if self.dict_name not in ARUCO_DICT_BY_NAME:
            supported = ", ".join(sorted(ARUCO_DICT_BY_NAME))
            raise ValueError(f"Unsupported dictionary '{self.dict_name}'. Supported: {supported}")

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, qos_profile_sensor_data)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_BY_NAME[self.dict_name])
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self._last_no_camera_info_warn_ns = 0
        self._warn_period_ns = 2_000_000_000

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"Aruco tracker active. image={self.image_topic}, dictionary={self.dict_name}, marker_size={self.marker_size_m:.4f}m"
        )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if len(msg.k) != 9:
            self.get_logger().error("CameraInfo.k must have 9 entries.")
            return

        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if msg.d:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        else:
            self.dist_coeffs = np.zeros(5, dtype=np.float64)

    def _image_cb(self, msg: Image) -> None:
        if self.camera_matrix is None or self.dist_coeffs is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_no_camera_info_warn_ns > self._warn_period_ns:
                self.get_logger().warn("Waiting for valid camera_info before running marker detection.")
                self._last_no_camera_info_warn_ns = now_ns
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params,
        )

        debug_frame = frame.copy() if self.publish_debug_image else None
        if ids is None or len(ids) == 0:
            if debug_frame is not None:
                self._publish_debug_image(debug_frame, msg.header)
            return

        ids_flat = ids.flatten().astype(int).tolist()
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size_m,
            self.camera_matrix,
            self.dist_coeffs,
        )

        frame_id = self.camera_frame_id if self.camera_frame_id else msg.header.frame_id
        for i, marker_id in enumerate(ids_flat):
            if self.tracked_ids and marker_id not in self.tracked_ids:
                continue

            rvec = rvecs[i, 0, :]
            tvec = tvecs[i, 0, :]

            rot_mtx, _ = cv2.Rodrigues(rvec)
            quat = matrix_to_quaternion(rot_mtx)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
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
                cv2.drawFrameAxes(debug_frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size_m * 0.6)

        if debug_frame is not None:
            cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)
            self._publish_debug_image(debug_frame, msg.header)

    def _publish_debug_image(self, frame: np.ndarray, header) -> None:
        if self.debug_pub is None:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header = header
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


if __name__ == "__main__":
    main()
