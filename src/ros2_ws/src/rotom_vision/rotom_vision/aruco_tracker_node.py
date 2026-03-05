from typing import Dict, Optional, Set

import cv2
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

        self.tf_broadcaster = TransformBroadcaster(self)
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, qos_profile_sensor_data)

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_BY_NAME[self.dict_name])
        detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

        # Object points for a square marker (used by solvePnP)
        h = self.marker_size_m / 2.0
        self._marker_obj_pts = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float64)

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

    @staticmethod
    def _imgmsg_to_bgr(msg: Image) -> Optional[np.ndarray]:
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ("bgr8", "rgb8"):
            frame = data.reshape(msg.height, msg.width, 3)
            if enc == "rgb8":
                frame = frame[:, :, ::-1]
            return np.ascontiguousarray(frame)
        elif enc in ("bgra8", "rgba8"):
            frame = data.reshape(msg.height, msg.width, 4)
            if enc == "rgba8":
                frame = frame[:, :, [2, 1, 0, 3]]
            return np.ascontiguousarray(frame[:, :, :3])
        elif enc == "mono8":
            gray = data.reshape(msg.height, msg.width)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        elif enc == "yuyv":
            yuv = data.reshape(msg.height, msg.width, 2)
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
        return None

    def _image_cb(self, msg: Image) -> None:
        if self.camera_matrix is None or self.dist_coeffs is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_no_camera_info_warn_ns > self._warn_period_ns:
                self.get_logger().warn("Waiting for valid camera_info before running marker detection.")
                self._last_no_camera_info_warn_ns = now_ns
            return

        frame = self._imgmsg_to_bgr(msg)
        if frame is None:
            self.get_logger().error(f"Unsupported image encoding: {msg.encoding}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        debug_frame = frame.copy() if self.publish_debug_image else None
        if ids is None or len(ids) == 0:
            if debug_frame is not None:
                self._publish_debug_image(debug_frame, msg.header)
            return

        ids_flat = ids.flatten().astype(int).tolist()
        frame_id = self.camera_frame_id if self.camera_frame_id else msg.header.frame_id

        for i, marker_id in enumerate(ids_flat):
            if self.tracked_ids and marker_id not in self.tracked_ids:
                continue

            img_pts = corners[i][0].astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                self._marker_obj_pts, img_pts,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue

            rvec = rvec.flatten()
            tvec = tvec.flatten()
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


if __name__ == "__main__":
    main()
