import json
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _resolve_output_root(root_value: str) -> Path:
    path = Path(root_value).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    return path.resolve()


class EpisodeRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("episode_recorder")

        self.declare_parameter("image_topic", "/camera/selected/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/selected/camera_info")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_commands")
        self.declare_parameter("delta_topic", "/rotom_task/delta_cmd")
        self.declare_parameter("raw_hand_pose_topic", "/rotom_task/raw_hand_pose")
        self.declare_parameter("teleop_enabled_topic", "/rotom_task/teleop_enabled")
        self.declare_parameter("controller_enabled_topic", "/rotom_task/controller_enabled")
        self.declare_parameter("current_pose_topic", "/rotom_task/current_pose")
        self.declare_parameter("target_pose_topic", "/rotom_task/target_pose")
        self.declare_parameter("command_pose_topic", "/rotom_task/command_pose")
        self.declare_parameter("current_pitch_topic", "/rotom_task/current_pitch")
        self.declare_parameter("target_pitch_topic", "/rotom_task/target_pitch")
        self.declare_parameter("command_pitch_topic", "/rotom_task/command_pitch")
        self.declare_parameter("output_root", "data/demos")
        self.declare_parameter("image_subdir", "images")
        self.declare_parameter("image_format", "jpg")
        self.declare_parameter("image_jpeg_quality", 90)
        self.declare_parameter("max_record_rate_hz", 20.0)
        self.declare_parameter("resize_width", 112)
        self.declare_parameter("resize_height", 112)
        self.declare_parameter("save_camera_info", True)
        self.declare_parameter("start_recording_on_launch", False)
        self.declare_parameter("start_service", "/rotom_dataset/start_episode")
        self.declare_parameter("stop_service", "/rotom_dataset/stop_episode")
        self.declare_parameter("stop_success_service", "/rotom_dataset/stop_episode_success")
        self.declare_parameter("stop_failure_service", "/rotom_dataset/stop_episode_failure")
        self.declare_parameter("discard_service", "/rotom_dataset/discard_episode")
        self.declare_parameter("recording_topic", "/rotom_dataset/recording")
        self.declare_parameter("current_episode_topic", "/rotom_dataset/current_episode")
        self.declare_parameter("processed_image_topic", "/rotom_dataset/image_resized")
        self.declare_parameter("warn_period_s", 2.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self.delta_topic = str(self.get_parameter("delta_topic").value)
        self.raw_hand_pose_topic = str(self.get_parameter("raw_hand_pose_topic").value)
        self.teleop_enabled_topic = str(self.get_parameter("teleop_enabled_topic").value)
        self.controller_enabled_topic = str(self.get_parameter("controller_enabled_topic").value)
        self.current_pose_topic = str(self.get_parameter("current_pose_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.command_pose_topic = str(self.get_parameter("command_pose_topic").value)
        self.current_pitch_topic = str(self.get_parameter("current_pitch_topic").value)
        self.target_pitch_topic = str(self.get_parameter("target_pitch_topic").value)
        self.command_pitch_topic = str(self.get_parameter("command_pitch_topic").value)
        self.output_root = _resolve_output_root(str(self.get_parameter("output_root").value))
        self.image_subdir = str(self.get_parameter("image_subdir").value)
        self.image_format = str(self.get_parameter("image_format").value).lower()
        self.image_jpeg_quality = int(self.get_parameter("image_jpeg_quality").value)
        self.max_record_rate_hz = float(self.get_parameter("max_record_rate_hz").value)
        self.resize_width = int(self.get_parameter("resize_width").value)
        self.resize_height = int(self.get_parameter("resize_height").value)
        self.save_camera_info = bool(self.get_parameter("save_camera_info").value)
        self.start_recording_on_launch = bool(self.get_parameter("start_recording_on_launch").value)
        self.start_service_name = str(self.get_parameter("start_service").value)
        self.stop_service_name = str(self.get_parameter("stop_service").value)
        self.stop_success_service_name = str(self.get_parameter("stop_success_service").value)
        self.stop_failure_service_name = str(self.get_parameter("stop_failure_service").value)
        self.discard_service_name = str(self.get_parameter("discard_service").value)
        self.recording_topic = str(self.get_parameter("recording_topic").value)
        self.current_episode_topic = str(self.get_parameter("current_episode_topic").value)
        self.processed_image_topic = str(self.get_parameter("processed_image_topic").value)
        self.warn_period_ns = int(float(self.get_parameter("warn_period_s").value) * 1e9)

        if self.image_format not in ("jpg", "jpeg", "png"):
            raise ValueError("image_format must be one of: jpg, jpeg, png")
        if self.max_record_rate_hz <= 0.0:
            raise ValueError("max_record_rate_hz must be > 0.0")
        if self.resize_width <= 0 or self.resize_height <= 0:
            raise ValueError("resize_width and resize_height must be > 0")

        self.bridge = CvBridge()
        self.output_root.mkdir(parents=True, exist_ok=True)

        self.recording_pub = self.create_publisher(Bool, self.recording_topic, 10)
        self.current_episode_pub = self.create_publisher(String, self.current_episode_topic, 10)
        self.processed_image_pub = self.create_publisher(Image, self.processed_image_topic, qos_profile_sensor_data)

        self.create_service(Trigger, self.start_service_name, self._start_episode_cb)
        self.create_service(Trigger, self.stop_service_name, self._stop_episode_cb)
        self.create_service(Trigger, self.stop_success_service_name, self._stop_episode_success_cb)
        self.create_service(Trigger, self.stop_failure_service_name, self._stop_episode_failure_cb)
        self.create_service(Trigger, self.discard_service_name, self._discard_episode_cb)

        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 10)
        self.create_subscription(JointState, self.joint_command_topic, self._joint_command_cb, 10)
        self.create_subscription(Vector3, self.delta_topic, self._delta_cb, 10)
        self.create_subscription(PoseStamped, self.raw_hand_pose_topic, self._raw_hand_pose_cb, 10)
        self.create_subscription(Bool, self.teleop_enabled_topic, self._teleop_enabled_cb, 10)
        self.create_subscription(Bool, self.controller_enabled_topic, self._controller_enabled_cb, 10)
        self.create_subscription(PointStamped, self.current_pose_topic, self._current_pose_cb, 10)
        self.create_subscription(PointStamped, self.target_pose_topic, self._target_pose_cb, 10)
        self.create_subscription(PointStamped, self.command_pose_topic, self._command_pose_cb, 10)
        self.create_subscription(Float64, self.current_pitch_topic, self._current_pitch_cb, 10)
        self.create_subscription(Float64, self.target_pitch_topic, self._target_pitch_cb, 10)
        self.create_subscription(Float64, self.command_pitch_topic, self._command_pitch_cb, 10)
        self.record_timer = self.create_timer(1.0 / self.max_record_rate_hz, self._record_step)

        self._recording = False
        self._episode_id = ""
        self._episode_dir: Optional[Path] = None
        self._images_dir: Optional[Path] = None
        self._meta_path: Optional[Path] = None
        self._camera_info_path: Optional[Path] = None
        self._steps_handle = None
        self._episode_start_ns: Optional[int] = None
        self._step_index = 0
        self._last_record_stamp_ns: Optional[int] = None
        self._last_warn_ns = 0
        self._camera_info_written = False
        self._episode_meta: dict[str, Any] = {}

        self._latest_camera_info: Optional[dict[str, Any]] = None
        self._latest_joint_state: Optional[dict[str, Any]] = None
        self._latest_joint_command: Optional[dict[str, Any]] = None
        self._latest_delta_cmd: Optional[dict[str, Any]] = None
        self._latest_raw_hand_pose: Optional[dict[str, Any]] = None
        self._latest_teleop_enabled: Optional[dict[str, Any]] = None
        self._latest_controller_enabled: Optional[dict[str, Any]] = None
        self._latest_current_pose: Optional[dict[str, Any]] = None
        self._latest_target_pose: Optional[dict[str, Any]] = None
        self._latest_command_pose: Optional[dict[str, Any]] = None
        self._latest_current_pitch: Optional[dict[str, Any]] = None
        self._latest_target_pitch: Optional[dict[str, Any]] = None
        self._latest_command_pitch: Optional[dict[str, Any]] = None
        self._latest_image_array: Optional[np.ndarray] = None
        self._latest_image_stamp_ns: Optional[int] = None
        self._latest_image_encoding = ""

        self._publish_status()
        self.get_logger().info(
            f"episode_recorder active. image_topic={self.image_topic}, processed_image_topic={self.processed_image_topic}, output_root={self.output_root}, fps={self.max_record_rate_hz:.1f}, image={self.resize_width}x{self.resize_height}"
        )

        if self.start_recording_on_launch:
            ok, message = self._start_episode_internal()
            if ok:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)

    def destroy_node(self) -> bool:
        self._close_episode_file()
        return super().destroy_node()

    def _warn(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns > self.warn_period_ns:
            self.get_logger().warn(message)
            self._last_warn_ns = now_ns

    def _publish_status(self) -> None:
        active = Bool()
        active.data = self._recording
        self.recording_pub.publish(active)

        episode_msg = String()
        episode_msg.data = self._episode_id
        self.current_episode_pub.publish(episode_msg)

    def _serialize_joint_state(self, msg: JointState) -> dict[str, Any]:
        return {
            "stamp_ns": _stamp_to_ns(msg.header.stamp),
            "name": list(msg.name),
            "position": [float(v) for v in msg.position],
            "velocity": [float(v) for v in msg.velocity],
            "effort": [float(v) for v in msg.effort],
        }

    def _serialize_point(self, msg: PointStamped) -> dict[str, Any]:
        return {
            "stamp_ns": _stamp_to_ns(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "xyz": [float(msg.point.x), float(msg.point.y), float(msg.point.z)],
        }

    def _serialize_pose(self, msg: PoseStamped) -> dict[str, Any]:
        return {
            "stamp_ns": _stamp_to_ns(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "position": [float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)],
            "orientation_xyzw": [
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            ],
        }

    def _serialize_vector3(self, msg: Vector3) -> dict[str, Any]:
        return {"stamp_ns": self.get_clock().now().nanoseconds, "xyz": [float(msg.x), float(msg.y), float(msg.z)]}

    def _serialize_bool(self, msg: Bool) -> dict[str, Any]:
        return {"stamp_ns": self.get_clock().now().nanoseconds, "data": bool(msg.data)}

    def _serialize_float(self, msg: Float64) -> dict[str, Any]:
        return {"stamp_ns": self.get_clock().now().nanoseconds, "data": float(msg.data)}

    def _serialize_camera_info(self, msg: CameraInfo) -> dict[str, Any]:
        return {
            "stamp_ns": _stamp_to_ns(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "width": int(self.resize_width),
            "height": int(self.resize_height),
            "distortion_model": msg.distortion_model,
            "d": [float(v) for v in msg.d],
            "k": [float(v) for v in msg.k],
            "r": [float(v) for v in msg.r],
            "p": [float(v) for v in msg.p],
            "binning_x": int(msg.binning_x),
            "binning_y": int(msg.binning_y),
        }

    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = self._serialize_joint_state(msg)

    def _joint_command_cb(self, msg: JointState) -> None:
        self._latest_joint_command = self._serialize_joint_state(msg)

    def _delta_cb(self, msg: Vector3) -> None:
        self._latest_delta_cmd = self._serialize_vector3(msg)

    def _raw_hand_pose_cb(self, msg: PoseStamped) -> None:
        self._latest_raw_hand_pose = self._serialize_pose(msg)

    def _teleop_enabled_cb(self, msg: Bool) -> None:
        self._latest_teleop_enabled = self._serialize_bool(msg)

    def _controller_enabled_cb(self, msg: Bool) -> None:
        self._latest_controller_enabled = self._serialize_bool(msg)

    def _current_pose_cb(self, msg: PointStamped) -> None:
        self._latest_current_pose = self._serialize_point(msg)

    def _target_pose_cb(self, msg: PointStamped) -> None:
        self._latest_target_pose = self._serialize_point(msg)

    def _command_pose_cb(self, msg: PointStamped) -> None:
        self._latest_command_pose = self._serialize_point(msg)

    def _current_pitch_cb(self, msg: Float64) -> None:
        self._latest_current_pitch = self._serialize_float(msg)

    def _target_pitch_cb(self, msg: Float64) -> None:
        self._latest_target_pitch = self._serialize_float(msg)

    def _command_pitch_cb(self, msg: Float64) -> None:
        self._latest_command_pitch = self._serialize_float(msg)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._latest_camera_info = self._serialize_camera_info(msg)
        self._maybe_write_camera_info()

    @staticmethod
    def _center_square_crop(image: np.ndarray) -> np.ndarray:
        height, width = image.shape[:2]
        crop_size = min(height, width)
        y0 = max((height - crop_size) // 2, 0)
        x0 = max((width - crop_size) // 2, 0)
        return image[y0 : y0 + crop_size, x0 : x0 + crop_size]

    def _prepare_image(self, image: np.ndarray, encoding: str) -> np.ndarray:
        enc = encoding.lower()
        if image.ndim == 2:
            image = np.repeat(image[:, :, None], 3, axis=2)
        elif image.ndim == 3 and image.shape[2] == 1:
            image = np.repeat(image, 3, axis=2)
        elif image.ndim == 3 and image.shape[2] == 4:
            if enc == "rgba8":
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
            else:
                image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
        elif image.ndim == 3 and image.shape[2] == 3:
            if enc != "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        image = self._center_square_crop(image)
        if image.shape[1] != self.resize_width or image.shape[0] != self.resize_height:
            image = cv2.resize(image, (self.resize_width, self.resize_height), interpolation=cv2.INTER_AREA)

        if image.dtype != np.uint8:
            image = np.clip(image, 0, 255).astype(np.uint8)
        return image

    def _image_cb(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # pragma: no cover - runtime integration path
            self._warn(f"failed converting image for recorder cache: {exc}")
            return
        processed_image = self._prepare_image(image, msg.encoding)
        self._latest_image_array = processed_image
        stamp_ns = _stamp_to_ns(msg.header.stamp)
        self._latest_image_stamp_ns = stamp_ns if stamp_ns > 0 else self.get_clock().now().nanoseconds
        self._latest_image_encoding = "rgb8"

        processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="rgb8")
        processed_msg.header = msg.header
        self.processed_image_pub.publish(processed_msg)

    def _build_episode_meta(self) -> dict[str, Any]:
        assert self._episode_dir is not None
        return {
            "dataset_type": "rotom_demo_dataset_v2",
            "episode_id": self._episode_id,
            "start_time_ns": self._episode_start_ns,
            "output_dir": str(self._episode_dir),
            "steps_file": "steps.jsonl",
            "image_dir": self.image_subdir,
            "image_format": self.image_format,
            "record_rate_hz": self.max_record_rate_hz,
            "image_size": [self.resize_height, self.resize_width, 3],
            "label": "unknown",
            "success": None,
            "task_id": None,
            "failure_code": None,
            "notes": "",
            "topics": {
                "image": self.image_topic,
                "camera_info": self.camera_info_topic,
                "joint_state": self.joint_state_topic,
                "joint_command": self.joint_command_topic,
                "delta_cmd": self.delta_topic,
                "raw_hand_pose": self.raw_hand_pose_topic,
                "teleop_enabled": self.teleop_enabled_topic,
                "controller_enabled": self.controller_enabled_topic,
                "current_pose": self.current_pose_topic,
                "target_pose": self.target_pose_topic,
                "command_pose": self.command_pose_topic,
                "current_pitch": self.current_pitch_topic,
                "target_pitch": self.target_pitch_topic,
                "command_pitch": self.command_pitch_topic,
            },
        }

    def _write_meta(self) -> None:
        if self._meta_path is None:
            return
        self._meta_path.write_text(json.dumps(self._episode_meta, indent=2) + "\n", encoding="utf-8")

    def _close_episode_file(self) -> None:
        if self._steps_handle is not None:
            self._steps_handle.close()
            self._steps_handle = None

    def _start_episode_internal(self) -> tuple[bool, str]:
        if self._recording:
            return False, f"already recording episode {self._episode_id}"

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self._episode_id = f"episode_{timestamp}"
        self._episode_dir = self.output_root / self._episode_id
        self._images_dir = self._episode_dir / self.image_subdir
        self._meta_path = self._episode_dir / "meta.json"
        self._camera_info_path = self._episode_dir / "camera_info.json"
        self._episode_dir.mkdir(parents=True, exist_ok=False)
        self._images_dir.mkdir(parents=True, exist_ok=False)
        self._steps_handle = (self._episode_dir / "steps.jsonl").open("w", encoding="utf-8")
        self._episode_start_ns = self.get_clock().now().nanoseconds
        self._step_index = 0
        self._last_record_stamp_ns = None
        self._camera_info_written = False
        self._episode_meta = self._build_episode_meta()
        self._write_meta()
        self._recording = True
        self._publish_status()
        self._maybe_write_camera_info()
        return True, f"recording to {self._episode_dir}"

    def _finalize_episode(self) -> None:
        end_time_ns = self.get_clock().now().nanoseconds
        self._episode_meta["end_time_ns"] = end_time_ns
        if self._episode_start_ns is not None:
            self._episode_meta["duration_s"] = (end_time_ns - self._episode_start_ns) * 1e-9
        self._episode_meta["sample_count"] = self._step_index
        self._write_meta()
        self._close_episode_file()
        self._recording = False
        self._publish_status()

    def _start_episode_cb(self, request, response):
        del request
        try:
            ok, message = self._start_episode_internal()
        except FileExistsError:
            ok = False
            message = "episode directory already exists unexpectedly; retry start"
        response.success = ok
        response.message = message
        return response

    def _stop_episode_with_label(self, success: Optional[bool], label: str) -> tuple[bool, str]:
        if not self._recording:
            return False, "no active episode"

        episode_dir = self._episode_dir
        self._episode_meta["label"] = label
        self._episode_meta["success"] = success
        self._finalize_episode()
        self._episode_id = ""
        self._publish_status()
        return True, f"saved episode to {episode_dir}"

    def _stop_episode_cb(self, request, response):
        del request
        response.success, response.message = self._stop_episode_with_label(success=None, label="unknown")
        return response

    def _stop_episode_success_cb(self, request, response):
        del request
        response.success, response.message = self._stop_episode_with_label(success=True, label="success")
        return response

    def _stop_episode_failure_cb(self, request, response):
        del request
        response.success, response.message = self._stop_episode_with_label(success=False, label="failure")
        return response

    def _discard_episode_cb(self, request, response):
        del request
        target_dir = self._episode_dir
        if target_dir is None:
            response.success = False
            response.message = "no active or staged episode to discard"
            return response

        self._close_episode_file()
        self._recording = False
        self._episode_id = ""
        self._publish_status()

        try:
            shutil.rmtree(target_dir)
        except FileNotFoundError:
            pass

        self._episode_dir = None
        self._images_dir = None
        self._meta_path = None
        self._camera_info_path = None
        self._episode_meta = {}
        self._episode_start_ns = None
        self._step_index = 0
        self._last_record_stamp_ns = None
        self._camera_info_written = False

        response.success = True
        response.message = f"discarded {target_dir}"
        return response

    def _maybe_write_camera_info(self) -> None:
        if (
            not self.save_camera_info
            or not self._recording
            or self._camera_info_written
            or self._latest_camera_info is None
            or self._camera_info_path is None
        ):
            return
        self._camera_info_path.write_text(json.dumps(self._latest_camera_info, indent=2) + "\n", encoding="utf-8")
        self._episode_meta["camera_info_file"] = "camera_info.json"
        self._write_meta()
        self._camera_info_written = True

    def _snapshot_step(self, image_rel_path: str, image_shape: list[int], sample_stamp_ns: int) -> dict[str, Any]:
        image_age_ms = None
        if self._latest_image_stamp_ns is not None:
            image_age_ms = (sample_stamp_ns - self._latest_image_stamp_ns) / 1e6
        return {
            "index": self._step_index,
            "stamp_ns": sample_stamp_ns,
            "source_image_stamp_ns": self._latest_image_stamp_ns,
            "image_age_ms": image_age_ms,
            "image_path": image_rel_path,
            "image_encoding": self._latest_image_encoding,
            "image_shape": image_shape,
            "observation": {
                "joint_state": self._latest_joint_state,
                "current_pose": self._latest_current_pose,
                "current_pitch": self._latest_current_pitch,
                "raw_hand_pose": self._latest_raw_hand_pose,
            },
            "action": {
                "joint_command": self._latest_joint_command,
                "task_delta_cmd": self._latest_delta_cmd,
                "teleop_enabled": self._latest_teleop_enabled,
                "controller_enabled": self._latest_controller_enabled,
                "target_pose": self._latest_target_pose,
                "command_pose": self._latest_command_pose,
                "target_pitch": self._latest_target_pitch,
                "command_pitch": self._latest_command_pitch,
            },
        }

    def _record_step(self) -> None:
        if not self._recording:
            return
        if self._images_dir is None or self._steps_handle is None:
            self._warn("recorder is active but episode files are not ready")
            return
        if self._latest_image_array is None:
            self._warn("waiting for camera frames before recording")
            return

        self._maybe_write_camera_info()
        sample_stamp_ns = self.get_clock().now().nanoseconds
        extension = "jpg" if self.image_format == "jpeg" else self.image_format
        image_filename = f"frame_{self._step_index:06d}.{extension}"
        image_rel_path = f"{self.image_subdir}/{image_filename}"
        image_path = self._images_dir / image_filename
        encode_image = cv2.cvtColor(self._latest_image_array, cv2.COLOR_RGB2BGR)

        if extension == "png":
            ok, encoded = cv2.imencode(".png", encode_image)
        else:
            ok, encoded = cv2.imencode(
                ".jpg",
                encode_image,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.image_jpeg_quality)],
            )
        if not ok:
            self._warn("failed to encode frame for dataset recording")
            return

        image_path.write_bytes(encoded.tobytes())
        image_shape = [int(dim) for dim in self._latest_image_array.shape]
        step = self._snapshot_step(image_rel_path, image_shape, sample_stamp_ns)
        self._steps_handle.write(json.dumps(step) + "\n")
        self._steps_handle.flush()
        self._step_index += 1
        self._last_record_stamp_ns = sample_stamp_ns


def main() -> None:
    rclpy.init()
    node = EpisodeRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
