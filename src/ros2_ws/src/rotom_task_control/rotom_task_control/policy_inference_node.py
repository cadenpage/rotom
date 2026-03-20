from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _clamp_xy(vec: np.ndarray, max_magnitude: float) -> np.ndarray:
    if max_magnitude <= 0.0:
        return vec
    magnitude = float(np.linalg.norm(vec))
    if magnitude <= max_magnitude or magnitude <= 1e-12:
        return vec
    return vec * (max_magnitude / magnitude)


class PolicyInferenceNode(Node):
    def __init__(self) -> None:
        super().__init__('policy_inference')

        self.declare_parameter('policy_repo_id', 'caden-ut/rotom_task_teleop_diffusion')
        self.declare_parameter('local_policy_path', '/home/caden/Documents/rotom/policies/rotom_task_teleop_diffusion/pretrained_model')
        self.declare_parameter('image_topic', '/camera/selected/image_raw')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('action_topic', '/rotom_task/delta_cmd')
        self.declare_parameter('processed_image_topic', '/rotom_policy/image_input')
        self.declare_parameter('enabled_topic', '/rotom_policy/enabled')
        self.declare_parameter('start_service', '/rotom_policy/start')
        self.declare_parameter('stop_service', '/rotom_policy/stop')
        self.declare_parameter('task_name', 'table_teleop')
        self.declare_parameter('robot_type', 'rotom')
        self.declare_parameter('joint_names', ['O', 'A', 'B', 'C'])
        self.declare_parameter('inference_rate_hz', 20.0)
        self.declare_parameter('observation_timeout_s', 0.5)
        self.declare_parameter('resize_width', 112)
        self.declare_parameter('resize_height', 112)
        self.declare_parameter('action_scale_xy', [1.0, 1.0])
        self.declare_parameter('action_signs_xy', [1.0, 1.0])
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('max_delta_speed', 0.20)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('use_fp16', True)
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('publish_zero_when_disabled', True)
        self.declare_parameter('warn_period_s', 2.0)

        self.policy_repo_id = str(self.get_parameter('policy_repo_id').value)
        self.local_policy_path = Path(str(self.get_parameter('local_policy_path').value)).expanduser().resolve()
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.action_topic = str(self.get_parameter('action_topic').value)
        self.processed_image_topic = str(self.get_parameter('processed_image_topic').value)
        self.enabled_topic = str(self.get_parameter('enabled_topic').value)
        self.start_service_name = str(self.get_parameter('start_service').value)
        self.stop_service_name = str(self.get_parameter('stop_service').value)
        self.task_name = str(self.get_parameter('task_name').value)
        self.robot_type = str(self.get_parameter('robot_type').value)
        self.joint_names = tuple(str(v) for v in self.get_parameter('joint_names').value)
        self.inference_rate_hz = float(self.get_parameter('inference_rate_hz').value)
        self.observation_timeout_s = float(self.get_parameter('observation_timeout_s').value)
        self.resize_width = int(self.get_parameter('resize_width').value)
        self.resize_height = int(self.get_parameter('resize_height').value)
        self.action_scale_xy = np.asarray(self.get_parameter('action_scale_xy').value, dtype=np.float32)
        self.action_signs_xy = np.asarray(self.get_parameter('action_signs_xy').value, dtype=np.float32)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.max_delta_speed = float(self.get_parameter('max_delta_speed').value)
        self.device_override = str(self.get_parameter('device').value)
        self.use_fp16 = bool(self.get_parameter('use_fp16').value)
        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.publish_zero_when_disabled = bool(self.get_parameter('publish_zero_when_disabled').value)
        self.warn_period_ns = int(float(self.get_parameter('warn_period_s').value) * 1e9)

        if self.inference_rate_hz <= 0.0:
            raise ValueError('inference_rate_hz must be > 0')
        if len(self.joint_names) != 4:
            raise ValueError('joint_names must have length 4')
        if self.action_scale_xy.shape != (2,) or self.action_signs_xy.shape != (2,):
            raise ValueError('action_scale_xy and action_signs_xy must each have length 2')

        self.action_pub = self.create_publisher(Vector3, self.action_topic, 10)
        self.enabled_pub = self.create_publisher(Bool, self.enabled_topic, 10)
        self.processed_image_pub = self.create_publisher(Image, self.processed_image_topic, qos_profile_sensor_data)

        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 10)
        self.create_service(Trigger, self.start_service_name, self._start_cb)
        self.create_service(Trigger, self.stop_service_name, self._stop_cb)

        self._latest_image: Optional[np.ndarray] = None
        self._latest_image_stamp_ns: Optional[int] = None
        self._latest_joint_state: Optional[np.ndarray] = None
        self._latest_joint_stamp_ns: Optional[int] = None
        self._smoothed_action = np.zeros(2, dtype=np.float32)
        self._last_warn_ns = 0
        self._had_fresh_inputs = False
        self._policy_source = ''
        self._warmed_up = False

        self._load_policy()
        self.timer = self.create_timer(1.0 / self.inference_rate_hz, self._inference_step)
        self._publish_enabled()
        self.get_logger().info(
            f'policy_inference active. source={self._policy_source}, image_topic={self.image_topic}, action_topic={self.action_topic}, device={self.device}'
        )

    def _resolve_device(self, torch, config) -> object:
        preferred_device = self.device_override
        if preferred_device == 'auto':
            cfg_device = getattr(config, 'device', None)
            if cfg_device and cfg_device != 'auto':
                preferred_device = str(cfg_device)
            else:
                preferred_device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if preferred_device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warn('cuda requested but unavailable on this host; falling back to cpu')
            preferred_device = 'cpu'
        return torch.device(preferred_device)

    def _try_load_source(self, source: str, torch, DiffusionPolicy, make_pre_post_processors, PreTrainedConfig):
        load_config = PreTrainedConfig.from_pretrained(source)
        saved_cfg_device = getattr(load_config, 'device', None)
        load_config.device = 'cpu'
        policy = DiffusionPolicy.from_pretrained(source, config=load_config)
        if saved_cfg_device is not None:
            policy.config.device = saved_cfg_device
        device = self._resolve_device(torch, policy.config)
        if self.use_fp16 and device.type == 'cuda':
            policy.half()
        policy.to(device)
        policy.eval()
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy.config,
            pretrained_path=source,
            preprocessor_overrides={'device_processor': {'device': str(device)}},
        )
        policy.reset()
        return policy, device, preprocessor, postprocessor

    def _load_policy(self) -> None:
        try:
            import torch
            from lerobot.configs.policies import PreTrainedConfig
            from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
            from lerobot.policies.factory import make_pre_post_processors
            from lerobot.policies.utils import prepare_observation_for_inference
        except Exception as exc:
            raise RuntimeError(
                'Failed to import LeRobot diffusion inference dependencies. Run `just lerobot-install` in this repo first.'
            ) from exc

        self.torch = torch
        self.prepare_observation_for_inference = prepare_observation_for_inference
        errors: list[str] = []

        if self.policy_repo_id:
            try:
                self.policy, self.device, self.preprocessor, self.postprocessor = self._try_load_source(
                    self.policy_repo_id,
                    torch,
                    DiffusionPolicy,
                    make_pre_post_processors,
                    PreTrainedConfig,
                )
                self._policy_source = self.policy_repo_id
                return
            except Exception as exc:
                errors.append(f'hub load failed for {self.policy_repo_id}: {exc}')
                self.get_logger().warn(errors[-1])

        if self.local_policy_path.exists():
            local_source = str(self.local_policy_path)
            try:
                self.policy, self.device, self.preprocessor, self.postprocessor = self._try_load_source(
                    local_source,
                    torch,
                    DiffusionPolicy,
                    make_pre_post_processors,
                    PreTrainedConfig,
                )
                self._policy_source = local_source
                return
            except Exception as exc:
                errors.append(f'local load failed for {local_source}: {exc}')
                self.get_logger().warn(errors[-1])
        else:
            errors.append(f'local fallback path does not exist: {self.local_policy_path}')

        raise RuntimeError('unable to load diffusion policy from hub or local fallback: ' + ' | '.join(errors))

    def _warn(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns > self.warn_period_ns:
            self.get_logger().warn(message)
            self._last_warn_ns = now_ns

    def _publish_enabled(self) -> None:
        msg = Bool()
        msg.data = self.enabled
        self.enabled_pub.publish(msg)

    def _publish_action(self, action_xy: np.ndarray) -> None:
        msg = Vector3()
        msg.x = float(action_xy[0])
        msg.y = float(action_xy[1])
        msg.z = 0.0
        self.action_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._smoothed_action[:] = 0.0
        self._publish_action(self._smoothed_action)

    @staticmethod
    def _msg_to_array(msg: Image) -> np.ndarray:
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ('mono8', '8uc1'):
            row_stride = int(msg.step)
            image = data.reshape((msg.height, row_stride))[:, : msg.width]
            return np.ascontiguousarray(image)

        if enc in ('rgb8', 'bgr8', '8uc3'):
            row_stride = int(msg.step) // 3
            image = data.reshape((msg.height, row_stride, 3))[:, : msg.width, :]
            return np.ascontiguousarray(image)

        if enc in ('rgba8', 'bgra8', '8uc4'):
            row_stride = int(msg.step) // 4
            image = data.reshape((msg.height, row_stride, 4))[:, : msg.width, :]
            return np.ascontiguousarray(image)

        raise ValueError(f'unsupported image encoding: {msg.encoding}')

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
            if enc == 'rgba8':
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
            else:
                image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
        elif image.ndim == 3 and image.shape[2] == 3:
            if enc != 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        image = self._center_square_crop(image)
        if image.shape[0] != self.resize_height or image.shape[1] != self.resize_width:
            image = cv2.resize(image, (self.resize_width, self.resize_height), interpolation=cv2.INTER_AREA)
        if image.dtype != np.uint8:
            image = np.clip(image, 0, 255).astype(np.uint8)
        return image

    def _image_cb(self, msg: Image) -> None:
        try:
            image = self._msg_to_array(msg)
            image = self._prepare_image(image, msg.encoding)
        except Exception as exc:
            self._warn(f'failed to preprocess image: {exc}')
            return

        self._latest_image = image
        self._latest_image_stamp_ns = _stamp_to_ns(msg.header.stamp)
        processed_msg = Image()
        processed_msg.header.stamp = msg.header.stamp
        processed_msg.header.frame_id = msg.header.frame_id
        processed_msg.height = self.resize_height
        processed_msg.width = self.resize_width
        processed_msg.encoding = 'rgb8'
        processed_msg.is_bigendian = 0
        processed_msg.step = self.resize_width * 3
        processed_msg.data = image.tobytes()
        self.processed_image_pub.publish(processed_msg)

    def _joint_state_cb(self, msg: JointState) -> None:
        index_by_name = {name: idx for idx, name in enumerate(msg.name)}
        if any(name not in index_by_name for name in self.joint_names):
            missing = [name for name in self.joint_names if name not in index_by_name]
            self._warn(f'joint_states missing expected joints: {missing}')
            return
        positions = np.asarray([msg.position[index_by_name[name]] for name in self.joint_names], dtype=np.float32)
        self._latest_joint_state = positions
        self._latest_joint_stamp_ns = _stamp_to_ns(msg.header.stamp)

    def _inputs_fresh(self) -> bool:
        if self._latest_image is None or self._latest_joint_state is None:
            return False
        now_ns = self.get_clock().now().nanoseconds
        image_age = (now_ns - int(self._latest_image_stamp_ns or 0)) * 1e-9
        joint_age = (now_ns - int(self._latest_joint_stamp_ns or 0)) * 1e-9
        return image_age <= self.observation_timeout_s and joint_age <= self.observation_timeout_s

    def _reset_policy_state(self) -> None:
        self.policy.reset()
        self._smoothed_action[:] = 0.0

    def _maybe_warmup(self) -> None:
        if self._warmed_up:
            return
        if not self._inputs_fresh():
            return
        try:
            _ = self._policy_action()
            self._warmed_up = True
            self.get_logger().info('policy warmup complete')
        except Exception as exc:
            self._warn(f'policy warmup failed: {exc}')
            self._reset_policy_state()

    def _set_enabled(self, enabled: bool) -> None:
        if self.enabled == enabled:
            self._publish_enabled()
            return
        self.enabled = enabled
        if not self.enabled:
            self._reset_policy_state()
            if self.publish_zero_when_disabled:
                self._publish_zero()
        elif not self._warmed_up:
            self._reset_policy_state()
        self._publish_enabled()

    def _start_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._set_enabled(True)
        response.success = True
        response.message = 'policy inference enabled'
        return response

    def _stop_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._set_enabled(False)
        response.success = True
        response.message = 'policy inference disabled'
        return response

    def _policy_action(self) -> Optional[np.ndarray]:
        observation = {
            'observation.images.front': self._latest_image,
            'observation.state': self._latest_joint_state.astype(np.float32, copy=True),
        }
        batch = self.prepare_observation_for_inference(
            observation=observation,
            device=self.device,
            task=self.task_name,
            robot_type=self.robot_type,
        )
        batch = self.preprocessor(batch)
        if self.use_fp16 and self.device.type == 'cuda':
            for key, value in list(batch.items()):
                if isinstance(value, self.torch.Tensor) and self.torch.is_floating_point(value):
                    batch[key] = value.half()
            with self.torch.autocast(device_type='cuda', dtype=self.torch.float16):
                action = self.policy.select_action(batch)
        else:
            action = self.policy.select_action(batch)
        action = self.postprocessor(action)
        action_np = np.asarray(action.detach().cpu().numpy(), dtype=np.float32).reshape(-1)
        if action_np.size < 2:
            self._warn(f'policy returned too few action dimensions: {action_np.shape}')
            return None
        action_xy = action_np[:2] * self.action_signs_xy * self.action_scale_xy
        return _clamp_xy(action_xy, self.max_delta_speed)

    def _inference_step(self) -> None:
        self._publish_enabled()
        if not self.enabled:
            self._maybe_warmup()
            if self.publish_zero_when_disabled:
                self._publish_zero()
            return

        fresh = self._inputs_fresh()
        if not fresh:
            if self._had_fresh_inputs:
                self._reset_policy_state()
            self._had_fresh_inputs = False
            if self.publish_zero_when_disabled:
                self._publish_zero()
            self._warn('waiting for fresh policy observations (image + joint_states)')
            return

        self._had_fresh_inputs = True
        try:
            raw_action = self._policy_action()
        except Exception as exc:
            self._warn(f'policy inference failed: {exc}')
            self._reset_policy_state()
            if self.publish_zero_when_disabled:
                self._publish_zero()
            return

        if raw_action is None:
            if self.publish_zero_when_disabled:
                self._publish_zero()
            return

        alpha = min(max(self.smoothing_alpha, 0.0), 1.0)
        self._smoothed_action = (alpha * raw_action + (1.0 - alpha) * self._smoothed_action).astype(np.float32)
        self._smoothed_action = _clamp_xy(self._smoothed_action, self.max_delta_speed)
        self._publish_action(self._smoothed_action)


def main() -> None:
    rclpy.init()
    node = PolicyInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
