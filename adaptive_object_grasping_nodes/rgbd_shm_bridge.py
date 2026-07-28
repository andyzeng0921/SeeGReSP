import importlib
import sys
import threading
import types

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class RgbdShmBridge(Node):
    """Publish the Autolife RealSense shared-memory stream as standard ROS topics."""

    def __init__(self):
        super().__init__('rgbd_shm_bridge')
        self.declare_parameter(
            'vendor_python_path',
            '/home/ubuntu/miniconda3/envs/robot_env/lib/python3.12/site-packages',
        )
        self.declare_parameter('color_topic', '/head_camera/color/image_raw')
        self.declare_parameter('depth_topic', '/head_camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/head_camera/color/camera_info')
        self.declare_parameter('optical_frame', 'rgbd_head_color_optical_frame')
        self.declare_parameter('publish_rate', 60.0)
        self.declare_parameter('retry_period', 2.0)
        self.declare_parameter('maximum_frame_id_delta', 2)

        self._color_consumer = None
        self._depth_consumer = None
        self._intrinsics = None
        self._pending_color = None
        self._pending_depth = None
        self._lock = threading.Lock()
        self._last_error = ''
        self._published_frames = 0

        self._color_pub = self.create_publisher(
            Image, str(self.get_parameter('color_topic').value), qos_profile_sensor_data
        )
        self._depth_pub = self.create_publisher(
            Image, str(self.get_parameter('depth_topic').value), qos_profile_sensor_data
        )
        self._info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            qos_profile_sensor_data,
        )
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._poll)
        self.create_timer(float(self.get_parameter('retry_period').value), self._ensure_open)
        self._ensure_open()

    @staticmethod
    def _stream_config(stream):
        suffix = f'rgbd_head_{stream}'
        protocol_options = {
            'data_shm_name': f'/camera_image_buffer_{suffix}',
            'meta_shm_name': f'/camera_metadata_struct_{suffix}',
            'read_mutex_name': f'/read_mutex_lock_{suffix}',
            'write_lock_name': f'/write_data_lock_{suffix}',
            'writer_queue_name': f'/writer_queue_sem_{suffix}',
            'intrinsics_shm_name': f'/camera_intrinsics_struct_{suffix}',
        }
        return {
            'selected_protocol': 'v1',
            'protocols': {
                'v1': protocol_options,
            },
        }

    def _ensure_open(self):
        if self._color_consumer is not None and self._depth_consumer is not None:
            return
        vendor_path = str(self.get_parameter('vendor_python_path').value)
        if vendor_path and vendor_path not in sys.path:
            sys.path.insert(0, vendor_path)
        try:
            # Import camera_shm without executing autolife_robot_sdk.utils.__init__.
            # That initializer eagerly imports the audio/VAD stack, whose torchaudio
            # build is incompatible with the project venv's torch. The camera SHM
            # extension itself has no audio or torch dependency.
            import autolife_robot_sdk

            utils_name = 'autolife_robot_sdk.utils'
            camera_module_name = f'{utils_name}.camera_shm'
            if camera_module_name not in sys.modules:
                utils_package = types.ModuleType(utils_name)
                utils_package.__package__ = utils_name
                utils_package.__path__ = [
                    f'{vendor_path}/autolife_robot_sdk/utils',
                ]
                sys.modules[utils_name] = utils_package
                setattr(autolife_robot_sdk, 'utils', utils_package)
            camera_shm = importlib.import_module(camera_module_name)
            SHMCameraFrameConsumer = camera_shm.SHMCameraFrameConsumer

            color = SHMCameraFrameConsumer(self._stream_config('color'), name='rgbd_color')
            depth = SHMCameraFrameConsumer(self._stream_config('depth'), name='rgbd_depth')
            intrinsics = color.get_intrinsics()
            if not intrinsics:
                color.close()
                depth.close()
                raise RuntimeError('color intrinsics are unavailable')
            with self._lock:
                self._color_consumer = color
                self._depth_consumer = depth
                self._intrinsics = intrinsics
                self._last_error = ''
            self.get_logger().info(
                'RGB-D SHM opened: aligned depth, '
                f'fx={intrinsics["fx"]:.3f}, fy={intrinsics["fy"]:.3f}'
            )
        except Exception as exc:
            message = str(exc)
            if message != self._last_error:
                self.get_logger().warning(f'waiting for RGB-D shared memory: {message}')
                self._last_error = message

    def _poll(self):
        with self._lock:
            color_consumer = self._color_consumer
            depth_consumer = self._depth_consumer
        if color_consumer is None or depth_consumer is None:
            return
        try:
            color = color_consumer.get_latest(nonblock=True)
            depth = depth_consumer.get_latest(nonblock=True)
            if color is not None:
                self._pending_color = color
            if depth is not None:
                self._pending_depth = depth
            if self._pending_color is None or self._pending_depth is None:
                return
            color_frame, color_id = self._pending_color
            depth_frame, depth_id = self._pending_depth
            maximum_delta = max(0, int(self.get_parameter('maximum_frame_id_delta').value))
            if abs(color_id - depth_id) > maximum_delta:
                if color_id < depth_id:
                    self._pending_color = None
                else:
                    self._pending_depth = None
                return
            self._publish_pair(color_frame, depth_frame)
            self._published_frames += 1
            if self._published_frames == 1:
                self.get_logger().info(
                    f'published first aligned RGB-D pair '
                    f'({color_frame.shape[1]}x{color_frame.shape[0]}, '
                    f'frame delta={color_id - depth_id})'
                )
            self._pending_color = None
            self._pending_depth = None
        except Exception as exc:
            self.get_logger().error(f'RGB-D SHM read failed: {exc}')
            self._close_consumers()

    def _publish_pair(self, color, depth):
        if color.ndim != 3 or color.shape[2] != 3:
            raise ValueError(f'unexpected color shape {color.shape}')
        if depth.shape != color.shape[:2] or depth.dtype != np.uint16:
            raise ValueError(f'unexpected aligned depth {depth.shape}/{depth.dtype}')
        stamp = self.get_clock().now().to_msg()
        frame = str(self.get_parameter('optical_frame').value)

        color_msg = self._image_message(color, 'bgr8', stamp, frame)
        depth_msg = self._image_message(depth, '16UC1', stamp, frame)
        info_msg = self._camera_info(color.shape[1], color.shape[0], stamp, frame)
        self._color_pub.publish(color_msg)
        self._depth_pub.publish(depth_msg)
        self._info_pub.publish(info_msg)

    @staticmethod
    def _image_message(array, encoding, stamp, frame):
        contiguous = np.ascontiguousarray(array)
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.height = int(contiguous.shape[0])
        message.width = int(contiguous.shape[1])
        message.encoding = encoding
        message.is_bigendian = False
        message.step = int(contiguous.strides[0])
        message.data = contiguous.tobytes()
        return message

    def _camera_info(self, width, height, stamp, frame):
        intr = self._intrinsics
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.width = int(width)
        message.height = int(height)
        message.distortion_model = 'plumb_bob'
        message.d = [float(value) for value in intr.get('coeffs', [])[:5]]
        while len(message.d) < 5:
            message.d.append(0.0)
        fx, fy = float(intr['fx']), float(intr['fy'])
        cx, cy = float(intr['ppx']), float(intr['ppy'])
        message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return message

    def _close_consumers(self):
        with self._lock:
            consumers = (self._color_consumer, self._depth_consumer)
            self._color_consumer = None
            self._depth_consumer = None
        for consumer in consumers:
            if consumer is not None:
                try:
                    consumer.close()
                except Exception:
                    pass

    def destroy_node(self):
        self._close_consumers()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RgbdShmBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
