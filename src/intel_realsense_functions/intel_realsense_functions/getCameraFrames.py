#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class GetCameraFrames(Node):
    def __init__(self):
        super().__init__('Get_camera_frames')

        self.declare_parameters('', [
            ('depth_min', 0.3), ('depth_max', 3.5),
            ('spatial_magnitude', 2), ('spatial_alpha', 0.5),
            ('spatial_delta', 20), ('temporal_enabled', False),
            ('hole_filling_enabled', False),
        ])
        self.depth_min = self.get_parameter('depth_min').value
        self.depth_max = self.get_parameter('depth_max').value
        if not 0.0 <= self.depth_min < self.depth_max:
            raise ValueError('Require 0 <= depth_min < depth_max')
        self.temporal_enabled = self.get_parameter('temporal_enabled').value
        self.hole_filling_enabled = self.get_parameter('hole_filling_enabled').value

        # ---- CONFIG (use lower bandwidth defaults) ----
        self.color_w, self.color_h, self.color_fps = 640, 480, 15
        self.depth_w, self.depth_h, self.depth_fps = 640, 480, 15
        self.max_poll_ms = 500        # per-try timeout
        self.poll_tries = 6            # total ~3s per timer tick
        self.restart_after_timeouts = 5

        # ---- PIPELINE SETUP ----
        self.ctx = rs.context()
        if len(self.ctx.devices) == 0:
            raise RuntimeError("No RealSense device found")

        self.pipeline = rs.pipeline(self.ctx)
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.depth_w, self.depth_h, rs.format.z16, self.depth_fps)
        self.config.enable_stream(rs.stream.color, self.color_w, self.color_h, rs.format.bgr8, self.color_fps)

        self._start_pipeline()

        # ---- ROS PUBS ----
        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(ImageMsg, '/realsense_rgb_image', 10)
        self.depth_pub = self.create_publisher(ImageMsg, '/realsense_depth_image', 10)

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.camIntrPub = self.create_publisher(CameraInfo, '/depth_camera_intrinsics', qos)

        # Intrinsics (color stream)
        vprof = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = vprof.get_intrinsics()
        self.cam_info = CameraInfo()
        self.cam_info.width = intr.width
        self.cam_info.height = intr.height
        self.cam_info.k = [intr.fx, 0.0, intr.ppx,
                           0.0, intr.fy, intr.ppy,
                           0.0, 0.0, 1.0]
        self.cam_info.p = [intr.fx, 0.0, intr.ppx, 0.0,
                           0.0, intr.fy, intr.ppy, 0.0,
                           0.0, 0.0, 1.0, 0.0]
        self.cam_info.r = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]
        self.cam_info.d = [0.0]*5
        self.cam_info.distortion_model = 'plumb_bob'
        self.cam_info.header.frame_id = 'camera_color_optical_frame'

        # Timers
        # self.create_timer(0.5, lambda: self.camIntrPub.publish(self.cam_info))
        self.create_timer(0.067, self.imagePubTimerCallback)

        self.consecutive_timeouts = 0

    def _start_pipeline(self):
        self.profile = self.pipeline.start(self.config)

        dev = self.profile.get_device()
        self._warned_timestamp_domain = False
        for s in dev.sensors:
            if s.supports(rs.option.global_time_enabled):
                try:
                    s.set_option(rs.option.global_time_enabled, 1)
                except RuntimeError as exc:
                    self.get_logger().warn(f'Cannot enable RealSense global time: {exc}')
            if s.supports(rs.option.frames_queue_size):
                try:
                    s.set_option(rs.option.frames_queue_size, 1)
                except Exception:
                    pass

        self.depth_sensor = dev.first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)

        # D435 stereo smoothing operates in disparity space. Temporal history
        # and inferred hole depths are disabled by default on a moving robot.
        self.to_disparity = rs.disparity_transform(True)
        self.to_depth = rs.disparity_transform(False)
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, self.get_parameter('spatial_magnitude').value)
        self.spatial.set_option(rs.option.filter_smooth_alpha, self.get_parameter('spatial_alpha').value)
        self.spatial.set_option(rs.option.filter_smooth_delta, self.get_parameter('spatial_delta').value)
        self.temporal = rs.temporal_filter()
        self.hole_fill = rs.hole_filling_filter()
        self.hole_fill.set_option(rs.option.holes_fill, 1)

        for _ in range(10):
            try:
                fs = self.pipeline.wait_for_frames(200)
                if fs: break
            except Exception:
                pass
        self.get_logger().info(f"Depth scale: {self.depth_scale:.6f}. Pipeline started.")

    def _stop_pipeline(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass

    def _restart_pipeline(self, do_hw_reset=False):
        self.get_logger().warn("Restarting RealSense pipeline...")
        self._stop_pipeline()
        if do_hw_reset:
            try:
                self.profile.get_device().hardware_reset()
            except Exception:
                pass
        self._start_pipeline()
        self.consecutive_timeouts = 0

    def _poll_frames(self):
        for _ in range(self.poll_tries):
            try:
                fs = self.pipeline.wait_for_frames(self.max_poll_ms)
            except Exception:
                fs = None
            if fs:
                return fs
        return None

    def imagePubTimerCallback(self):
        frames = self._poll_frames()
        if frames is None:
            self.consecutive_timeouts += 1
            self.get_logger().warn(f"No frames ({self.consecutive_timeouts} consecutive timeouts).")
            if self.consecutive_timeouts >= self.restart_after_timeouts:
                hard = (self.consecutive_timeouts >= 2*self.restart_after_timeouts)
                self._restart_pipeline(do_hw_reset=hard)
            return

        self.consecutive_timeouts = 0
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if not depth or not color:
            self.get_logger().warn("Got invalid frames (None). Skipping this tick.")
            return

        # Stamp acquisition, before filtering/alignment adds variable latency.
        # Preserve each stream's capture time; RGB-D uses approximate sync.
        color_stamp = self._capture_stamp(color)
        depth_stamp = self._capture_stamp(depth)

        # Filter native stereo depth before aligning it into color coordinates.
        filtered = self.to_disparity.process(frames)
        filtered = self.spatial.process(filtered)
        if self.temporal_enabled:
            filtered = self.temporal.process(filtered)
        filtered = self.to_depth.process(filtered)
        if self.hole_filling_enabled:
            filtered = self.hole_fill.process(filtered)
        aligned = self.align.process(filtered)
        depth = aligned.get_depth_frame()
        color = aligned.get_color_frame()
        if not depth or not color:
            return

        depth_np = np.asanyarray(depth.get_data()).astype(np.float32) * self.depth_scale
        depth_np[(~np.isfinite(depth_np)) | (depth_np < self.depth_min) |
                 (depth_np > self.depth_max)] = 0.0
        color_bgr = np.asanyarray(color.get_data())
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

        depth_np = np.ascontiguousarray(depth_np)
        color_rgb = np.ascontiguousarray(color_rgb)

        color_msg = self.bridge.cv2_to_imgmsg(color_rgb, encoding='rgb8')
        color_msg.header.stamp = color_stamp
        color_msg.header.frame_id = 'camera_color_optical_frame'
        
        depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='32FC1')
        depth_msg.header.stamp = depth_stamp
        depth_msg.header.frame_id = 'camera_color_optical_frame'
        
        # Geometry is the color camera's, but this CameraInfo also accompanies
        # aligned depth for depthimage_to_laserscan's exact camera subscriber.
        self.cam_info.header.stamp = depth_stamp
        self.camIntrPub.publish(self.cam_info)

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        

    def _capture_stamp(self, frame):
        clock_now = self.get_clock().now()
        domain = frame.get_frame_timestamp_domain()
        if domain in (rs.timestamp_domain.system_time, rs.timestamp_domain.global_time):
            # SDK timestamps are milliseconds on the host system clock. Apply
            # frame age to ROS time so a different ROS clock epoch is respected.
            age_ns = int((rs.get_time() - frame.get_timestamp()) * 1_000_000)
            return Time(nanoseconds=max(0, clock_now.nanoseconds - max(0, age_ns)),
                        clock_type=clock_now.clock_type).to_msg()
        if not self._warned_timestamp_domain:
            self.get_logger().warn(
                'RealSense has no host-synchronized capture timestamp; using '
                'receipt time before processing. Check global-time support '
                'before diagnosing motion distortion.')
            self._warned_timestamp_domain = True
        return clock_now.to_msg()

    def destroy_node(self):
        self._stop_pipeline()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GetCameraFrames()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
