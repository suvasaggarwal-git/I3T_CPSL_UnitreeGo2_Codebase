#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class GetCameraFrames(Node):
    def __init__(self):
        super().__init__('Get_camera_frames')

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
        for s in dev.sensors:
            if s.supports(rs.option.frames_queue_size):
                try:
                    s.set_option(rs.option.frames_queue_size, 1)
                except Exception:
                    pass

        self.depth_sensor = dev.first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)

        # Depth post-processing filters (applied in order each frame).
        # Spatial: fills holes and smooths edges using neighboring pixels.
        # Temporal: reduces per-pixel noise by blending with previous frames.
        # Hole-filling: fills remaining invalid pixels with nearby valid depth.
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 5)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 1.0)
        self.spatial.set_option(rs.option.filter_smooth_delta, 50)
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
        aligned = self.align.process(frames)
        depth = aligned.get_depth_frame()
        color = aligned.get_color_frame()
        if not depth or not color:
            self.get_logger().warn("Got invalid frames (None). Skipping this tick.")
            return

        depth = self.spatial.process(depth)
        depth = self.temporal.process(depth)
        depth = self.hole_fill.process(depth)

        depth_np = np.asanyarray(depth.get_data()).astype(np.float32) * self.depth_scale
        color_bgr = np.asanyarray(color.get_data())
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

        depth_np = np.ascontiguousarray(depth_np)
        color_rgb = np.ascontiguousarray(color_rgb)

        now = self.get_clock().now().to_msg()

        color_msg = self.bridge.cv2_to_imgmsg(color_rgb, encoding='rgb8')
        color_msg.header.stamp = now
        color_msg.header.frame_id = 'camera_color_optical_frame'
        
        depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='32FC1')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_color_optical_frame'
        
        self.cam_info.header.stamp = now
        self.camIntrPub.publish(self.cam_info)   # publish cam_info here, same stamp

        '''try:
            color_msg = self.bridge.cv2_to_imgmsg(color_rgb, encoding='rgb8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='32FC1')
        except CvBridgeError as e:
            self.get_logger().warn(f"CvBridge error: {e}")
            return'''

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        

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
