# import os

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String

# # First import the library
# import pyrealsense2 as rs
# # Import Numpy for easy array manipulation
# import numpy as np
# # Import OpenCV for easy image rendering
# from PIL import Image
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import CameraInfo
# from rclpy.qos import QoSProfile, QoSDurabilityPolicy


# class GetCameraFrames(Node):

#     def __init__(self):
#         super().__init__('Get_camera_frames')
#         # self.subscription = self.create_subscription(
#         #     String,
#         #     'topic',
#         #     self.listener_callback,
#         #     10)
#         # self.subscription  # prevent unused variable warning
#         self.pipeline = rs.pipeline()

#         # Create a config and configure the pipeline to stream
#         #  different resolutions of color and depth streams
#         self.config = rs.config()

#         # Get device product line for setting a supporting resolution
#         self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
#         self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
#         self.device = self.pipeline_profile.get_device()
#         self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

#         found_rgb = False
#         for s in self.device.sensors:
#             if s.get_info(rs.camera_info.name) == 'RGB Camera':
#                 found_rgb = True
#                 break
#         if not found_rgb:
#             print("The demo requires Depth camera with Color sensor")
#             exit(0)

#         self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#         self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

#         # Start streaming
#         self.profile = self.pipeline.start(self.config)

#         # Getting the depth sensor's depth scale (see rs-align example for explanation)
#         self.depth_sensor = self.profile.get_device().first_depth_sensor()
#         self.depth_scale = self.depth_sensor.get_depth_scale()
#         print("Depth Scale is: " , self.depth_scale)

#         # We will be removing the background of objects more than
#         #  clipping_distance_in_meters meters away
#         self.clipping_distance_in_meters = 1 #1 meter
#         self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale

#         # Create an align object
#         # rs.align allows us to perform alignment of depth frames to others frames
#         # The "align_to" is the stream type to which we plan to align depth frames.
#         self.align_to = rs.stream.color
#         self.align = rs.align(self.align_to)

#         #config_package = 'intel_realsense_functions'  # <- this is the name of the original package holding the YAML
#         #package_share_dir = get_package_share_directory(config_package)
#         #yaml_file = os.path.join(package_share_dir, "config", "config.yaml") # change the yaml file for different robots

#         # Load YAML config
#         #with open(yaml_file, 'r') as f:
#         #    config_data = yaml.safe_load(f)
#         #self.serverURL = config_data.get('server_url')

        
#         postFrequency = 1.0  # seconds
#         self.color_pub = self.create_publisher(Image, '/realsense_rgb_image', 10)
#         self.depth_pub = self.create_publisher(Image, '/realsense_depth_image', 10)
#         self.bridge = CvBridge()
#         self.timer = self.create_timer(postFrequency, self.imagePubTimerCallback)
#         #Get and publish camera intrinsics
#         self.intrQOSProfile = QoSProfile(depth=1,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
#         self.camIntrPub = self.create_publisher(CameraInfo, '/depth_camera_intrinsics', self.intrQOSProfile)
#         self.videoProfile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
#         self.cameraIntr = self.videoProfile.get_intrinsics()
#         self.depthCamInfo = CameraInfo()
#         self.depthCamInfo.width = self.cameraIntr.width
#         self.depthCamInfo.height = self.cameraIntr.height
#         self.depthCamInfo.k = [self.cameraIntr.fx, 0.0, self.cameraIntr.ppx,
#                       0.0, self.cameraIntr.fy, self.cameraIntr.ppy,
#                       0.0, 0.0, 1.0]
#         self.depthCamInfo.p = [self.cameraIntr.fx, 0.0, self.cameraIntr.ppx, 0.0,
#                       0.0, self.cameraIntr.fy, self.cameraIntr.ppy, 0.0,
#                       0.0, 0.0, 1.0, 0.0]
#         self.depthCamInfo.distortion_model = 'none'

#         # Publish once (or use a timer to repeat)
#         self.intrTimer = self.create_timer(postFrequency, self.pubCamIntr)
#         #self.camIntrPub.publish(depthCamInfo)
#         self.get_logger().info("Published intrinsics")
    
#     def pubCamIntr(self):
#         self.camIntrPub.publish(self.depthCamInfo)

    
#     def getRGBDepthAlignedImages(self):
#         frames = self.pipeline.wait_for_frames()
#         alignedFrames = self.align.process(frames)
#         alignedDepth = alignedFrames.get_depth_frame()
#         alignedColor = alignedFrames.get_color_frame()
#         depthImage = np.asanyarray(alignedDepth.get_data()).T
#         depthImage = depthImage.astype(np.float32) * self.depth_scale
#         colorImage = np.asanyarray(alignedColor.get_data())
#         colorImage = colorImage[..., ::-1]
#         print("Got images")
#         return colorImage, depthImage
    
#     def imagePubTimerCallback(self):
#         colorImage, depthImage = self.getRGBDepthAlignedImages()

#         # Convert and publish color image
#         color_msg = self.bridge.cv2_to_imgmsg(colorImage, encoding='rgb8')
#         self.color_pub.publish(color_msg)

#         # Convert and publish depth image (ensure it is float32 or 16UC1)
#         depth_msg = self.bridge.cv2_to_imgmsg(depthImage, encoding='32FC1')
#         self.depth_pub.publish(depth_msg)
    
#     # def sendRGBDepthImagesToServer(self):
#     #     colorImage,depthImage = self.getRGBDepthAlignedImages()
#     #     imagesURL = self.serverURL + "/segmentrobotview"
#     #     rgb_io = io.BytesIO()
#     #     Image.fromarray(colorImage).save(rgb_io, format='JPEG')
#     #     rgb_io.seek(0)

#     #     depth_io = io.BytesIO()
#     #     np.save(depth_io, depthImage)
#     #     depth_io.seek(0)

#     #     # Construct POST request
#     #     files = {
#     #         'colorImage': ('rgb.png', rgb_io, 'image/png'),
#     #         'depthImage': ('depth.npy', depth_io, 'application/octet-stream')
#     #     }
#     #     response = requests.post(imagesURL, files=files)
        



# def main(args=None):
#     rclpy.init(args=args)

#     getCamFrames = GetCameraFrames()

#     rclpy.spin(getCamFrames)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     getCamFrames.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

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
        self.cam_info.distortion_model = 'none'

        # Timers
        self.create_timer(0.5, lambda: self.camIntrPub.publish(self.cam_info))
        self.create_timer(0.5, self.imagePubTimerCallback)

        self.consecutive_timeouts = 0

    def _start_pipeline(self):
        # (Re)start pipeline and related helpers
        self.profile = self.pipeline.start(self.config)

        # Reduce internal frame queues to avoid stale frames
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

        # Warmup (non-fatal if some time out)
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
        # Try poll loop to avoid long blocking timeouts
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
                # First soft restart, then hard reset if still bad
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

        depth_np = np.asanyarray(depth.get_data()).astype(np.float32) * self.depth_scale
        color_bgr = np.asanyarray(color.get_data())
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

        depth_np = np.ascontiguousarray(depth_np)
        color_rgb = np.ascontiguousarray(color_rgb)

        try:
            color_msg = self.bridge.cv2_to_imgmsg(color_rgb, encoding='rgb8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='32FC1')
        except CvBridgeError as e:
            self.get_logger().warn(f"CvBridge error: {e}")
            return

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
