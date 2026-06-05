import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

import tf2_ros 
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # params
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('depth_patch_size', 5)      # NxN median patch
        self.declare_parameter('dedup_distance_m', 0.5)    # meters
        self.declare_parameter('max_depth_m', 6.0)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('map_frame', 'map')

        self.conf_thresh   = self.get_parameter('confidence_threshold').value
        self.patch_half     = self.get_parameter('depth_patch_size').value // 2
        self.dedup_dist     = self.get_parameter('dedup_distance_m').value
        self.max_depth      = self.get_parameter('max_depth_m').value
        self.min_depth      = self.get_parameter('min_depth_m').value
        self.camera_frame   = self.get_parameter('camera_frame').value
        self.map_frame      = self.get_parameter('map_frame').value

        model_path = self.get_parameter('model_path').value
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded YOLO model: {model_path}')

        # ---- convert ros image to numpy array ----
        self.bridge = CvBridge()

        # --- initialize state values ----
        self.fx = self.fy = self.cx = self.cy = None          # intrinsics (cached)
        self.latest_depth       = None                         # latest depth numpy array
        self.known_objects       = []                           # [(class, (x,y,z))]
        self.next_marker_id     = 0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # subscribers
        self.create_subscription(CameraInfo, '/depth_camera_instrinsics', self._caminfo_cb, 10)
        self.create_subscription(Image, '/realsense_depth_image', self._depth_cb, 10)
        self.create_subscription(Image, '/realsense_rgb_image', self._rgb_cb, 10)

        # publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)
        self.annotated_pub = self.create_publisher(Image, '/yolo_annotated', 10)

        self.get_logger().info("YOLO detector node ready")

    def _caminfo_cb(self, msg: CameraInfo):
        # only cache cam info once
        if self.fx is not None:
            return

        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.get_logger().info(
            f'Intrinsics cached: fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f}')
        
    def _depth_cb(self, msg: Image):
        # Cache the latest aligned depth (32FC1, meters).
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')

    def _rgb_cb(self, msg: Image):
        # Main loop: detect → back-project → transform → publish.
        if self.fx is None or self.latest_depth is None:
            return
        
        rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        results = self.model(rgb, conf=self.conf_thresh, verbose=False)

        # Look up the camera → map transform once per frame
        R, t = self._lookup_transform(msg.header.stamp)
        if R is None:
            return
        
        markers = MarkerArray()

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cls_name = self.model.names[int(box.cls[0])]
                conf     = float(box.conf[0])
 
                # --- sample depth at bbox center (median of a small patch) ---
                u, v = (x1 + x2) // 2, (y1 + y2) // 2
                patch = self.latest_depth[
                    max(0, v - self.patch_half) : v + self.patch_half + 1,
                    max(0, u - self.patch_half) : u + self.patch_half + 1]
                valid = patch[np.isfinite(patch)
                              & (patch > self.min_depth)
                              & (patch < self.max_depth)]
                if valid.size == 0:
                    continue
                Z = float(np.median(valid))
 
                # --- back-project: pixel + depth → 3D in optical frame ---
                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy
 
                # --- rotate + translate into map frame ---
                p_map = R @ np.array([X, Y, Z]) + t
 
                # --- deduplicate: same class within dedup_distance → skip ---
                if self._is_duplicate(cls_name, p_map):
                    continue
                self.known_objects.append((cls_name, p_map.copy()))
 
                # --- build rviz markers (cube + floating label) ---
                m_cube, m_text = self._make_markers(
                    p_map, cls_name, conf, msg.header.stamp)
                markers.markers.extend([m_cube, m_text])
 
                self.get_logger().info(
                    f'{cls_name} ({conf:.0%}) at map '
                    f'({p_map[0]:.2f}, {p_map[1]:.2f}, {p_map[2]:.2f})')
 
        if markers.markers:
            self.marker_pub.publish(markers)
 
        # publish annotated image for debugging (view as Image in rviz)
        annotated = results[0].plot() if results else rgb
        ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
        ann_msg.header = msg.header
        self.annotated_pub.publish(ann_msg)
        
    def _lookup_transform(self, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.camera_frame,
                stamp, timeout=Duration(seconds=0.1)
            )
        except tf2_ros.TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.camera_frame,
                    Time(),                           # fallback to latest
                    timeout=Duration(seconds=0.1))
            except tf2_ros.TransformException as e:
                self.get_logger().warn(
                    f'TF {self.camera_frame}→{self.map_frame}: {e}',
                    throttle_duration_sec=2.0)
                return None, None
            
        q = tf.transform.rotation
        tr = tf.transform.translation
 
        # quaternion → rotation matrix
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
        ])
        t = np.array([tr.x, tr.y, tr.z])
        return R, t
    
    def _is_duplicate(self, cls_name, p):
        """True if cls_name was already detected within dedup_distance."""
        for known_cls, known_pt in self.known_objects:
            if known_cls == cls_name:
                if np.linalg.norm(p - known_pt) < self.dedup_dist:
                    return True
        return False

    def _make_markers(self, p, cls_name, conf, stamp):
        """Create a colored cube and a floating text label."""
        cube = Marker()
        cube.header.frame_id = self.map_frame
        cube.header.stamp     = stamp
        cube.ns    = 'yolo_detections'
        cube.id    = self.next_marker_id; self.next_marker_id += 1
        cube.type  = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position.x = float(p[0])
        cube.pose.position.y = float(p[1])
        cube.pose.position.z = float(p[2])
        cube.pose.orientation.w = 1.0
        cube.scale.x = cube.scale.y = cube.scale.z = 0.2
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.1, 0.9, 0.2, 0.85
        cube.lifetime = Duration(seconds=0).to_msg()   # 0 = persist forever
 
        label = Marker()
        label.header = cube.header
        label.ns     = 'yolo_labels'
        label.id     = self.next_marker_id; self.next_marker_id += 1
        label.type   = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(p[0])
        label.pose.position.y = float(p[1])
        label.pose.position.z = float(p[2]) + 0.3      # float above the cube
        label.scale.z = 0.15
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f'{cls_name} ({conf:.0%})'
        label.lifetime = Duration(seconds=0).to_msg()
 
        return cube, label
 
 
def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()