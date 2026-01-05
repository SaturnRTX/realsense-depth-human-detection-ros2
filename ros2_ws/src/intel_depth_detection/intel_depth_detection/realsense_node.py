import rclpy
import pyrealsense2 as rs
import numpy as np
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from cv_bridge import CvBridge

from intel_depth_detection.msg import HumanDetection
from intel_depth_detection.yolo_detector import YoloDetector


class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolo_node')

        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(Image, '/intel_depth/image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/intel_depth/markers', 10)
        self.det_pub = self.create_publisher(HumanDetection, '/intel_depth/human_detection', 10)

        self.detector = YoloDetector()

        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(cfg)

        self.align = rs.align(rs.stream.color)
        self.timer = self.create_timer(0.1, self.loop)
        self.id_counter = 0

    def loop(self):
        frames = self.align.process(self.pipeline.wait_for_frames())
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if not depth or not color:
            return

        image = np.asanyarray(color.get_data())
        detections = self.detector.detect(image)

        marker_array = MarkerArray()

        for i, det in enumerate(detections):
            cx = (det['x_min'] + det['x_max']) // 2
            cy = (det['y_min'] + det['y_max']) // 2
            distance = depth.get_distance(cx, cy)

            # Publish detection message
            det_msg = HumanDetection()
            det_msg.id = self.id_counter
            det_msg.confidence = det['confidence']
            det_msg.distance_m = distance
            det_msg.x_min = det['x_min']
            det_msg.y_min = det['y_min']
            det_msg.x_max = det['x_max']
            det_msg.y_max = det['y_max']
            det_msg.header.stamp = self.get_clock().now().to_msg()
            det_msg.header.frame_id = 'camera_color_frame'
            self.det_pub.publish(det_msg)

            # Bounding box marker
            box = Marker()
            box.header = det_msg.header
            box.ns = 'boxes'
            box.id = i
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = distance
            box.pose.position.y = 0.0
            box.pose.position.z = 0.0
            box.scale.x = 0.5
            box.scale.y = 0.5
            box.scale.z = 0.5
            box.color.r = 0.0
            box.color.g = 1.0
            box.color.b = 0.0
            box.color.a = 0.6
            marker_array.markers.append(box)

            # Text marker
            text = Marker()
            text.header = det_msg.header
            text.ns = 'labels'
            text.id = i + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.pose.position.x = distance
            text.pose.position.z = 0.8
            text.scale.z = 0.3
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"{distance:.2f} m"
            marker_array.markers.append(text)

            self.id_counter += 1

        self.marker_pub.publish(marker_array)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='bgr8'))


def main():
    rclpy.init()
    node = RealSenseYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
