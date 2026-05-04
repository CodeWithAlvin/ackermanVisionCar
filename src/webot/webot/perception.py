import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(Point, '/perception/box_data', 10)
        self.get_logger().info("Advanced Perception Node Initialized.")

    def is_cube(self, cnt):
        """Advanced geometrical filter to reject cylinders and spheres."""
        area = cv2.contourArea(cnt)
        if area < 500: 
            return False 

        # 1. Strict Polygon Approximation
        perimeter = cv2.arcLength(cnt, True)
        epsilon = 0.015 * perimeter
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        vertices = len(approx)

        if vertices not in [4, 5, 6]:
            return False

        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0: return False
        solidity = area / hull_area
        if solidity < 0.95: 
            return False # Reject if not highly solid

        if vertices == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            if aspect_ratio < 0.7 or aspect_ratio > 1.3:
                return False 

        return True 

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape
        img_center = width / 2

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_box = None
        max_area = 0

        for cnt in contours:
            if self.is_cube(cnt):
                area = cv2.contourArea(cnt)
                if area > max_area:
                    max_area = area
                    best_box = cnt 

        box_msg = Point()
        
        if best_box is not None:
            M = cv2.moments(best_box)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                _, _, w, _ = cv2.boundingRect(best_box)
                
                box_msg.x = float(img_center - cx) 
                box_msg.y = 1.0  
                box_msg.z = float(w / width) 
                
                cv2.drawContours(cv_image, [best_box], -1, (0, 255, 0), 3)
        else:
            box_msg.y = 0.0 

        self.pub.publish(box_msg)
        