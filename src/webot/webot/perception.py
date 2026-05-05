import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError
import cv2

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()
        
        # Declare ROS2 Parameters for easy live-tuning
        self.declare_parameter('min_area', 500)
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('epsilon_mult', 0.015)

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)
        
        self.pub = self.create_publisher(Point, '/perception/box_data', 10)
        # Add a debug publisher to see what the robot sees
        self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 10)
        
        self.get_logger().info("Perception Node Initialized.")

    def is_cube_silhouette(self, cnt, epsilon_mult):
        area = cv2.contourArea(cnt)
        if area == 0: return False

        perimeter = cv2.arcLength(cnt, True)
        epsilon = epsilon_mult * perimeter
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        vertices = len(approx)

        if vertices < 4 or vertices > 6:
            return False

        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = float(area) / hull_area if hull_area > 0 else 0
        if solidity < 0.90: 
            return False

        circularity = (4 * 3.14159 * area) / (perimeter * perimeter)
        if circularity > 0.85: # If it's too 'round', it's probably a cylinder
            return False

        if vertices == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            if aspect_ratio < 0.8 or aspect_ratio > 1.2:
                return False

        return True

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Fetch live parameters
        min_area = self.get_parameter('min_area').value
        canny_low = self.get_parameter('canny_low').value
        canny_high = self.get_parameter('canny_high').value
        eps_mult = self.get_parameter('epsilon_mult').value

        height, width, _ = cv_image.shape
        img_center = width / 2

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, canny_low, canny_high)
        
        # Changed to RETR_TREE so we can eventually analyze internal lines
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        best_box = None
        max_area = 0

        # Optional: Only look at top-level parent contours for the silhouette check
        for i, cnt in enumerate(contours):
            # Check if contour has no parent (it's an outer silhouette)
            if hierarchy[0][i][3] == -1: 
                area = cv2.contourArea(cnt)
                if area > min_area and self.is_cube_silhouette(cnt, eps_mult):
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
                
                # Draw on debug image
                cv2.drawContours(cv_image, [best_box], -1, (0, 255, 0), 3)
                cv2.circle(cv_image, (cx, int(M['m01'] / M['m00'])), 5, (0, 0, 255), -1)
        else:
            box_msg.y = 0.0

        self.pub.publish(box_msg)
        
        # Publish the debug image so you can view it in rviz2 or rqt_image_view
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.debug_pub.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")