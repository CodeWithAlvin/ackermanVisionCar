import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TwistStamped

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # --- TUNED PARAMETERS ---
        self.target_width_ratio = 0.4  
        self.linear_speed = 0.4       
        
        # Control Gains
        self.turn_kp = 0.0010   
        self.turn_kd = 0.0050  
        self.deadzone = 10.0    
        
        self.last_error_x = 0.0 
        
        # --- TOLERANCE ---
        self.missed_frames = 0
        self.max_missed_frames = 20  
        self.last_steer_cmd = 0.0    
        
        self.sub = self.create_subscription(
            Point, '/perception/box_data', self.control_callback, 10)
            
        self.pub = self.create_publisher(
            TwistStamped, '/ackermann_steering_controller/reference', 10)
            
        self.get_logger().info("PD Control Node Initialized (With Dropout Tolerance).")

    def control_callback(self, msg):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link' 
        
        target_found = bool(msg.y)
        raw_error_x = msg.x
        box_width_ratio = msg.z

        if target_found:
            self.missed_frames = 0
            
            if box_width_ratio > self.target_width_ratio:
                # Target Reached
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.get_logger().info("Target Reached Stopping.", throttle_duration_sec=1.0)
            else:
                if abs(raw_error_x) < self.deadzone:
                    error_x = 0.0
                else:
                    error_x = raw_error_x

                delta_error = error_x - self.last_error_x
                steering_angle = (error_x * self.turn_kp) + (delta_error * self.turn_kd)
                
                self.last_error_x = error_x
                self.last_steer_cmd = steering_angle 

                cmd.twist.linear.x = self.linear_speed
                cmd.twist.angular.z = steering_angle 
                
                self.get_logger().info(
                    f"Approaching Error: {raw_error_x:.0f} | Steer: {steering_angle:.3f}", 
                    throttle_duration_sec=0.5)
        else:
            self.missed_frames += 1
            
            if self.missed_frames < self.max_missed_frames:
                cmd.twist.linear.x = self.linear_speed * 0.7
                cmd.twist.angular.z = self.last_steer_cmd
                self.get_logger().info("Target lost Coasting.", throttle_duration_sec=0.5)
            else:
                self.last_error_x = 0.0 
                cmd.twist.linear.x = 0.15
                cmd.twist.angular.z = 0.5 
                self.get_logger().info("Searching for box", throttle_duration_sec=1.0)

        self.pub.publish(cmd)