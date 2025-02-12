import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.pure_pursuit_control)
        
        x_values = np.linspace(0, 5, num=50)
        self.waypoints = [(x, math.sin(2 * math.pi * x / 5)) for x in x_values]
        
        self.current_index = 0
        self.lookahead_distance = 0.02  # ระยะมองไปข้างหน้า
        self.position = (0.0, 0.0)
        self.yaw = 0.0

    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def pure_pursuit_control(self):
        if self.current_index >= len(self.waypoints):
            self.current_index = 0  # Reset index to loop the path
        # Implement Here
        goal_x, goal_y = self.waypoints[self.current_index]
        dx = goal_x - self.position[0]
        dy = goal_y - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.lookahead_distance:
            self.current_index += 1
            return
        
        goal_theta = math.atan2(dy, dx)
        angle_diff = goal_theta - self.yaw
        # angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        cmd_vel = Twist()
        cmd_vel.linear.x = min(0.2, distance)  # จำกัดความเร็ว
        cmd_vel.angular.z = 2.0 * angle_diff  # ควบคุมการเลี้ยว
        
        self.publisher.publish(cmd_vel)
        self.get_logger().info(f"Moving to waypoint {self.current_index + 1}: ({goal_x}, {goal_y})")


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
