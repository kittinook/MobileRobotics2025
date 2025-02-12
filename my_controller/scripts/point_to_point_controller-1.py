#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math


class PointToPointController(Node):
    def __init__(self):
        super().__init__('point_to_point_controller')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize variables
        self.current_pose = None
        self.goal_pose = None
        self.control_rate = 0.1  # Control frequency in seconds
        self.heading_aligned = False  # Flag for heading alignment

        # Timer for controlling the robot
        self.timer = self.create_timer(self.control_rate, self.control_loop)

    def goal_callback(self, msg: PoseStamped):
        """Callback to update the goal position."""
        self.goal_pose = msg.pose
        self.heading_aligned = False  # Reset heading alignment flag
        self.get_logger().info(f"New goal received: x={self.goal_pose.position.x}, y={self.goal_pose.position.y}")

    def odom_callback(self, msg: Odometry):
        """Callback to update the robot's current pose."""
        self.current_pose = msg.pose.pose

    def control_loop(self):
        """Main control loop to move the robot towards the goal."""
        if self.goal_pose is None or self.current_pose is None:
            # If either goal or current pose is missing, do nothing
            return

        # Extract current pose
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # Extract goal pose
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        goal_yaw = self.get_yaw_from_quaternion(self.goal_pose.orientation)

        # Compute distance and angle to goal
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # Angle error
        angle_error = angle_to_goal - current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize angle to [-pi, pi]

        # Thresholds for stopping
        distance_threshold = 0.05  # 5 cm
        angle_threshold = 0.1  # 0.1 radians (~5.7 degrees)

        # Control logic
        twist = Twist()
        if distance_to_goal > distance_threshold:
            # Rotate towards the goal if angle error is significant
            if abs(angle_error) > angle_threshold:
                twist.angular.z = 0.5 * angle_error  # Proportional control for angular velocity
                twist.linear.x = 0.0
            else:
                twist.linear.x = 0.2  # Proportional control for linear velocity
                twist.angular.z = 0.5 * angle_error  # Adjust heading slightly while moving forward
        else:
            # Align heading to goal's yaw
            if not self.heading_aligned:
                heading_error = goal_yaw - current_yaw
                heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # Normalize angle to [-pi, pi]
                
                if abs(heading_error) > angle_threshold:
                    twist.angular.z = 0.5 * heading_error  # Proportional control for angular velocity
                    twist.linear.x = 0.0
                else:
                    # Stop once heading is aligned
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.heading_aligned = True
                    self.get_logger().info("Goal reached and heading aligned!")
            else:
                # Stop if both position and heading are aligned
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        # Publish velocity command
        self.cmd_vel_pub.publish(twist)

    @staticmethod
    def get_yaw_from_quaternion(quaternion):
        """Convert quaternion to yaw (rotation around z-axis)."""
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = PointToPointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
