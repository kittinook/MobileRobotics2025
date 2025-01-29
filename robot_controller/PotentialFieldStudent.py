import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class PotentialFieldNavNode(Node):
    def __init__(self):
        super().__init__('potential_field_node')

        # Create a publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to Odometry to get the robot's current position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Subscribe to LaserScan to detect obstacles around the robot
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Declare parameters and retrieve values from the Parameter Server
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 1.0)
        self.declare_parameter('d_safe', 0.6)  # Safe distance from obstacles
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('max_turn_speed', 1.0)
        self.declare_parameter('min_goal_dist', 0.3)

        # Retrieve parameters
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.k_att = self.get_parameter('k_att').value
        self.k_rep = self.get_parameter('k_rep').value
        self.d_safe = self.get_parameter('d_safe').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn_speed = self.get_parameter('max_turn_speed').value
        self.min_goal_dist = self.get_parameter('min_goal_dist').value

        # Variables for storing data
        self.robot_pos = np.array([0.0, 0.0])
        self.robot_yaw = 0.0
        self.laser_ranges = None

        self.get_logger().info("Potential Field Node started. Waiting for data...")

    def odom_callback(self, msg: Odometry):
        """
        Callback function for Odometry.
        Extracts the robot's (x, y) position and yaw orientation.
        """
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Convert quaternion to yaw angle
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.robot_pos = np.array([position.x, position.y])
        self.robot_yaw = yaw

    def scan_callback(self, msg: LaserScan):
        """
        Callback function for LaserScan.
        Computes attractive and repulsive forces based on detected obstacles
        and publishes velocity commands.
        """
        if self.robot_pos is None:
            return  # Wait for Odometry data

        # Store LaserScan data
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # Check if the robot has reached the goal
        dist_to_goal = np.linalg.norm(self.robot_pos - np.array([self.goal_x, self.goal_y]))
        if dist_to_goal < self.min_goal_dist:
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            return

        # Compute Attractive Force
        F_att = self.compute_attractive_force(self.robot_pos, np.array([self.goal_x, self.goal_y]), self.k_att)

        # Compute Repulsive Force from obstacles
        F_rep = ____

        # Compute the total force by summing attractive and repulsive forces
        F_total = ____

        # Convert force vector into linear and angular velocity
        linear_vel, angular_vel = self.force_to_cmd_vel(F_total)

        # Limit velocity to avoid exceeding max values
        linear_vel = max(min(linear_vel, self.max_speed), -self.max_speed)
        angular_vel = max(min(angular_vel, self.max_turn_speed), -self.max_turn_speed)

        # Publish velocity command
        twist = Twist()
        twist.linear.x = float(linear_vel)
        twist.angular.z = float(angular_vel)
        self.cmd_pub.publish(twist)

    def compute_attractive_force(self, robot_pos, goal_pos, k_att):
        """
        Compute the attractive force towards the goal.
        Formula: F_att = -k_att * (robot_pos - goal_pos)
        """
        diff = robot_pos - goal_pos
        return ____

    def compute_repulsive_force(self, robot_pos, ranges, angle_min, angle_inc, k_rep, d_safe):
        """
        Compute the repulsive force from detected obstacles.
        """
        F_rep_total = ____

        for i in range(len(ranges)):
            dist = ranges[i]
            if math.isinf(dist):
                continue
            if dist < d_safe:
                rep_magnitude = ____
                unit_vec_rob2obs = ____
                F_rep_local = ____
                F_rep_total += F_rep_local

        return F_rep_total

    def force_to_cmd_vel(self, F_total):
        """
        Convert force vector (global frame) to linear and angular velocity.
        """
        fx, fy = F_total
        f_mag = np.linalg.norm(F_total)
        desired_heading = math.atan2(fy, fx)
        heading_error = ____
        linear_vel = ____
        angular_vel = ____
        return linear_vel, angular_vel

    def stop_robot(self):
        """
        Stops the robot by publishing zero velocity.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

main()