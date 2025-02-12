#!/usr/bin/python3

# from my_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

class Diff_Drive_Kinematic():
    def __init__(self, r : float, b : float):
        self.Tt2w = np.array([1.0 / r, -0.5 * b / r,
                              1.0 / r, 0.5 * b / r]).reshape(2, 2)
        self.Tw2t = np.array([0.5 * r, 0.5 * r,
                              0.0, 0.0, 
                              -r / b, r / b]).reshape(3, 2)
        self.pos_g = np.zeros(3)
        self.vel_g = np.zeros(3)

    def get_wheelspeed(self, vel_r : list):
        return self.Tt2w @ np.array(vel_r)
    
    def get_pose(self, qd : np, dt : float):
        Rr2g = np.array([np.cos(self.pos_g[2]), -np.sin(self.pos_g[2]), 0.0,
                         np.sin(self.pos_g[2]), np.cos(self.pos_g[2]), 0.0,
                         0.0, 0.0, 1.0]).reshape(3, 3)
        self.pos_g += (Rr2g @ self.Tw2t @ qd) * dt 
        self.pos_g[2] = np.arctan2(np.sin(self.pos_g[2]), np.cos(self.pos_g[2]))
        return self.pos_g

    def get_twist(self, qd : np):
        return np.linalg.inv(self.Tt2w) @ qd

class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')

        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.joint_subscription = self.create_subscription(JointState, "joint_states", self.jointstate_callback, 10)
        self.velocity_publisher = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.jointstate_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.kine = Diff_Drive_Kinematic(r=0.05, b=0.35)
        self.prev_time = None

        self.joint_names = ['caster_wheel_r_joint', 'caster_wheel_f_joint']
        self.joint_positions = [0.0, 0.0]  # Initial positions
        self.joint_velocities = [0.0, 0.0]  # Optional velocities
        self.timer = self.create_timer(0.01, self.publish_joint_states)  # 10 Hz

    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Populate joint state message
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities

        # Publish the message
        self.jointstate_publisher.publish(msg)


    def cmd_callback(self, msg : Twist):
        robot_speed = [msg.linear.x, msg.angular.z]
        wheel_speed = self.kine.get_wheelspeed(robot_speed)

        velo_msg = Float64MultiArray()
        velo_msg.data = list(map(float, wheel_speed))
        self.velocity_publisher.publish(velo_msg)

    def jointstate_callback(self, msg : JointState):
        if msg.name[0] == "left_wheel_joint" or msg.name[1] == "left_wheel_joint":
            wheel_speed = [msg.velocity[0], msg.velocity[1]]
            curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            pose = [0, 0, 0]
            twist = [0, 0]
            if self.prev_time is not None:
                dt = curr_time - self.prev_time
                pose = self.kine.get_pose(wheel_speed, dt)
                twist = self.kine.get_twist(wheel_speed)

            self.prev_time = curr_time

            quat = tf_transformations.quaternion_from_euler(0, 0, pose[2])

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"

            t.transform.translation.x = float(pose[0])
            t.transform.translation.y = float(pose[1])
            t.transform.translation.z = 0.0

            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            # ส่ง TF
            self.tf_broadcaster.sendTransform(t)

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            odom_msg.pose.pose.position.x = float(pose[0])
            odom_msg.pose.pose.position.y = float(pose[1])
            odom_msg.pose.pose.position.z = 0.0

            odom_msg.pose.pose.orientation = Quaternion(
                x=quat[0],
                y=quat[1],
                z=quat[2],
                w=quat[3],
            )

            odom_msg.twist.twist.linear.x = float(twist[0])
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = float(twist[1])

            self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
