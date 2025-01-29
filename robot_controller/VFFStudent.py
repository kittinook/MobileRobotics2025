import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
import numpy as np

class VFF_Avoidance(Node):
    def __init__(self):
        super().__init__('vff_avoidance')
        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        self.create_timer(0.05, self.vff_controller)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "marker_debug", 10)

        self.laser_scan = None
        self.RED, self.GREEN, self.BLUE = 0, 1, 2

    def laser_scan_callback(self, msg):
        self.laser_scan = msg

    def get_debug_vff(self, vff_vectors):
        marker_array = MarkerArray()
        marker_array.markers.append(self.make_marker(vff_vectors['attractive'], self.BLUE))
        marker_array.markers.append(self.make_marker(vff_vectors['repulsive'], self.RED))
        marker_array.markers.append(self.make_marker(vff_vectors['result'], self.GREEN))
        return marker_array

    def make_marker(self, vector, vff_color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=vector[0], y=vector[1], z=0.0)]
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        color = ColorRGBA(a=1.0)
        if vff_color == self.RED:
            marker.id, color.r = 0, 1.0
        elif vff_color == self.GREEN:
            marker.id, color.g = 1, 1.0
        elif vff_color == self.BLUE:
            marker.id, color.b = 2, 1.0
        marker.color = color
        return marker

    def vff_controller(self):
        if self.laser_scan is None:
            return
        
        vff_vectors = self.get_vff(self.laser_scan)
        resultant = vff_vectors['result']

        cmd_vel = Twist()
        cmd_vel.linear.x = max(0.0, min(0.3, resultant[0]))  
        cmd_vel.angular.z = max(-1.0, min(1.0, -resultant[1]))  
        
        self.vel_pub.publish(cmd_vel)
        self.marker_pub.publish(self.get_debug_vff(vff_vectors))

    def get_vff(self, scan):
        OBSTACLE_DISTANCE = 2.0
        REPULSIVE_GAIN = 1.5

        vff_vector = {'attractive': [0.5, 0.0], 'repulsive': [0.0, 0.0], 'result': [0.0, 0.0]}

        # Implement Here

        return vff_vector

def main(args=None):
    rclpy.init(args=args)
    node = VFF_Avoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
