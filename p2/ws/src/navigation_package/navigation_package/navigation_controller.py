#! /usr/bin/env python3 
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from math import pi

class Navigation:
    def __init__(self):
        self.nav = BasicNavigator()
        self.pose = PoseStamped()

    def create_pose(self, x, y, theta):
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, 0.0)
        self.pose.header.frame_id = 'map'
        self.pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = theta
        self.pose.pose.orientation.x = q_x
        self.pose.pose.orientation.y = q_y
        self.pose.pose.orientation.z = q_z
        self.pose.pose.orientation.w = q_w

        self.nav.waitUntilNav2Active()
        self.nav.goToPose(self.pose)
        while not self.nav.isTaskComplete():
            print(self.nav.getFeedback())

def main():
        rclpy.init()
        controller = Navigation()
        controller.create_pose(2.5, 0.7, 0.0)
        controller.create_pose(1.46, 2.5, 0.0)
        controller.create_pose(3.6, -1.0, 0.0)
        controller.create_pose(0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
