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
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, theta)
        self.pose.header.frame_id = 'map'
        self.pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = x
        self.pose.pose.orientation.x = q_x
        self.pose.pose.orientation.y = q_y
        self.pose.pose.orientation.z = q_z
        self.pose.pose.orientation.w = q_w
        return self.pose

def main():
        rclpy.init()
        controller = Navigation()
        position_1 = controller.create_pose(controller, 2.5, 1.0, 1.57)
        position_2 = controller.create_pose(controller, 0.0, 1.0, 1.57)
        position_3 = controller.create_pose(controller, 0.0, 0.0, 0.0)
        origin = controller.create_pose(controller, 0.0, 0.0, 0.0)
        waypoints = [position_1, position_2, position_3, origin]

        controller.followWaypoints(waypoints)
        while not controller.isTaskComplete():
            print(controller.getFeedback())

        rclpy.shutdown()

if __name__ == "__main__":
    main()
