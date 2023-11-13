#! /usr/bin/env python3 
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

def create_pose_stamped(navigator, pos_x, pos_y, rot_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = pos_x
    pose.pose.position.y = pos_y
    pose.pose.position.z = pos_x
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    point_1 = create_pose_stamped(nav, 1.03, 0.0, 0.0)
    point_2 = create_pose_stamped(nav, 3.61, 0.04, 0.0)
    point_3 = create_pose_stamped(nav, 2.56, 2.34, 0.0)
    point_4 = create_pose_stamped(nav, 1.74, 1.08, 0.0)
    origin = create_pose_stamped(nav, 0, 0, 0.0)
    waypoints = [point_1, point_2, point_3, point_4, origin]

    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        print(nav.getFeedback())

    rclpy.shutdown()


if __name__ == '__main__':
    main()