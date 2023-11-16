#! /usr/bin/env python3 
import re, os, time
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

class Bot:
    def __init__(self):
        self.nav = Navigation()
        self._intents = {
            r'\b(?:[Vv]á)?\s?(?:(?:[pP]ara)\s)?(?:[oO]\s)?(?:[lL]ab)\b': "coordinate_1",
            r'\b(?:[Vv]á)?\s?(?:(?:[pP]ara)\s)?(?:[aA]\s)?(?:[sS]ecretaria)\s(?:[aA]cademica)\b': "coordinate_2",
            r'\b(?:[Vv]á)?\s?(?:(?:[pP]ara)\s)?(?:[oO]\s)?(?:[aA]telie)\s(?:[dD]e)\s(?:[eE]ng\s[cC]omp)\b': "coordinate_3",
            r'\b(?:[Vv]á)?\s?(?:(?:[pP]ara)\s)?(?:[aA]\s)?(?:[oO]rigem)\b': "origin",
            r'\b(?:ola|oi|hi)\b': 'greetings'
        }
        self._actions = {
            "coordinate_1": self.coordinate_1,
            "coordinate_2": self.coordinate_2,
            "coordinate_3": self.coordinate_3,
            "origin": self.origin,
            "greetings": self.greetings
        }

    def greetings(self, message):
        return print("Fala, Nicola, tudo bem? Pra onde você quer ir?")

    def coordinate_1(self, message):
        return self.nav.create_pose(2.5, 0.7, 0.0)

    def coordinate_2(self, message):
        return self.nav.create_pose(1.46, 2.5, 0.0)

    def coordinate_3(self, message):
        return self.nav.create_pose(3.6, -1.0, 0.0)

    def origin(self, message):
        return self.nav.create_pose(0.0, 0.0, 0.0)

    def execute_action(self, message):
        for key, value in self._intents.items():
            pattern = re.compile(key, re.IGNORECASE)
            groups = pattern.findall(message)
            if groups:
                print(f"{self._actions[value](groups[0])}", end=" ")
    
    def get_input(self):
        while True:
            message = input("Eu sou um ChatBot para controler o TurtleBot, fala comigo: ")
            self.execute_action(message)
            time.sleep(3)
            os.system('clear')

def main():
    rclpy.init()
    bot = Bot()
    bot.get_input()
    
if __name__ == '__main__':
    main()
