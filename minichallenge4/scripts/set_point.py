#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np

class GoalChecker:
    def __init__(self):
        rospy.init_node('goal_checker')

        self.xg = 0.0
        self.yg = 0.0
        self.xr = 0.0
        self.yr = 0.0
        self.tolerance = 0.05

        rospy.Subscriber("set_point", Pose, self.set_point_cb)
        rospy.Subscriber("puzzlebot_1/base_controller/odom", Odometry, self.odom_cb)
        self.pub_goal_status = rospy.Publisher("goal_status", String, queue_size=1)

        rospy.spin()

    def set_point_cb(self, msg):
        self.xg = msg.position.x
        self.yg = msg.position.y

    def odom_cb(self, msg):
        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y
        self.check_goal()

    def check_goal(self):
        distance = np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2)
        if distance < self.tolerance:
            self.pub_goal_status.publish("Objetivo alcanzado")

if __name__ == "__main__":
    try:
        GoalChecker()
    except rospy.ROSInterruptException:
        pass
