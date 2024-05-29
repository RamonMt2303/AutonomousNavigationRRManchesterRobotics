#!/usr/bin/env python 
import rospy 
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import numpy as np

class GoToGoal():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber("scan", LaserScan, self.laser_cb)
        rospy.Subscriber(gtg vel)
        rospy.Subscriber()


def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        vel_msg = Twist()
        self.pub_cmd_vel.publish(vel_msg)