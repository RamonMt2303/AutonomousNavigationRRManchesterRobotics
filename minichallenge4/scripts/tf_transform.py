#!/usr/bin/env python3 

import rospy    
from nav_msgs.msg import Odometry
import numpy as np 
from geometry_msgs.msg import TransformStamped
import tf2_ros
 
class TfBroadcaster():  
    def __init__(self):  
        # first thing, init a node! 
        rospy.init_node('Coordinate_Transform') 
        ###******* INIT PUBLISHERS *******###  
        # Create the subscriber to cmd_vel topic 
        rospy.Subscriber("odom", Odometry, self.odom_cb) 

        self.x = 0.0
        self.y = 0.0
        self.orientation = [0.0, 0.0, 0.0, 0.0]
        self.dt = 0.02

        tf_br = tf2_ros.TransformBroadcaster()
                
        rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while loop will be the inverse of the desired delta_t 

        while not rospy.is_shutdown(): 

            #Declarar la transformación
            t = TransformStamped()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.header.stamp = rospy.Time.now()
            t.transform.translation.x = self.x 
            t.transform.translation.y = self.y 
            t.transform.translation.z = 0.0

            t.transform.rotation.x = self.orientation[0]
            t.transform.rotation.y = self.orientation[1]
            t.transform.rotation.z = self.orientation[2]
            t.transform.rotation.w = self.orientation[3]

            #Broadcast de la transformación
            tf_br.sendTransform(t)

            rate.sleep() 

            


    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

 
############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    TfBroadcaster()  