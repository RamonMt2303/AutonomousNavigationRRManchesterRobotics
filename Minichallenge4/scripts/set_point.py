#!/usr/bin/env python3
import rospy
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist  
from geometry_msgs.msg import Pose

#This class will subscribe to the /message topic and publish to the /cmd_vel topic 

class path_generator():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) #This function will be called before killing the node. 
        #########PUBLISHERS AND SUBSCRIBERS ################# 
        self.goal_pub = rospy.Publisher('set_point', Pose, queue_size=1)  
        self.pub_cmd_vel = rospy.Publisher('puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)  
        rospy.Subscriber("flag", Float32, self.flag_cb) 

        ############ CONSTANTS AND VARIABLES ################  
        self.flag = 0
        self.data = [[1.5, 1.2], [-1.15, 1.5], [0.0, -2.5], [0.0,-2.5]]
        self.i = 0
        self.goal = Pose()
        self.cmd_vel_pub = Twist()
        self.r = rospy.Rate(50) #20 Hz 
        
        self.init_time = rospy.get_time()

        self.cmd_vel_pub.linear.x = 0.2
        self.cmd_vel_pub.angular.z = 0.0
        self.pub_cmd_vel.publish(self.cmd_vel_pub)
        rospy.sleep(1)

   
        while not rospy.is_shutdown():
            self.goal.position.x = self.data[self.flag][0]
            self.goal.position.y = self.data[self.flag][1]

            self.goal_pub.publish(self.goal)
            rospy.sleep(1)

    def flag_cb(self, msg):
        self.flag = msg.data
     
    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.
        zero = Pose()    
        self.goal_pub.publish(zero) #publish the message 
        print("I'm dying, bye bye!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("Set_point_generator", anonymous=True)  
    path_generator() 


