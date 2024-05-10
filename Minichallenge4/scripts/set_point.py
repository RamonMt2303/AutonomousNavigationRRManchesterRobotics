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
        rospy.Subscriber("flag", Float32, self.flag_cb) 

        ############ CONSTANTS AND VARIABLES ################  
        self.flag = 0
        self.data = [[0,0], [2,0], [2,2], [0,2], [0,0]]
        self.i = 0
        self.goal = Pose()
        self.r = rospy.Rate(20) #20 Hz 
        
        self.init_time = rospy.get_time()
   
        while not rospy.is_shutdown():
            if self.flag == 1:
                self.goal.position.x = self.data[self.i][0] if (self.i<len(self.data)) else 0
                self.goal.position.y = self.data[self.i][1] if (self.i<len(self.data)) else 0
                self.i = self.i + 1 if (self.i<len(self.data)) else self.i
                self.flag = 0

            self.goal_pub.publish(self.goal)
            self.r.sleep()

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


