#!/usr/bin/env python3 

import rospy  
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32 
from tf.transformations import quaternion_from_euler 
import numpy as np 

#This class will do the following: 

# Suscribe to /wr and /wr
# Publish to the /odom topic

class OdomClass():  
    def __init__(self):  
        rospy.init_node('localisation') 
        ###******* INIT PUBLISHERS *******###  
        # Create the subscriber to cmd_vel topic 
        rospy.Subscriber("puzzlebot_1/wl", Float32 , self.wl_cb) 
        rospy.Subscriber("puzzlebot_1/wr", Float32 , self.wr_cb) 
        # Create ROS publishers 
        self.odom_pub = rospy.Publisher('odom', Odometry ,queue_size=1) #Publisher to pose_sim topic 
        ############ ROBOT CONSTANTS ################  
        self.r=0.05 #puzzlebot wheel radius [m] 
        self.L = 0.19 #puzzlebot wheel separation [m] 
        self.dt = 0.02 # Desired time to update the robot's pose [s] 
        ############ Variables ############### 
        self.w = 0.0 # robot's angular speed [rad/s] 
        self.v = 0.0 #robot's linear speed [m/s] 
        #Mapa1 y 2
        self.x = 3.0 # initial position along x
        self.y = 3.0  # initial position along y
        self.theta = -(np.pi/2) # initial position of theta

        '''#Mapa3 y 4
        self.x = 2.0 # initial position along x
        self.y = 2.0  # initial position along y
        self.theta = -(np.pi/2) # initial position of theta'''

        self.wr = 0.0
        self.wl = 0.0

        #Mensaje de Odometry
        self.odom = Odometry() 
        rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while loop will be the inverse of the desired delta_t 
        while not rospy.is_shutdown(): 
            ####### Add / modify  your code here ############# 
            [v, w] = self.get_robot_vel(self.wr,self.wl) 
            self.update_robot_pose(v, w) 
            self.get_odometry()
            rate.sleep() 

    def wl_cb(self, msg): 
        self.wl = msg.data 
        
    def wr_cb(self, msg): 
        self.wr = msg.data 
        
    def get_robot_vel(self, wr, wl): 
        #rospy.logwarn("get_wheel_speeds: This function must be modified by you") 
        #poner aca las ecuaciones
        #v = 0.0 # Left wheel angular speed in [rad/s] 
        #w =  0.0 # Right wheel angular speed in [rad/s]
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L
        return [v, w] 

    def get_odometry(self): 
        odom_quat = quaternion_from_euler(0, 0, self.theta)
        self.update_robot_pose(self.v,self.w)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        # Publish the message
        self.odom_pub.publish(odom)

        # Reset time and encoder readings
        #self.last_time = rospy.Time.now()
        self.wr = 0
        self.wl = 0

    def update_robot_pose(self, v, w): 
        #This functions receives the robot speed v [m/s] and w [rad/s] 
        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad] 
        # is the orientation,     
        ############ MODIFY THIS CODE   ################ 
        #rospy.logwarn("set_robot_pose: Make sure to modify this function") 
        #aqui las integrales
        #self.x = 0.0 
        #self.y = 0.0 
        #self.theta = 0.0 

        self.x = self.x + v*np.cos(self.theta)*self.dt
        self.y = self.y + v*np.sin(self.theta)*self.dt
        self.theta = self.theta + w*self.dt


############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    OdomClass()  