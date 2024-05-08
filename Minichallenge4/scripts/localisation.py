#!/usr/bin/env python3 

import rospy    
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32 
from tf.transformations import quaternion_from_euler 
import numpy as np 
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_msgs.msg import TFMessage
 
#This class will do the following: 
#   subscribe to /wr and /wl
#   publish to the /odom topic

class OdomClass():  
    def __init__(self):  
        # first thing, init a node! 
        rospy.init_node('localisation') 
        ###******* INIT PUBLISHERS *******###  
        # Create the subscriber to cmd_vel topic 
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 
        # Create ROS publishers 
        self.odom_pub = rospy.Publisher('odom', Odometry ,queue_size=1) #Publisher to pose_sim topic 

        t = TransformStamped() 

        ############ ROBOT CONSTANTS ################  
        self.r = 0.05 #puzzlebot wheel radius [m] 
        self.L = 0.19 #puzzlebot wheel separation [m] 
        self.dt = 0.02 # Desired time to update the robot's pose [s] 
        ############ Variables ############### 
        self.w = 0.0 # robot's angular speed [rad/s] 
        self.v = 0.0 #robot's linear speed [m/s] 
        self.wr = 0.0
        self.wl = 0.0
        self.x = 0.0 # 
        self.x_ant = 0.0
        self.y = 0.0  
        self.y_ant = 0.0
        self.theta = 0.0  
        self.theta_ant = 0.0

        self.odom = Odometry()


        self.mu = np.array([0.0, 0.0, 0.0]) #X Y Theta

        self.Q = np.array([[0.0002, 0.0001, 0.0001], [0.0001, 0.0002, 0.0001],[0.0001, 0.0001, 0.0002]])
        
        self.sigma = np.eye(3)

                
        rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while loop will be the inverse of the desired delta_t 
        while not rospy.is_shutdown(): 

            self.get_robot_vel() 

            self.H = np.array([[1.0, 0.0, -(self.dt * self.v * np.sin(self.theta))], 
                               [0.0, 1.0, self.dt * self.v * np.cos(self.theta)], 
                               [0.0, 0.0, 1.0]])

            self.mu = np.array([[self.mu[0] + (self.v * self.dt * np.cos(self.mu[2]))],
                                [self.mu[1] + (self.v * self.dt * np.sin(self.mu[2]))],
                                [self.mu[2] + (self.dt * self.w)]])
            
            self.sigma = self.H.dot(self.sigma).dot(self.H.T) + self.Q

            self.update_robot_pose()
            self.get_pose_odometry(self.theta, self.sigma)

            rate.sleep() 

     
    def wl_cb(self, msg): 
        self.wl = msg.data

    def wr_cb(self, msg): 
        self.wr = msg.data
     
    def get_robot_vel(self): 
        self.v = self.r * ((self.wr + self.wl) / 2.0)
        self.w = self.r * ((self.wr - self.wl) / self.L)

    def get_pose_odometry(self, yaw, Sigma): 
        # Write the data as a ROS PoseStamped message 
        self.odom.header.frame_id = "odom"    #This can be changed in this case I'm using a frame called odom. 
        self.odom.child_frame_id = "base_link"
        self.odom.header.stamp = rospy.Time.now() 
        # Position 
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y


        # Rotation of the mobile base frame w.r.t. "odom" frame as a quaternion 
        quat = quaternion_from_euler(0, 0, yaw) 
        self.odom.pose.pose.orientation.x = quat[0] 
        self.odom.pose.pose.orientation.y = quat[1] 
        self.odom.pose.pose.orientation.z = quat[2] 
        self.odom.pose.pose.orientation.w = quat[3] 

        # Init a 36 elements array 
        self.odom.pose.covariance = [0.0]*36 
        self.odom.pose.covariance[0] = Sigma[0][0] * Sigma[0][0]#variance in x
        self.odom.pose.covariance[1] = Sigma[0][0] * Sigma[1][1]
        self.odom.pose.covariance[5] = Sigma[0][0] * Sigma[2][2]
        self.odom.pose.covariance[6] = Sigma[1][1] * Sigma[0][0]
        self.odom.pose.covariance[7] = Sigma[1][1] * Sigma[1][1]#variance in y
        self.odom.pose.covariance[11] = Sigma[1][1] * Sigma[2][2]
        self.odom.pose.covariance[30] = Sigma[2][2] * Sigma[0][0]
        self.odom.pose.covariance[31] = Sigma[2][2] * Sigma[1][1]
        self.odom.pose.covariance[35] = Sigma[2][2] * Sigma[2][2] #variance in theta

        # Fill the speed information 
        self.odom.twist.twist.linear.x = self.v 
        self.odom.twist.twist.angular.z = self.w 

        self.odom_pub.publish(self.odom)

    def update_robot_pose(self): 
        #This functions receives the robot speed v [m/s] and w [rad/s] 
        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad] 
        # is the orientation,     
        ############ MODIFY THIS CODE   ################ 
        self.x = self.x_ant + self.v * np.cos(self.theta) * self.dt
        self.y = self.y_ant + self.v * np.sin(self.theta) * self.dt
        self.theta = self.theta_ant + self.w * self.dt

        self.x_ant = self.x
        self.y_ant = self.y
        self.theta_ant = self.theta
        


 
############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    OdomClass()  