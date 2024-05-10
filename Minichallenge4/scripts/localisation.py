#!/usr/bin/env python3 

import rospy    
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32 
from tf.transformations import quaternion_from_euler 
from geometry_msgs.msg import PoseArray, Pose 
import numpy as np 
from geometry_msgs.msg import TransformStamped
 
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
        self.odom_pub = rospy.Publisher('odom', Odometry ,queue_size=1) #Publisher to odom topic 
        self.pose_array_pub = rospy.Publisher("pose_array_topic", PoseArray, queue_size=1) #Publisher to pose array topic 

        t = TransformStamped() 

        # Create a PoseArray message 
        self.pose_array_msg = PoseArray() 

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

        self.Xmedida = [2.16, 2.16, 2.175, 2.16, 2.16, 2.165, 2.15, 2.16, 2.16, 2.175,
                    2.16, 2.15, 2.18, 2.16, 2.17, 2.165, 2.16, 2.155, 2.15, 2.17,
                    2.13, 2.13, 2.18, 2.11, 2.11, 2.14, 2.13, 2.12, 2.13, 2.13,
                    2.09, 2.08, 2.11, 2.09, 2.06, 2.15, 2.1, 2.11, 2.12, 2.06]
        
        ##Valores de la posicion obtenidos en y 
        self.Ymedida = [-0.07, -0.04, -0.01, -0.06, -0.07, -0.05, -0.04, -0.095,
                    -0.01, -0.01, -0.01, -0.085, 0.05, 0.065, 0.08, 0.025,
                    0.02, -0.01, -0.035, 0.07, 0.01, 0.025, 0.038, 0.062,
                    0.052, 0.065, 0.09, 0.025, 0.09, 0.05, 0.055, 0.01,
                    0.07, 0.08, 0.05, 0.012, 0.05, 0.04, 0.012, 0.025]

        self.odom = Odometry()

        #Gains for wl and wr
        self.kl = 0.3
        self.kr = 0.25

        self.sigma_k = np.array([[0.0, 0.0], [0.0, 0.0]])

        self.mu = np.array([[0.0], [0.0], [0.0]]) #X Y Theta
        
        self.sigma = np.zeros((3,3))

        self.get_pose_array()

        # Set the header information (frame ID and timestamp) 
        self.pose_array_msg.header.frame_id = 'odom' 
        self.pose_array_msg.header.stamp = rospy.Time.now() 
                
        rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while loop will be the inverse of the desired delta_t 
        while not rospy.is_shutdown(): 

            self.get_robot_vel() 
            self.update_robot_pose()
            self.get_pose_odometry(self.theta, self.sigma)

            self.sigma_k[0][0] = self.kr * np.abs(self.wr)
            self.sigma_k[1][1] = self.kl * np.abs(self.wl)

            #self.sigma_k = np.array([[self.kr * np.abs(self.wr), 0],
                                     #[0, self.kl * np.abs(self.wl)]])

            print("Sigma K: " + str(self.sigma_k))

            self.gradient_w = 0.5 * self.r * self.dt * (np.array([[np.cos(self.theta_ant), np.cos(self.theta_ant)], 
                                                                  [np.sin(self.theta_ant), np.sin(self.theta_ant)], 
                                                                  [2.0/self.L, -2.0/self.L]]))


            #self.Q = self.gradient_w.dot(self.sigma_k).dot(self.gradient_w.T)
            self.Q = self.gradient_w @ self.sigma_k @ self.gradient_w.T


            self.H = np.array([[1.0, 0.0, -(self.dt * self.v * np.sin(self.theta_ant))], 
                               [0.0, 1.0, self.dt * self.v * np.cos(self.theta_ant)], 
                               [0.0, 0.0, 1.0]])

            self.mu = np.array([[self.x + (self.v * self.dt * np.cos(self.theta))],
                                [self.y + (self.v * self.dt * np.sin(self.theta))],
                                [self.theta + (self.dt * self.w)]])
             
            
            self.sigma = self.H.dot(self.sigma).dot(self.H.T) + self.Q

            print("Sigma: " + str(self.sigma))

            #self.update_robot_pose()
            #self.get_pose_odometry(self.theta, self.sigma)

            self.pose_array_pub.publish(self.pose_array_msg)

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
        odom_array = np.array([[Sigma[0][0], Sigma[0][1], 0, 0, 0, Sigma[0][2]],
                                              [Sigma[1][0], Sigma[1][1], 0.0, 0, 0, Sigma[1][2]],
                                              [0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 0, 0],
                                              [Sigma[2][0], Sigma[2][1], 0, 0, 0, Sigma[2][2]]
        ])

        self.odom.pose.covariance = odom_array.flatten().tolist()

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
        
    def get_pose_array(self):
        for i in range(len(self.Xmedida)):
            pose = Pose()
            pose.position.x = self.Xmedida[i]
            pose.position.y = self.Ymedida[i]
            pose.position.z = 0

            theta = 0
            quat = quaternion_from_euler(0.0, 0.0, theta) 
            pose.orientation.x = quat[0] 
            pose.orientation.y = quat[1] 
            pose.orientation.z = quat[2] 
            pose.orientation.w = quat[3] 
            pose.orientation.w = 1.0 
            self.pose_array_msg.poses.append(pose) 


 
############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    OdomClass()  