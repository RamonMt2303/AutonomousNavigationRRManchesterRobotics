#!/usr/bin/env python

import rospy  
from std_msgs.msg import Float32, Int32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32MultiArray 
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np 
from numpy.linalg import inv
import tf2_ros

class Localisation():  
    def __init__(self):
        rospy.init_node('localisation') 
        
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 
        rospy.Subscriber("aruco_topic", Float32MultiArray, self.aruco_cb)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) 

        ################# ROBOT CONSTANTS #################
        self.r = 0.05  
        self.L = 0.19
        self.dt = 0.02
        
        ################# MOVEMENT VARIABLES ##############
        self.x = 2.9
        self.y = 0.3
        self.theta = np.pi / 2

        self.kr = 20.0
        self.kl = 30.0

        self.w = 0.0
        self.v = 0.0
        self.wr = 0.0
        self.wl = 0.0

        ################# ARUCO POSITIONS #################
        self.x_aruco = 0.0
        self.y_aruco = 0.0
        self.id_aruco = 0
        self.get_aruco = False
        self.coords_aruco = np.array([0.0, 0.0])

        ################# EKF CONDITIONS ##################
        self.miu = np.array([self.x, self.y, self.theta])
        self.miu_hat = np.array([0, 0, 0])
        
        self.sigma = np.zeros((3, 3))
        self.sigma_hat = np.zeros((3, 3))

        self.Z = np.zeros((2, 2))
        self.z_hat = np.zeros((2, 2))

        self.H = np.zeros((3, 3))
        self.G = np.zeros((3, 3))
        self.gradient_W = np.zeros((2, 3))

        self.Q = np.zeros((3, 3))
        self.K = np.array((3, 2))
        self.covariance = np.zeros((2, 3))

        rate = rospy.Rate(int(1.0/self.dt))

        while not rospy.is_shutdown():

            [self.v, self.w] = self.get_robot_vel(self.wr, self.wl)

            self.covariance = np.array([[self.kl * np.abs(self.wr), 0],
                                        [0, self.kl * np.abs(self.wl)]])
            
            self.gradient_W = 0.5 * self.r * self.dt * np.array([[np.cos(self.theta), np.cos(self.theta)],
                                                                 [np.sin(self.theta), np.sin(self.theta)],
                                                                 [2.0 / self.L, -2.0 / self.L]])
            
            self.Q = self.gradient_W.dot(self.covariance).dot(self.gradient_W.T)

            self.prediccion()

            if self.get_aruco == True:
                self.correction()
                self.get_aruco = False
            else:
                self.propagate()

            odom_msg = self.get_odom_stamped()
            self.send_transform(odom_msg)
            self.odom_pub.publish(odom_msg)
            rate.sleep()

    def wl_cb(self, msg): 
        self.wl = msg.data

    def wr_cb(self, msg): 
        self.wr = msg.data       

    def get_robot_vel(self, wr, wl): 
        v = ((wl + wr) / 2) * self.r 
        w = ((wr - wl) / self.L) * self.r
        return [v, w] 
    
    def aruco_cb(self, msg):
        self.id_aruco = msg.data[0]
        self.d_aruco = msg.data[1]
        self.theta_aruco = msg.data[2]
        self.coords_aruco = [self.d_aruco, self.theta_aruco]
        
        self.id_arucos(self.id_aruco)
        self.get_aruco = True

    def id_arucos(self, id):
        x_y = {702: [0, 0.80], 701: [0, 1.60], 703: [1.73, 0.80],
               704: [2.63, 0.39], 705: [2.85, 0], 706: [2.865,2.0],
               707: [1.735, 1.22]}
        
        self.x_aruco = x_y[id][0]
        self.y_aruco = x_y[id][1]

        #self.coords_aruco = [self.x_aruco, self.y_aruco]

        #print("X aruco: " + str(self.x_aruco))
        #print("Y aruco: " + str(self.y_aruco))

    def get_odom_stamped(self): 

        odom_stamped = Odometry() 
        odom_stamped.header.frame_id = "odom" 
        odom_stamped.child_frame_id = "base_link"
        odom_stamped.header.stamp = rospy.Time.now() 
        odom_stamped.pose.pose.position.x = self.x
        odom_stamped.pose.pose.position.y = self.y

        quat = quaternion_from_euler(0, 0, self.theta) 
        odom_stamped.pose.pose.orientation.x = quat[0]
        odom_stamped.pose.pose.orientation.y = quat[1]
        odom_stamped.pose.pose.orientation.z = quat[2]
        odom_stamped.pose.pose.orientation.w = quat[3]

        odom_stamped.pose.covariance[0] = self.sigma[0][0]
        odom_stamped.pose.covariance[1] = self.sigma[0][1]
        odom_stamped.pose.covariance[5] = self.sigma[0][2]
        odom_stamped.pose.covariance[6] = self.sigma[1][0]
        odom_stamped.pose.covariance[7] = self.sigma[1][1]
        odom_stamped.pose.covariance[11] = self.sigma[1][2]
        odom_stamped.pose.covariance[30] = self.sigma[2][0]
        odom_stamped.pose.covariance[31] = self.sigma[2][1]
        odom_stamped.pose.covariance[35] = self.sigma[2][2]

        odom_stamped.twist.twist.linear.x = self.v
        odom_stamped.twist.twist.angular.z = self.w
        
        return odom_stamped
    
    def send_transform(self, odom):
        self.tf_send = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.frame_id = "odom" 
        t.child_frame_id = "base_link"
        t.header.stamp = rospy.Time.now() 
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0

        t.transform.rotation.x = odom.pose.pose.orientation.x
        t.transform.rotation.y = odom.pose.pose.orientation.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w

        self.tf_send.sendTransform(t)

        '''t.header.frame_id = "map" 
        t.child_frame_id = "odom"
        t.header.stamp = rospy.Time.now() 
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0

        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        self.tf_send.sendTransform(t)'''


    def update_robot_pose(self, v, w): 
        self.x = self.x + v * np.cos(self.theta) * self.dt
        self.y = self.y + v * np.sin(self.theta) * self.dt
        self.theta = self.theta + w * self.dt

    def prediccion(self):

        self.miu_hat = np.array([[self.x + self.dt * self.v * np.cos(self.theta)],
                                 [self.y + self.dt * self.v * np.sin(self.theta)],
                                 [self.theta + self.dt * self.w]])
        
        self.H = np.array([[1, 0, -self.dt * self.v * np.sin(self.theta)],
                           [0, 1, self.dt * self.v * np.cos(self.theta)],
                           [0, 0, 1]])
        
        self.sigma_hat = self.H.dot(self.sigma).dot(self.H.T) + self.Q

    def correction(self):
        dx = self.x_aruco - self.x
        dy = self.y_aruco - self.y
        p = dx ** 2 + dy ** 2
        print(dx, dy)
        print(self.coords_aruco)

        self.z_hat = np.array([[np.sqrt(p)], [np.arctan2(dy, dx) - self.theta_pred]])

        self.G = np.array([[-dx / np.sqrt(p), -dy / np.sqrt(p), 0],
                           [dy / p, -dx / p, -1]])
        
        self.R = np.array([[2.8774e-07, 0.0], [0.0, 7.3658e-09]])
        self.R = np.random.normal(0, self.R)

        self.Z = self.G.dot(self.sigma_hat).dot(self.G.T) + self.R

        self.K = self.sigma_hat.dot(self.G.T).dot(inv(self.Z))

        self.miu = self.miu_hat + self.K.dot((self.coords_aruco - self.z_hat))

        self.sigma = (np.eye(3) - (self.K.dot(self.G))) * self.sigma_hat
        
    def propagate(self):
        self.miu = self.miu_hat
        self.x = self.miu[0].item()
        self.y = self.miu[1].item()
        self.theta = self.miu[2].item()
        self.theta_pred = self.miu_hat[2].item()
        self.sigma = self.sigma_hat

if __name__ == "__main__": 
    Localisation()