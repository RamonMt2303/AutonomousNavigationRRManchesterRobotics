#!/usr/bin/env python

import rospy  
from geometry_msgs.msg import Twist  
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped 
from tf.transformations import quaternion_from_euler 
from std_msgs.msg import Float32MultiArray
import numpy as np 

np.set_printoptions(suppress=True) 
np.set_printoptions(formatter={'float': '{: 0.4f}'.format}) 

class Localisation():  
    def __init__(self):  
        rospy.init_node('localisation')
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb)
        rospy.Subscriber("aruco_topic", Float32MultiArray, self.aruco_cb)
        self.odom_pub = rospy.Publisher('odom', Odometry ,queue_size=1)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        self.tf_map_to_odom = tf2_ros.TransformBroadcaster()

        self.r = 0.05 
        self.L = 0.19 
        self.dt = 0.1 

        self.w = 0.0 
        self.v = 0.0 
        self.x = 2.9
        self.x_ant = 2.9
        self.y = 0.3 
        self.y_ant = 0.3
        self.theta = 1.5708 
        self.theta_ant = 1.5708
        self.theta = 0.0  
        self.wr = 0.0
        self.wl = 0.0
        self.id_aruco = 0
        self.wr_k = 0.16
        self.wl_k = 0.0175
        
        self.Z = np.array([[0, 0], [0, 0]])
        self.I = np.eye(3)
        self.H = np.eye(3, 3)
        self.covariance_ant = np.eye(3) * 0.01
        self.covariance_final = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
        self.miu_ant = np.array([[self.x], [self.y], [self.theta]])
        self.miu = np.array([[0], [0], [0]])
        self.miu_final = np.array([[0], [0], [0]])
        self.sigma = np.array([[0, 0], [0, 0]])
        self.w_sigma_const = 0.5 * self.dt * self.r
        self.odom = Odometry() 
        self.get_aruco = False
      
        rate = rospy.Rate(int(1.0 / self.dt))

        while not rospy.is_shutdown(): 

            self.sigma[0][0] = 1.5 * np.abs(self.wr)
            self.sigma[1][1] = 2.0 * np.abs(self.wl)

            self.gradient_W = 0.5 * self.r * self.dt * (np.array([[np.cos(self.theta_ant), np.cos(self.theta_ant)],
                                                                  [np.sin(self.theta_ant), np.sin(self.theta_ant)],
                                                                  [2.0 / self.L, -2.0 / self.L]]))
            
            self.Q = self.gradient_W.dot(self.sigma).dot(self.gradient_W.T)

            [self.v, self.w] = self.get_robot_vel(self.wr, self.wl)
            self.prediction_step()

            if self.get_aruco:
                #self.correction_step()
                self.get_aruco = False

            self.update_robot_pose(self.v, self.w) 
            odom_msg = self.get_odom_stamped(self.x, self.y, self.theta, self.covariance_final, self.v, self.w) 

            self.send_transform(odom_msg)
            self.odom_pub.publish(odom_msg) 
            
            
            rate.sleep()

    def prediction_step(self):

        self.miu = np.array([
            [self.miu_ant[0][0] + self.dt * self.v * np.cos(self.miu_ant[2][0])],
            [self.miu_ant[1][0] + self.dt * self.v * np.sin(self.miu_ant[2][0])],
            [self.miu_ant[2][0] + self.dt * self.w]])
        
        self.H[0][2] = -self.dt * self.v * np.sin(self.miu_ant[2][0])
        self.H[1][2] = self.dt * self.v * np.cos(self.miu_ant[2][0])
	
        self.covariance = self.H.dot(self.covariance_ant).dot(self.H.T) + self.Q
        
        self.covariance_final = self.covariance.copy()

    
    def correction_step(self):
        self.delta_x = self.x_aruco - self.miu[0][0] 
        self.delta_y =  self.y_aruco - self.miu[1][0] 
        self.p = self.delta_x ** 2 + self.delta_y ** 2

        self.z = np.array([[self.d_aruco], 
                           [self.theta_aruco]])


        self.z_hat = np.array([[np.sqrt(self.p)], 
                               [np.arctan2(self.delta_y, self.delta_x) - self.miu[2][0]]])

        self.G = np.array([[-(self.delta_x / np.sqrt(self.p)), -self.delta_y / np.sqrt(self.p), 0],
                           [(self.delta_y / self.p), -(self.delta_x / self.p), -1]])
        
        self.R_k = np.array([[0.1, 0], 
                             [0, 0.02]])
        
        self.R_k = np.random.normal(0,self.R_k)

        self.Z = self.G.dot(self.covariance).dot(self.G.T) + self.R_k

        self.K = np.array(self.covariance.dot(self.G.T).dot(np.linalg.inv(self.Z)))

        self.miu_final = self.miu + self.K.dot((self.z - self.z_hat))

        self.covariance_final = (self.I - self.K.dot(self.G)).dot(self.covariance)

        self.x = self.miu_final[0][0]
        self.y = self.miu_final[1][0]
        self.theta = self.miu_final[2][0]


        self.miu = self.miu_final.copy()
        self.covariance_ant = self.covariance_final.copy()

    def wl_cb(self, msg): 
        self.wl = msg.data

    def wr_cb(self, msg): 
        self.wr = msg.data

    def aruco_cb(self, msg):
        id_aruco = int(msg.data[0])
        self.d_aruco = msg.data[1]
        self.theta_aruco = msg.data[2]
        self.id_arucos(id_aruco)
        self.get_aruco = True

    def id_arucos(self, id):
        x_y = {702: [0, 0.80], 701: [0, 1.60], 703: [1.73, 0.80],
               704: [2.63, 0.39], 705: [2.85, 0], 706: [2.865, 2.00],
               707: [1.735, 1.22]}
        self.x_aruco = x_y[id][0]
        self.y_aruco = x_y[id][1]

    def get_robot_vel(self, wr, wl): 
        v = ((wl + wr) / 2) * self.r
        w = ((wr - wl) / self.L) * self.r
        return [v, w]

    def get_odom_stamped(self, x, y, yaw, Sigma, Mu_v, Mu_w): 
        odom_stamped = Odometry()
        odom_stamped.header.frame_id = "odom"
        odom_stamped.child_frame_id = "base_link"
        odom_stamped.header.stamp = rospy.Time.now()
        odom_stamped.pose.pose.position.x = x
        odom_stamped.pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        odom_stamped.pose.pose.orientation.x = quat[0]
        odom_stamped.pose.pose.orientation.y = quat[1]
        odom_stamped.pose.pose.orientation.z = quat[2]
        odom_stamped.pose.pose.orientation.w = quat[3]

        odom_array = np.array([[Sigma[0][0], Sigma[0][1], 0, 0, 0, Sigma[0][2]],
                               [Sigma[1][0], Sigma[1][1], 0, 0, 0, Sigma[1][2]],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [Sigma[2][0], Sigma[2][1], 0, 0, 0, Sigma[2][2]]])
        
        odom_stamped.pose.covariance = odom_array.flatten().tolist()
        odom_stamped.twist.twist.linear.x = Mu_v
        odom_stamped.twist.twist.angular.z = Mu_w
        
        return odom_stamped

    def update_robot_pose(self, v, w): 
        self.x = self.x + v * np.cos(self.theta) * self.dt
        self.y = self.y + v * np.sin(self.theta) * self.dt
        self.theta = self.theta + w * self.dt
        print("Teta", self.theta)
        print("X", self.x)
        print("Y", self.y)
        
    
    
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

        self.tf_send = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.frame_id = "map" 
        t.child_frame_id = "odom"
        t.header.stamp = rospy.Time.now() 
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0

        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        self.tf_send.sendTransform(t)

if __name__ == '__main__':  
    try:  
        Localisation()  
    except rospy.ROSInterruptException:  
        pass