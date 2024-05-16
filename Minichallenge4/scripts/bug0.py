#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion 
from sensor_msgs.msg import LaserScan
import numpy as np

class Bug0():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        rospy.init_node('bug0') 

        rospy.Subscriber("scan", LaserScan, self.laser_cb)
        rospy.Subscriber("set_point", Pose, self.set_point_cb)
        rospy.Subscriber("odom", Odometry, self.odom_cb)
        rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb)
        rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb)

        self.pub_cmd_vel = rospy.Publisher("puzzlebot_1/base_controller/cmd_vel", Twist, queue_size = 1)

        rate = rospy.Rate(50)
        dt = 1.0 / 10

        self.xg = 0.0
        self.yg = 0.0

        self.e_theta = 0.0
        self.theta_ao = 0.0
        self.theta_fw = 0.0
        
        self.xr = 0.0
        self.yr = 0.0
        self.tr = 0.0

        self.goal_r = False
        self.lidar_r = False

        vel_msg = Twist()

        self.wr = 0.0
        self.wl = 0.0

        self.closest_angle = 0.0
        self.closest_range = np.inf

        self.hp = 0.0
        self.lp = 0.0
        self.tolerance = 0.05
        self.min_progress = 0.15

        self.v = 0.0
        self.w = 0.0

        self.fw = 0.23

        self.current_state = 'GTG'

        while not rospy.is_shutdown():
             if self.lidar_r and self.goal_r:
                  self.goal_r = 0
                  self.get_closest_range(self.lidar_msg)
                  self.get_theta_ao()
                  self.get_theta_gtg()
                  d_t = np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2)

                  if self.at_goal():
                       print("Done")
                       vel_msg.linear.x = 0
                       vel_msg.angular.z = 0
                  elif self.current_state == "GTG":
                       if self.closest_range <= self.fw:
                            self.get_theta_fw(True)

                            if abs(self.theta_fw - self.e_theta) <= np.pi/2:
                                 self.current_state = "CW"
                            else:
                                 self.current_state = "CCW"
                       else:
                            print("GTG")
                            self.gtg_control()
                            vel_msg.linear.x = self.v
                            vel_msg.angular.z = self.w
                  elif self.current_state == "CW":
                       if d_t < abs(d_t - self.min_progress) and abs(self.theta_ao - self.e_theta) <= np.pi/2:
                            self.current_state = "GTG"
                            print("GTG")
                       elif self.at_goal:
                            self.current_state = "Stop"
                  elif self.current_state == "CCW":
                       if d_t < abs(d_t - self.min_progress) and abs(self.theta_ao - self.e_theta) > np.pi/2:
                            self.current_state = "GTG"
                            print("GTG")
                       elif self.at_goal:
                            self.current_state = "Stop"

    def at_goal(self):
        return np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2) < self.tolerance

    def get_closest_range(self):
        min_idx = np.argmin(self.lidar_msg.ranges)
        self.closest_range = self.lidar_msg.ranges[min_idx]
        self.closest_angle = self.lidar_msg.angle_min + min_idx * self.lidar_msg.angle_increment
        self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle))

    def get_theta_gtg(self):
        tg = np.arctan2(self.yg - self.yr, self.xg - self.xr)
        self.e_theta = tg - self.tr

    def gtg_control(self):
        kv_m = 0.16
        kw_m = 0.8

        av = 2.0
        aw = 2.0

        e_d = np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2)
        tg = np.arctan2(self.yg - self.yr, self.xg - self.xr)
        e_theta = tg - self.tr

        if e_theta != 0:
            kw = kw_m * (1 - np.exp(-aw * e_theta ** 2)) / abs(e_theta)
        else:
            kw = 0.05
        self.w = kw * e_theta

        if abs(e_theta) > np.pi/8:
            self.v = 0
        else:
            kv = kv_m * (1 - np.exp(-av * e_d ** 2))/abs(e_d)
            self.v = kv * e_d

    def get_theta_ao(self):
        theta_ao = self.closest_angle - np.pi
        self.theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

    def get_theta_fw(self, clockwise):
        if clockwise:
            theta_fw = -np.pi / 2 + self.theta_ao
        else:
            theta_fw = np.pi / 2 + self.theta_ao
        self.theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

    def fw_control(self):
        kw = 1.5
        self.v = 0.08
        self.w = kw * self.theta_fw

    def laser_cb(self, msg):
        self.lidar_msg = msg
        self.lidar_r = True

    def set_point_cb(self, msg):
        self.xg = msg.position.x
        self.yg = msg.position.y
        self.goal_r = True

    def odom_cb(self, msg):
        orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
            ]
        euler = euler_from_quaternion(orientation)

        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y
        self.tr = euler[2]

    def wr_cb(self, msg):
        self.wr = msg

    def wl_cb(self, msg):
        self.wl = msg

    def cleanup(self): 
            #This function is called just before finishing the node 
            # You can use it to clean things up before leaving 
            # Example: stop the robot before finishing a node.   
            vel_msg = Twist()
            self.pub_cmd_vel.publish(vel_msg)

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    Bug0()  