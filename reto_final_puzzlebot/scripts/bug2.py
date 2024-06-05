#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 
from sensor_msgs.msg import LaserScan
import numpy as np
import tf2_ros

class Bug0():
    def __init__(self):
        rospy.init_node('bug0') 
        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber("scan", LaserScan, self.laser_cb)
        rospy.Subscriber("odom", Odometry, self.odom_cb)

        self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size = 1)

        dt = 0.02
        rate = rospy.Rate(1/dt)

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
        self.tolerance = 0.1
        self.min_progress = 0.5 # CAMBIAR PARA COMPLETAR BUG0

        self.v = 0.0
        self.w = 0.0

        self.fw = 0.25

        self.current_state = 'GTG'
        self.set_point_cb()
        self.previous_distance_to_goal = np.inf

        while not rospy.is_shutdown():
            if self.lidar_r and self.goal_r:
                self.get_closest_range()
                if self.at_goal():
                    print("Done")
                    self.v = 0.0
                    self.w = 0.0
                elif self.current_state == "GTG":
                    print(self.current_state)
                    if self.closest_range <= self.fw:
                        if abs(self.closest_angle - self.e_theta) > np.pi/4:
                                self.current_state = "CW"
                                print(self.current_state)
                        elif abs(self.closest_angle - self.e_theta) <= np.pi/4:
                                self.current_state = "CCW"
                                print(self.current_state)
                    else:
                        self.gtg_control()
                elif self.current_state == "CW":
                    print(self.current_state)
                    self.fw_control(True)
                    if self.on_m_line():
                        self.current_state = "GTG"
                    elif self.at_goal():
                        self.current_state = "Stop"
                    else:
                        self.fw_control(True)
                elif self.current_state == "CCW":
                    print(self.current_state)
                    self.fw_control(False)
                    if self.on_m_line():
                        self.current_state = "GTG"
                    elif self.at_goal():
                        self.current_state = "Stop"
                    else:
                        self.fw_control(False)

            print("X robot: " + str(self.xr) + "X goal: " + str(self.xg))
            print("Y robot: " + str(self.yr) + "Y goal: " + str(self.yg))
            vel_msg.linear.x = self.v
            vel_msg.angular.z = self.w
            self.pub_cmd_vel.publish(vel_msg)
            rate.sleep()

    def on_m_line(self):
        a = self.yr - self.yg
        b = self.xg - self.xr
        c = self.xr * self.xg - self.yg * self.yr
        error = np.abs(a * self.xg + b * self.yg + c) / np.sqrt(a * a + b * b)
        
        if error <= 0.08:
             return True
        return False

    def at_goal(self):
        return (abs(self.xr - self.xg) < self.tolerance) and (abs(self.yr - self.yg) < self.tolerance)
    
    def made_progress(self):
        return np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2)

    def get_closest_range(self):
        '''# Create a copy of the ranges to avoid modifying the original data
        limited_ranges = np.array(self.lidar_msg.ranges)

        # Filter out ranges that exceed the max_range
        limited_ranges[limited_ranges > 4] = np.inf  # Use np.inf to ignore these values

        # Find the closest range within the limited ranges
        min_idx = np.argmin(limited_ranges)
        self.closest_range = limited_ranges[min_idx]
        self.closest_angle = self.lidar_msg.angle_min + min_idx * self.lidar_msg.angle_increment
        self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle))'''

        new_angle_min = self.lidar_msg.angle_min
        #ranges_size = len(self.lidar_msg.ranges)
        #cropped_ranges = self.lidar_msg.ranges[int((ranges_size)/4):3*int((ranges_size)/4)]
        #cropped_ranges = np.roll(cropped_ranges, int(len(cropped_ranges)/2 + 1))
        min_idx = np.argmin(self.lidar_msg.ranges)
        self.closest_range = self.lidar_msg.ranges[min_idx]
        closest_angle = new_angle_min + min_idx * self.lidar_msg.angle_increment
        closest_angle += np.pi
        # limit the angle to [-pi, pi]
        self.closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))

    def gtg_control(self):
        kv_m = 0.15
        kw_m = 0.2

        av = 1.0
        aw = 1.0

        e_d = np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2)
        tg = np.arctan2(self.yg - self.yr, self.xg - self.xr)
        #print("TG: ", tg)
        e_theta = tg - self.tr
        #print("theta r: ", self.tr)
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
        #print("e theta: ", e_theta)

        kw = kw_m * (1 - np.exp(-aw * e_theta ** 2)) / abs(e_theta)        
        self.w = kw * e_theta
        #print("W: ", self.w)

        if abs(e_theta) > np.pi/8:
             self.v = 0.0
        else:
            kv = kv_m * (1 - np.exp(-av * e_d ** 2))/abs(e_d)
            self.v = kv * e_d
        #print("V: ", self.v)

    def fw_control(self, clockwise):
        self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle))
        theta_ao = self.closest_angle
        self.theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))
        if clockwise:
            theta_fw = -np.pi / 2 + self.theta_ao
        else:
            theta_fw = np.pi / 2 + self.theta_ao
        self.theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        kw = 0.4
        self.v = 0.05
        self.w = kw * self.theta_fw

    def laser_cb(self, scan):
        self.lidar_msg = scan
        self.lidar_r = True

    def set_point_cb(self):
        '''#Map 1
        self.xg = 1.5
        self.yg = 1.2'''

        #Map 1
        self.xg = 1.15
        self.yg = 1.4

        '''#Map 2
        self.xg = -1.15
        self.yg = 1.5'''

        #Map 2
        '''self.xg = 0.7
        self.yg = 2.5'''

        #Map 3
        '''self.xg = 4.5
        self.yg = -0.5'''

        '''Map 4
        self.xg = 0.0
        self.yg = -2.5'''
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