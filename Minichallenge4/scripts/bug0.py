#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion 
from sensor_msgs.msg import LaserScan
import numpy as np

class Bug0():
    def __init__(self):
        rospy.init_node('bug0') 
        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb)
        #rospy.Subscriber("set_point", Pose, self.set_point_cb)
        rospy.Subscriber("puzzlebot_1/base_controller/odom", Odometry, self.odom_cb)

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
        self.tolerance = 0.1
        self.min_progress = 0.0

        self.v = 0.0
        self.w = 0.0

        self.fw = 0.33

        self.current_state = 'GTG'
        self.set_point_cb()
        self.previous_distance_to_goal = np.inf

        while not rospy.is_shutdown():
             if self.lidar_r and self.goal_r:
                  self.get_closest_range()
                  #self.get_theta_ao()
                  #self.get_theta_gtg()
                  #self.get_theta_fw(True)
                  d_t = np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2)

                  if self.at_goal():
                       print("Done")
                       self.v = 0.0
                       self.w = 0.0
                  elif self.current_state == "GTG":
                       print(self.current_state)
                       if self.closest_range < self.fw:
                            if abs(self.closest_angle - self.e_theta) > np.pi/2:
                                 self.current_state = "CW"
                            elif abs(self.closest_angle - self.e_theta) <= np.pi/2:
                                 self.current_state = "CCW"
                       else:
                            self.gtg_control()
                  elif self.current_state == "CW":
                       print(self.current_state)
                       self.fw_control(True)
                       if self.made_progress(d_t) and abs(self.theta_ao - self.e_theta) < np.pi/2:
                            self.current_state = "GTG"
                       elif self.at_goal():
                            self.current_state = "Stop"
                       else:
                            self.fw_control(True)
                  elif self.current_state == "CCW":
                       print(self.current_state)
                       self.fw_control(False)
                       if self.made_progress(d_t) and abs(self.theta_ao - self.e_theta) < np.pi/2:
                            self.current_state = "GTG"
                       elif self.at_goal():
                            self.current_state = "Stop"
                       else:
                            self.fw_control(False)

             vel_msg.linear.x = self.v
             vel_msg.angular.z = self.w
             self.pub_cmd_vel.publish(vel_msg)
             rate.sleep()


    def at_goal(self):
        #print(abs(self.xr - self.xg), abs(self.yr - self.yg))
        return (abs(self.xr - self.xg) < 0.05) and (abs(self.yr - self.yg) < 0.05)
    
    def made_progress(self, current_distance):
        progress = self.previous_distance_to_goal - current_distance
        self.previous_distance_to_goal = current_distance
        #print(abs(progress) > 0.001)
        return abs(progress) > 0.001

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

        new_angle_min = -np.pi/2.0
        ranges_size = len(self.lidar_msg.ranges)
        cropped_ranges = self.lidar_msg.ranges[int((ranges_size)/4):3*int((ranges_size)/4)]
        cropped_ranges = np.roll(cropped_ranges, int(len(cropped_ranges)/2 + 1))
        min_idx = np.argmin(cropped_ranges)
        self.closest_range = cropped_ranges[min_idx]
        closest_angle = new_angle_min + min_idx * self.lidar_msg.angle_increment
        # limit the angle to [-pi, pi]
        self.closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))

    def gtg_control(self):
        kv_m = 0.16
        kw_m = 0.3

        av = 2.0
        aw = 2.0

        e_d = np.sqrt((self.xg - self.xr) ** 2 + (self.yg - self.yr) ** 2)
        tg = np.arctan2(self.yg - self.yr, self.xg - self.xr)
        print("TG: ", tg)
        e_theta = tg - self.tr
        print("theta r: ", self.tr)
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
        print("e theta: ", e_theta)

        kw = kw_m * (1 - np.exp(-aw * e_theta ** 2)) / abs(e_theta)        
        self.w = kw * e_theta
        print("W: ", self.w)

        kv = kv_m * (1 - np.exp(-av * e_d ** 2))/abs(e_d)
        self.v = kv * e_d
        print("V: ", self.v)

    def fw_control(self, clockwise):
        theta_ao = self.closest_angle + np.pi/2
        self.theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))
        if clockwise:
            theta_fw = np.pi / 2 + self.theta_ao
        else:
            theta_fw = -np.pi / 2 + self.theta_ao
        self.theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        kw = 1.0
        self.v = 0.08
        self.w = kw * self.theta_fw

    def laser_cb(self, msg):
        self.lidar_msg = msg
        self.lidar_r = True

    def set_point_cb(self):
        '''#Map 1
        self.xg = 1.5
        self.yg = 1.2'''

        '''#Map 1
        self.xg = 2.3
        self.yg = 2.0'''

        '''#Map 2
        self.xg = -1.15
        self.yg = 1.5'''

        '''#Map 2
        self.xg = 0.35
        self.yg = 2.4'''

        #Map 3
        self.xg = 4.5
        self.yg = -0.5

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