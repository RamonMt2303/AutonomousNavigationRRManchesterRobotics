#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray
from tf.transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
import numpy as np

class PuzzlebotLocalisation():
	def __init__(self):
		rospy.init_node('localisation')

		############################### SUBSCRIBERS #####################################
		rospy.Subscriber("/wl", Float32, self.wl_cb)
		rospy.Subscriber("/wr", Float32, self.wr_cb)
		rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
		rospy.Subscriber("/aruco_position", Float32MultiArray, self.aruco_cb)
		rospy.Subscriber("/set_point", Float32, Pose, self.setpoint_cb)

		############################### PUBLISHERS #####################################
		rospy.Publisher('flag', Float32, queue_size=1)  

		self.xg = 0.0
		self.yg = 0.0


		x_init = 0.0 #x position of the robot
		y_init = 0.0 #y position of the robot
		theta_init = 0.0 #theta position of the robot

		# Publishers
		self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)
		self.pub_goal = rospy.Publisher('/goal', Marker, queue_size=1)

		############## ROBOT CONSTANTS ################
		self.r = 0.05 #puzzlebot wheel radius [m]
		self.L = 0.19 #puzzlebot wheel separation [m]
		self.dt = 0.02 # Desired time to update the robot's pose [s]

		################ Variables ###################
		self.mu = np.array([x_init, y_init, theta_init])
		self.sigma = np.zeros((3,3))
		self.Qk = np.zeros((3,3))
		self.Rk = np.zeros((2,2))
		self.cov_ruido = np.zeros((2,3))
		self.jacob = np.zeros((3,2))

		self.wl = 0.0
		self.wr = 0.0
		self.theta_ant = 0.0
		self.v = 0.0
		self.w = 0.0
		self.kr = 25.0
		self.kl = 35.0

		self.odometry = Odometry()

		self.tf_broadcaster = tf2_ros.TransformBroadcaster()
		self.t = TransformStamped()

		self.aruco_id = 0
		self.z_i = np.zeros(2)
		self.m_i = np.zeros(2)

		rate = rospy.Rate(int(0.5/self.dt))

		KF = KalmanFilter(self.dt, self.mu)

		while not rospy.is_shutdown():


			self.update_robot_pose()

			self.cov_ruido = np.array([[self.kr*np.abs(self.wr), 0], [0, self.kl*np.abs(self.wl)]])
			self.jacob = (0.5*self.r*self.dt) * np.array([[np.cos(self.theta_ant), np.cos(self.theta_ant)], [np.sin(self.theta_ant), np.sin(self.theta_ant)], [2/self.L, -2/self.L]])

			self.Qk = self.jacob.dot(self.cov_ruido).dot(self.jacob.T)

			self.mu, self.sigma = KF.predict([self.v, self.w], self.Qk)

			if self.aruco_id > 0:
				self.mu, self.sigma = KF.correct(self.m, self.z, self.Rk)

			self.aruco_id = 0

			self.pub_odom.publish(self.odometry)
			self.publish_goal_marker()


			rate.sleep()

	def wl_cb(self, msg):
		self.wl = msg.data

	def wr_cb(self, msg):
		self.wr = msg.data
		
	def setpoint_cb(self, msg):
		self.xg = msg.position.x
		self.yg = msg.position.y
		
	def update_robot_pose(self):
		#This functions receives the robot speed v [m/s] and w [rad/s]
		# and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad]
		# is the orientation,

		v = self.r*(self.wr + self.wl)/2
		w = self.r*(self.wr - self.wl)/self.L

		self.mu[0] = self.mu[0] + v*np.cos(self.mu[2])*self.dt
		self.mu[1] = self.mu[1] + v*np.sin(self.mu[2])*self.dt
		self.theta_ant = self.mu[2]
		self.mu[2] = self.mu[2] + w*self.dt

		self.odometry.header.stamp = rospy.Time.now()
		self.odometry.header.frame_id = "odom"
		self.odometry.child_frame_id = "base_link"

        # Fill the pose information
		self.odometry.pose.pose.position.x = self.mu[0]
		self.odometry.pose.pose.position.y = self.mu[1]
		self.odometry.pose.pose.position.z = 0.0

		quat = quaternion_from_euler(0,0,self.mu[2])

		self.odometry.pose.pose.orientation.x = quat[0]
		self.odometry.pose.pose.orientation.y = quat[1]
		self.odometry.pose.pose.orientation.z = quat[2]
		self.odometry.pose.pose.orientation.w = quat[3]

		self.odometry.twist.twist.linear.x = v
		self.odometry.twist.twist.angular.z = w

        # Init a 36 elements array
		self.odometry.pose.covariance = [0.0] * 36

        # Fill the 3D covariance matrix
		self.odometry.pose.covariance[0] = self.sigma[0][0]
		self.odometry.pose.covariance[1] = self.sigma[0][1]
		self.odometry.pose.covariance[5] = self.sigma[0][2]
		self.odometry.pose.covariance[6] = self.sigma[1][0]
		self.odometry.pose.covariance[7] = self.sigma[1][1]
		self.odometry.pose.covariance[11] = self.sigma[1][2]
		self.odometry.pose.covariance[30] = self.sigma[2][0]
		self.odometry.pose.covariance[31] = self.sigma[2][1]
		self.odometry.pose.covariance[35] = self.sigma[2][2]

        # Fill the transformation information
		self.t.header.stamp = rospy.Time.now()
		self.t.header.frame_id = "odom"
		self.t.child_frame_id = "base_link"
		self.t.transform.translation.x = self.mu[0]
		self.t.transform.translation.y = self.mu[1]
		self.t.transform.translation.z = 0.0

		self.t.transform.rotation.x = quat[0]
		self.t.transform.rotation.y = quat[1]
		self.t.transform.rotation.z = quat[2]
		self.t.transform.rotation.w = quat[3]

        # A transformation is broadcasted instead of published
		self.tf_broadcaster.sendTransform(self.t) #broadcast the transformation

	def publish_goal_marker(self):
		marker_goal = Marker()
		marker_goal.header.frame_id = "odom"
		marker_goal.header.stamp = rospy.Time.now()
		marker_goal.ns = "goal"
		marker_goal.id = 0
		marker_goal.type = Marker.CUBE
		marker_goal.action = Marker.ADD
		marker_goal.pose.position.x = self.x_target
		marker_goal.pose.position.y = self.y_target
		marker_goal.pose.position.z = 0
		marker_goal.pose.orientation.x = 0.0
		marker_goal.pose.orientation.y = 0.0
		marker_goal.pose.orientation.z = 0.0
		marker_goal.pose.orientation.w = 1.0
		marker_goal.scale.x = 0.2
		marker_goal.scale.y = 0.2
		marker_goal.scale.z = 0.5
		marker_goal.color.a = 1.0
		marker_goal.color.r = 0.0
		marker_goal.color.g = 1.0
		marker_goal.color.b = 0.0

		self.goal_pub.publish(marker_goal)


	def aruco_cb(self, msg):
		self.aruco_id = int(msg.data[2])
		self.z = msg.data[0:2]
		self.m = MarkerLocations[self.aruco_id]

	def cmd_vel_cb(self, msg):
		self.v = msg.linear.x
		self.w = msg.angular.z


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
	PuzzlebotLocalisation()