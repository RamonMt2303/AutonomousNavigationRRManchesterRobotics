#!/usr/bin/env python3  

import rospy  
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan   #Lidar
from tf.transformations import euler_from_quaternion
import numpy as np 

#This class will make the puzzlebot move to a given goal 
class AutonomousNav():  
	def __init__(self):  
		rospy.on_shutdown(self.cleanup)
		############ ROBOT CONSTANTS ###############
		self.r = 0.05 #wheel radius [m]
		self.L = 0.19 #wheel separation [m] 
		############ VARIABLES ############### 
		self.dGTG_h = 0
		self.closest_angle = 0
		self.closest_range = 0
		self.theta_r = 0
		self.x_r = 0
		self.y_r = 0

		self.x_target= -1.3 #x position of the goal 
		self.y_target= 1.5 #y position of the goal 

		self.goal_received = 1 #flag to indicate if the goal has been received 
		self.lidar_received = 0 #flag to indicate if the laser scan has been received 
		self.target_position_tolerance=0.29 #acceptable distance to the goal to declare the robot has arrived to it [m] 
		d_fw = 0.35 # distance from closest obstacle to activate the avoid obstacle behavior [m] # dFw 
		v_msg=Twist() #Robot's desired speed  
		self.wr = 0 #right wheel speed [rad/s]
		self.wl = 0 #left wheel speed [rad/s]
		self.current_state = 'GoToGoal' #Robot's current state 

		# inicializamos y definimos ganancias para goal to goal 
		self.k_v = 0.1
		self.k_w = 0.9
		
		#definimos minimo progreso requerido para ir a goal to goal
		self.minprogress = 2.8

		###******* INIT PUBLISHERS *******###
		self.pub_cmd_vel = rospy.Publisher('puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)  
		############################### SUBSCRIBERS #####################################  
		rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb)  
		rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb)
		rospy.Subscriber("odom", Odometry, self.update_pos_cb) 
		rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb) 
		#********** FRECUENCIA Y SAMPLING TIME **********###  
		freq = 20 
		rate = rospy.Rate(freq) #freq Hz

		while  rospy.get_time () == 0:
			print('NO simulated time')
		print('i got a valid time')
		################ MAIN LOOP ################  
		while not rospy.is_shutdown():			
			if self.lidar_received:
				######################## PRIMER ESTADO
				if self.current_state == 'Stop':
					if self.goal_received:
						print("Change to Go to goal from stop") 
						self.current_state = "GoToGoal" 
						self.goal_received = 0
					else:
						v_msg.linear.x = 0.0 
						v_msg.angular.z = 0.0

				######################## SEGUNDO ESTADO
				elif self.current_state == 'GoToGoal':
					# CALCULAR VEL Y THETA PARA IR AL GOAL. CALCULAR E_THETA PARA REALIZAR COMPARACION
					v, w, e_theta = self.move_to_goal()
					# ASIGNAR VEL Y THETA PARA IR AL GOAL
					v_msg.linear.x = v
					v_msg.angular.z = w
					# CALCULAR ANGULO DE THETAFWC PARA LUEGO REALIZAR COMPARACION DE GIRO EN FW
					wfwc = self.calculate_thetafwc() 
					# PRIMERA COMPARACION EN GO TO GOAL ----
					# GIRA EN CLOCKWISE O HACIA MANECILLAS DEL RELOJ
					if self.closest_range < d_fw and abs(wfwc - e_theta) <= np.pi/2:
						# CALCULAR DISTANCIA ANTERIOR, h1
						self.dGTG_h = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)
						print("distancia anterior de GoalToGoal en FWC" + str(self.dGTG_h))
						# CAMBIAMOS DE INMEDIATO DE ESTADO
						self.current_state = 'FollowingWallClockwise'
			
					# SEGUNDA COMPARACION EN GO TO GOAL ----
					# GIRA EN COUNTER CLOCKWISE O HACIA MANECILLAS CONTRARIAS DEL RELOJ
					elif self.closest_range < d_fw and abs(wfwc - e_theta) > np.pi/2:
						# CALCULAR DISTANCIA ANTERIOR, h1
						self.dGTG_h = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)
						print("distancia anterior de GoalToGoal en FCC" + str (self.dGTG_h))
						# CAMBIAMOS DE INMEDIATO DE ESTADO
						self.current_state = 'FollowingWallCounterClockwise'
					
					# TERCERA COMPARACION EN GO TO GOAL ----
					# DETENER EL ROBOT
					elif np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) < self.target_position_tolerance:
						
						self.current_state = 'Stop'
				
				######################## TERCER ESTADO
				elif self.current_state == 'FollowingWallClockwise':

					# CALCULAR E_THETA PARA LUEGO REALIZAR COMPARACIONES

					v,w,e_theta = self.move_to_goal()

					# DEFINIR DISTANCIA ACTUAL EN TIEMPO REAL

					d_GTG = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)

					# REALIZAR FOLLOW WALL EN CLOCKWISE

					vfw, wfw = self.following_wallc()
					
					# ASIGNAR VELOCIDADES CORRESPONDIENTES DEL ALG CLOCKWISE
					v_msg.linear.x = vfw
					v_msg.angular.z = wfw

	
					# CALCULAR ANGULO AVOIDOBSTACTLES PARA DEFINIR CLEAR SHOT
					
					theta_AO = self.closest_angle + np.pi
					theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO)) # arctan

					# PRIMERA COMPARACION PARA ESTADO DE FWC ----
					# REALIZAR CLEAR SHOT Y PROGRESS WITHFATGUARDS PARA IR A GOAL TO GOAL

					print("distancia actual " + str(d_GTG) + "dis anterior " + str(self.dGTG_h) + "minpro " + str(self.minprogress))
					if abs(theta_AO - e_theta) < np.pi/2 and d_GTG < abs(self.dGTG_h - self.minprogress):
						
						self.current_state = 'GoToGoal'
					
					# SEGUNDA COMPARACION PARA ESTADO DE FWC ----
					# DETENER EL ROBOT !!!
					elif np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) < self.target_position_tolerance:
						
						self.current_state = 'Stop'
						
				######################## CUARTO ESTADO	
				elif self.current_state == "FollowingWallCounterClockwise":
					

					# CALCULAR E_THETA PARA LUEGO REALIZAR COMPARACIONES
					
					v,w,e_theta = self.move_to_goal()

					# DEFINIR DISTANCIA ACTUAL EN TIEMPO REAL

					d_GTG = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)

					vfw, wfw = self.following_wallcc()
					v_msg.linear.x = vfw
					v_msg.angular.z = wfw

					
					# CALCULAR EL ANGULO DE AVOIDOBSTACLES PARA DEFINIR CLEAR SHOT
					theta_AO = self.closest_angle + np.pi
					theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO)) # arctan

					# REALIZAR CLEAR SHOT Y PROGRESSWITHFATGUARDS PARA IR A GO TO GOAL

					#print("THETA_AO " + str(theta_AO))
					#print("E_THETA " + str(e_theta))
					#print("D_GTG " + str(d_GTG))
					#print("DGTG_h " + str(self.dGTG_h))
					#print("minprogress " + str(self.minprogress))
					print("distancia actual " + str(d_GTG) + "dis anterior " + str(self.dGTG_h) + "minpro " + str(self.minprogress))
					if abs(theta_AO - e_theta) < np.pi/2 and d_GTG < abs(self.dGTG_h - self.minprogress):
						self.current_state = 'GoToGoal'
					elif np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) < self.target_position_tolerance:
						self.current_state = 'Stop'
			
			self.pub_cmd_vel.publish(v_msg) # PUBLICAMOS VELOCIDADES LINEALES Y ANGULARES AL ROBOT
			print(self.current_state) # IMPRIMIMOS ESTADO ACTUAL
			rate.sleep()
	
	def update_pos_cb(self, odom):
		self.x_r = odom.pose.pose.position.x # POSICION EN X
		self.y_r = odom.pose.pose.position.y # POSICION EN Y
		
		# OBTENER QUATERNION
		orientation_q = odom.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

		# CONVERTIR QUATERNION A ANGULO DE EULER
		_, _, self.theta_r = euler_from_quaternion(orientation_list)

		#print("x r", self.x_r, " y r", self.y_r, " theta r", self.theta_r)

	def move_to_goal(self): # FUNCION PARA CALCULAR VELOCIDADES LINEALES Y ANGULARES AL GOAL
		d = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) # CALCULAR DISTANCIA AL GOAL
		theta_G = np.arctan2(self.y_target - self.y_r, self.x_target - self.x_r) # CALCULAR ANGULO THETA G
	
		e_theta = theta_G - self.theta_r # CALCULAR ANGULO E_THETA
		e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) # LIMITAR ANGULO DE -PI A PI

		v = self.k_v*d
		w = self.k_w*e_theta

		print("distancia actual " + str(d) + "target tolerance " + str(self.target_position_tolerance))

		return v, w, e_theta

	def calculate_thetafwc(self): #FUNCION PARA CALCULAR ANGULO THETAFWC PARA PODER REALIZAR COMPARACION DE GO TO GOAL A FOLLOW WALL
		
		theta_AO = self.closest_angle + np.pi 
		theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO)) # ARCTAN
		wfwc = -(np.pi/2) + theta_AO
		wfwc = np.arctan2(np.sin(wfwc), np.cos(wfwc))

		return wfwc

	def following_wallc(self):  # FUNCION PARA FOLLOW WALL EN SENTIDO DE MANECILLAS DEL RELOJ

		theta_AO = self.closest_angle + np.pi 
		theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO)) # ARCTAN
		wfwc = -(np.pi/2) + theta_AO
		wfwc = np.arctan2(np.sin(wfwc), np.cos(wfwc))

		vfw = 0.12 # CONSTANTE DE VELOCIDAD LINEAL
		kw = 3.5 # GANANCIA PARA GIRO EN W
		wfw = kw * wfwc 
		return vfw, wfw

	
	def following_wallcc(self):  # FUNCION PARA FOLLOW WALL EN SENTIDO DE MANECILLAS DEL RELOJ
		
		theta_AO = self.closest_angle + np.pi 
		theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO)) # ARCTAN
		wfwcc = np.pi/2 + theta_AO
		wfwcc = np.arctan2(np.sin(wfwcc), np.cos(wfwcc))


		vfw = 0.12 # CONSTANTE DE VELOCIDAD LINEAL
		kw = 3.5 # GANANCIA PARA GIRO EN W
		wfw = kw * wfwcc 
		return vfw, wfw

	def laser_cb(self, msg):
		
		
		## This function receives a message of type LaserScan and computes the closest object direction and range 
		self.closest_range = min(msg.ranges) 
		idx = msg.ranges.index(self.closest_range)
		self.closest_angle = msg.angle_min + idx * msg.angle_increment 
		# Limit the angle to [-pi,pi]
		self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle))
		#print("ang" + str(self.closest_angle))
		self.lidar_received = 1

		"""
		#This function returns the closest object to the robot 

		#This functions receives a ROS LaserScan message and returns the distance and direction to the closest object 

		#returns  closest_range [m], closest_angle [rad], 
		
		new_angle_min = -np.pi/2.0  

		ranges_size=len(msg.ranges) 

		cropped_ranges = msg.ranges[int(ranges_size/4):3*int(ranges_size/4)] 

		min_idx = np.argmin(cropped_ranges) 

		self.closest_range = cropped_ranges[min_idx] 

		self.closest_angle = new_angle_min + min_idx * msg.angle_increment 

		# limit the angle to [-pi, pi]

		self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle)) 
		
		print("range " + str(self.closest_range))
		print("angle " + str(self.closest_angle))

		"""
		

	def wl_cb(self, wl):  
		## This function receives a the left wheel speed [rad/s] 
		self.wl = wl.data 


	def wr_cb(self, wr):  
		## This function receives a the right wheel speed.  
		self.wr = wr.data  
 

	def cleanup(self):  
		#This function is called just before finishing the node  
		# You can use it to clean things up before leaving  
		# Example: stop the robot before finishing a node.    
		vel_msg = Twist()
		self.pub_cmd_vel.publish(vel_msg) 
 
############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
	rospy.init_node("bug_0", anonymous=True)  
	AutonomousNav()