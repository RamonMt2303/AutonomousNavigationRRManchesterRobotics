#!/usr/bin/env python 
import rospy 
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan   #Lidar
import numpy as np

#This class will make the puzzlebot move to a given goal
class GoToGoal(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup)
        ############ Variables ###############
        targets = [[0.51,2.08],[2.39,4.77],[1.79,3.57],[1.53,1.30],[1.42,0.21]] 
        self.x_target=0.0
        self.y_target=0.0
        #self.x_target=1.42 #x position of the goal
        #self.y_target=0.21 #y position of the goal
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_received=1 #flag to indicate if the goal has been received
        self.lidar_received = False #flag to indicate if the laser scan has been received
        self.target_position_tolerance=0.10#0.08 #target position tolerance [m]
        fw_distance = 0.23 #0.2 # distance to activate the following walls behavior [m]
        progress = 0.15 #0.3 #If the robot is this close to the goal with respect to when it started following walls it will stop following walls
        v_msg=Twist() #Robot's desired speed 
        self.wr=0 #right wheel speed [rad/s]
        self.wl=0 #left wheel speed [rad/s]
        self.current_state = 'GoToGoal' #Robot's current state
        current_goal =  0 #current goal index
        rospy.on_shutdown(self.cleanup) 
        ###******* INIT PUBLISHERS *******###
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)
        rospy.Subscriber("miu",Float32MultiArray,self.cb_get_miu)

        #********** INIT NODE **********### 
        freq=10
        rate = rospy.Rate(freq) #freq Hz 
        Dt =1.0/float(freq) #Dt is the time between one calculation and the next one
        print("Node initialized")
        print("Please send a Goal from rviz using the button: 2D Nav Goal")
        print("You can also publish the goal to the (move_base_simple/goal) topic.")
        ################ MAIN LOOP ################ 
        
        while not rospy.is_shutdown(): 

            self.x_target = targets[current_goal][0]
            self.y_target = targets[current_goal][1]
            
            if self.lidar_received:
                self.x_robot = self.x
                self.y_robot = self.y
                self.theta_robot = self.theta
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle
                thetaAO = self.get_theta_ao(closest_angle)
                thetaGTG =self.get_theta_gtg(self.x_target, self.y_target, self.x_robot, self.y_robot, self.theta_robot)
                d_t=np.sqrt((self.x_target-self.x_robot)**2+(self.y_target-self.y_robot)**2)    # distance to goal

                if self.at_goal(): 
                    print("Goal reached")
                    self.current_state = 'Stop'
                    #print("Stop")
                    v_msg.linear.x = 0
                    v_msg.angular.z = 0
                elif self.current_state == 'GoToGoal':                 
                    if closest_range <= fw_distance:
                        # Implement the following walls behavior
                        d_tao = d_t
                        print(" change to following walls")
                        thetaFWC=self.get_theta_fw(thetaAO, True) #If it True is passed return clockwise, else counterclockwise
                        if abs(thetaFWC - thetaGTG) <= np.pi/2:
                            flag = 1
                        else:
                            flag = 0
                        self.current_state = "Clockwise"                   
                    else:
                        print("Moving to the Goal")
                        v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.x_robot, self.y_robot, self.theta_robot)
                        v_msg.linear.x = v_gtg
                        v_msg.angular.z = w_gtg

                elif self.current_state == 'Clockwise':
                    if d_t < abs(d_tao - progress) and abs(thetaAO - thetaGTG) < np.pi/2: #ADD output condition#:
                        self.current_state = 'GoToGoal'
                        print("Change to Go to goal")
                    else:
                        thetaFWC=self.get_theta_fw(thetaAO, True) #If it True is passed return clockwise, else counterclockwise
                        if flag == 1:
                            thetaFWC=thetaFWC
                        else:
                            thetaFWC=self.get_theta_fw(thetaAO, False)
                        vFWC, wFWC = self.compute_fw_control(thetaFWC)
                        v_msg.linear.x = vFWC
                        v_msg.angular.z = wFWC                        
                
                if self.current_state == 'Stop':
                    print("Stop")
                    v_msg.linear.x = 0
                    v_msg.angular.z = 0
                    current_goal += 1

                    if (current_goal > len(targets)):
                        v_msg.linear.x = 0
                        v_msg.angular.z = 0
                        self.cleanup()
                    else:
                        print("New Goal Received")
                        self.current_state = 'GoToGoal'
                        
            print("x:",self.x," y:",self.y)
                
            self.pub_cmd_vel.publish(v_msg) 
            rate.sleep() 
    
    def at_goal(self):
        #This function returns true if the robot is close enough to the goal
        #This functions receives the goal's position and returns a boolean
        #This functions returns a boolean
        return np.sqrt((self.x_target-self.x_robot)**2+(self.y_target-self.y_robot)**2)<self.target_position_tolerance

    def get_closest_object(self, lidar_msg):
        #This function returns the closest object to the robot
        #This functions receives a ROS LaserScan message and returns the distance and direction to the closest object
        #returns  closest_range [m], closest_angle [rad],
        min_idx = np.argmin(lidar_msg.ranges)
        closest_range = lidar_msg.ranges[min_idx]
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment
        # limit the angle to [-pi, pi]
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))
        return closest_range, closest_angle

    
    def get_theta_gtg(self, x_target, y_target, x_robot, y_robot, theta_robot):
        #This function returns the angle to the goal
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot)
        e_theta=theta_target-theta_robot
        #limit e_theta from -pi to pi
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
        return e_theta

    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot):
        #This function returns the linear and angular speed to reach a given goal
        #This functions receives the goal's position (x_target, y_target) [m]
        #  and robot's position (x_robot, y_robot, theta_robot) [m, rad]
        #This functions returns the robot's speed (v, w) [m/s] and [rad/s]
        kvmax = 0.16 #0.17 #linear speed maximum gain 
        kwmax=0.8#0.8 #angular angular speed maximum gain
        #kw=0.5
        av = 2.0 #Constant to adjust the exponential's growth rate  
        aw = 2.0 #Constant to adjust the exponential's growth rate
        ed=np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2)
        #Compute angle to the target position
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot)
        e_theta=theta_target-theta_robot

        #limit e_theta from -pi to pi
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        #Compute the robot's angular speed
        if e_theta != 0:
            kw=kwmax*(1-np.exp(-aw*e_theta**2))/abs(e_theta) #Constant to change the speed 
        else:
            kw = 0.05#0.08 #0.05
        w=kw*e_theta
        if abs(e_theta) > np.pi/8:
            #we first turn to the goal
            v=0 #linear speed 
        else:
            # Make the linear speed gain proportional to the distance to the target position
            kv=kvmax*(1-np.exp(-av*ed**2))/abs(ed) #Constant to change the speed 
            v=kv*ed #linear speed 
        return v,w

    def get_theta_ao(self, theta_closest):
        ##This function returns the angle for the Avoid obstacle behavior 
        # theta_closest is the angle to the closest object [rad]
        #This functions returns the angle for the Avoid obstacle behavior [rad]
        ############################################################
        thetaAO=theta_closest-np.pi
        #limit the angle to [-pi,pi]
        thetaAO = np.arctan2(np.sin(thetaAO),np.cos(thetaAO))
        return thetaAO

 
    def get_theta_fw(self, thetaAO, clockwise):
        ## This function computes the linear and angular speeds for the robot
        # It receives thetaAO [rad] and clockwise [bool]
        if clockwise:
            thetaFW = -np.pi/2 + thetaAO
        else:
            thetaFW = np.pi/2 + thetaAO
        thetaFW = np.arctan2(np.sin(thetaFW),np.cos(thetaFW))
        return thetaFW 
    
    def compute_fw_control(self, thetaFW):
        ## This function computes the linear and angular speeds for the robot
        # It receives thetaFW [rad]   
        #Compute linear and angular speeds
        kw = 1.5#1.7 #1.5
        vFWC = 0.08#0.10 #0.08 #constante
        wFWC = kw * thetaFW
        return vFWC, wFWC
    
    def get_angle(self, idx, angle_min, angle_increment): 
        ## This function returns the angle for a given element of the object in the lidar's frame 
        angle= angle_min + idx * angle_increment 
        # Limit the angle to [-pi,pi] 
        angle = np.arctan2(np.sin(angle),np.cos(angle)) 
        return angle 
    
    def polar_to_cartesian(self,r,theta): 
        ## This function converts polar coordinates to cartesian coordinates 
        x = r*np.cos(theta) 
        y = r*np.sin(theta) 
        return (x,y) 
    
    def laser_cb(self, msg):  
        ## This function receives a message of type LaserScan  
        self.lidar_msg = msg 
        self.lidar_received = True 

    def wl_cb(self, wl): 
        ## This function receives a the left wheel speed [rad/s]
        self.wl = wl.data
        
    def wr_cb(self, wr): 
        ## This function receives a the right wheel speed. 
        self.wr = wr.data 
    
    def goal_cb(self, goal): 
        ## This function receives a the goal from rviz. 
        print("Goal received I'm moving to x= "+str(goal.pose.position.x)+" y= "+str(goal.pose.position.y))
        self.current_state = "GoToGoal"
        # assign the goal position
        self.x_target = goal.pose.position.x
        self.y_target = goal.pose.position.y
        self.goal_received=1
        
    def cb_get_miu(self,msg):
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.theta = msg.data[2]

    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        vel_msg = Twist()
        self.pub_cmd_vel.publish(vel_msg)

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("bug_0", anonymous=True) 
    GoToGoal() 

