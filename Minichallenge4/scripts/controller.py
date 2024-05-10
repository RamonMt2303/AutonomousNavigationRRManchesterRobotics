#!/usr/bin/env python3

import rospy  

from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32 

import numpy as np 


#This class will make the puzzlebot move following a square 
class GoToGoal():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 

        ############ ROBOT CONSTANTS ################  

        r=0.05 #wheel radius [m] 
        L=0.19 #wheel separation [m] 

        ###########  INIT PUBLISHERS ################ 

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        self.pub_flag = rospy.Publisher('flag',Float32,queue_size=1)
        self.pub_position = rospy.Publisher('position', Pose, queue_size=1)  

        ############## SUBSCRIBERS ##################  

        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("set_point", Pose, self.goal_cb)


        #### Init variables ## 
        
        self.x_goal = 0.0
        self.y_goal = 0.0

        xG=self.x_goal #x position  

        yG=self.y_goal #y postion of the goal 

 
        kp_l = 0.3
        ki_l = 0.00001
        kd_l = 0.0

        kp_a = 4.8
        ki_a = 0.0000001
        kd_a = 0.00000006

        self.e=[0.0,0.0,0.0]
        #e[0] error actual    e[1] error anterior    e[2] error dos veces anterior
        self.u=[0.0,0.0]
        #u[0] salida actual    u[1] salida anterior

        self.e_a=[0.0,0.0,0.0]
        #e[0] error actual    e[1] error anterior    e[2] error dos veces anterior
        self.u_a=[0.0,0.0]
        #u[0] salida actual    u[1] salida anterior

        Ts = 0.02 
        #periodo de muestreo

        #ganancias del modelo discreto

        self.lim = 1000

        self.K1=kp_l + Ts*ki_l + kd_l/Ts
        self.K2=-kp_l - 2.0*kd_l/Ts
        self.K3=kd_l/Ts

        self.K1_a=kp_a + Ts*ki_a + kd_a/Ts
        self.K2_a=-kp_a - 2.0*kd_a/Ts
        self.K3_a=kd_a/Ts

        v_msg=Twist() 
        position = Pose()

        xr = 0.0  # [m] robot's postion along the x-axis 
        yr = 0.0  # [m] robot's postion along the y-axis 
        thetar=0.0 # [rad] robot's orientation  
        v = 0.0 # Robot's linear speed [m/s] 
        w = 0.0 # Robot's angular speed [rad/s] 
        self.wl = 0.0 # left wheel angular speed [rad/s] 
        self.wr = 0.0 # right wheel angular speed [rad/s] 
        d=2.0 
        d_min = 0.1 #[m] d_min to the goal to declare the robot arrived  

        while rospy.get_time == 0: 
            print("No simulated time has been received") 
        print("I got a valid time") 

        previous_time = rospy.get_time() #I will use this to compute delta_t 
        rate = rospy.Rate(1/Ts) #20Hz  
        print("Node initialized") 

        while not rospy.is_shutdown(): 
            xG=self.x_goal #x position  
            yG=self.y_goal #y postion of the goal 
            print(xG)
            print(yG)
            
            delta_t = rospy.get_time() - previous_time 
            previous_time = rospy.get_time() 
            v=r*(self.wl+self.wr)/2.0 
            w=r*(self.wr - self.wl)/L 
            thetar = thetar + w*delta_t 
            thetar = np.arctan2(np.sin(thetar), np.cos(thetar)) # set thetar between -pi and pi  
            xr = xr + v*np.cos(thetar)*delta_t 
            yr = yr + v*np.sin(thetar)*delta_t 

            #distance to the goal 

            d=np.sqrt((xG-xr)**2+(yG-yr)**2) 

            #Angle to the goal  

            theta_g = np.arctan2(yG-yr,xG-xr) 
            e_theta =theta_g-thetar 

            #crop e_theta from -pi to pi  

            e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 

            # Inicia control pid
            # Velocidad lineal
            self.e[0]=d
            self.u[0]=self.K1*self.e[0]+self.K2*self.e[1]+self.K3*self.e[2]+self.u[1]
            self.e[2]=self.e[1]
            self.e[1]=self.e[0]
            self.u[1]=self.u[0]

            #Velocidad angular
            self.e_a[0]= e_theta
            self.u_a[0]=self.K1_a*self.e_a[0]+self.K2_a*self.e_a[1]+self.K3_a*self.e_a[2]+self.u_a[1]
            self.e_a[2]=self.e_a[1]
            self.e_a[1]=self.e_a[0]
            self.u_a[1]=self.u_a[0]

            #Fill the message 

            v_msg.linear.x = self.u[0]
            v_msg.angular.z = self.u_a[0]

            position.position.x = xr
            position.position.y = yr 
            
            print("d: ",d)
            print("e: ",e_theta)
            print("v: " + str(v)) 
            print("w: " + str(w)) 

            if (d<=d_min):
                self.pub_flag.publish(1)
                v_msg.linear.x = 0.0 
                v_msg.angular.z = 0.0 
                rate.sleep()
            
            self.pub_position.publish(position)
            self.pub_cmd_vel.publish(v_msg) 
            
            rate.sleep()  

    def goal_cb(self, msg):
        #print('Entra a callback')
        self.x_goal = msg.position.x
        self.y_goal = msg.position.y

    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data 
        
    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data  

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("go_to_goal", anonymous=True)  
    GoToGoal()  