#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


#########################################
class teleop(object):
    """
    teleoperation
    """
    def __init__(self):

        self.sub_joy   = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1)
        self.pub_cmd   = rospy.Publisher("ctl_ref", Twist , queue_size=1  ) 

        self.max_vel  = rospy.get_param('~max_vel',   6.0) # Max linear velocity (m/s)
        self.max_volt = rospy.get_param('~max_volt',  8)   # Max voltage is set at 6 volts   
        self.maxStAng = rospy.get_param('~max_angle', 40)  # Supposing +/- 40 degrees max for the steering angle
        self.cmd2rad   = -self.maxStAng*2*3.1416/360     

    ####################################### 
        
    def joy_callback( self, joy_msg ):
        """ """
    
        propulsion_user_input = joy_msg.axes[3]    # Up-down Right joystick 
        steering_user_input   = joy_msg.axes[0]    # Left-right left joystick
        
        self.cmd_msg = Twist()             
                
        # Software deadman switch
        #If left button is active 
        if (joy_msg.buttons[4]):
            
            #If right button is active       
            if (joy_msg.buttons[5]):   
                # Fully Open-Loop
                self.cmd_msg.linear.x  = propulsion_user_input * self.max_volt #[volts]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z  = 1   #CtrlChoice
                
            #If button A is active 
            elif(joy_msg.buttons[1]):   
                
                # Closed-loop position, Open-loop sterring
                self.cmd_msg.linear.x  = propulsion_user_input # [m]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z  = 3  # Control mode
                
            #If button B is active 
            elif(joy_msg.buttons[2]):   
                
                # Autopilot
                # Closed-loop velocity, closed-loop steering sterring
                self.cmd_msg.linear.x  = 2.0 #[m/s]
                self.cmd_msg.angular.z = 0.0 
                self.cmd_msg.linear.z  = 5  # Control mode
                
            #If button x is active 
            elif(joy_msg.buttons[0]):   
                
                # Autopilot parking
                # Closed-loop velocity closed-loop sterring
                self.cmd_msg.linear.x  = 2.0 #[m]
                self.cmd_msg.angular.z = 0.0
                self.cmd_msg.linear.z  = 6  # Control mode
                
            #If button y is active 
            elif(joy_msg.buttons[3]):   
                
                # Reset Encoder
                self.cmd_msg.linear.x  = 0
                self.cmd_msg.angular.z = 0
                self.cmd_msg.linear.z  = 4  # Control mode
                
            #If left trigger is active 
            elif (joy_msg.buttons[6]):
                
                # Template
                self.cmd_msg.linear.x  = 0
                self.cmd_msg.angular.z = 0
                self.cmd_msg.linear.z  = 0 # Control mode
                
            #If both joy pushed
            elif(joy_msg.buttons[11] and joy_msg.buttons[12]):
                # Joystick disabled!
                return;
                
            #If bottom arrow is active
            elif(joy_msg.axes[5]):
                
                # Template
                self.cmd_msg.linear.x  = 0
                self.cmd_msg.angular.z = 0
                self.cmd_msg.linear.z  = 0 # Control mode

            # Defaults operation
            # No active button
            else:
                # Closed-loop velocity, Open-loop sterring
                self.cmd_msg.linear.x  = propulsion_user_input * self.max_vel #[m/s]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
                self.cmd_msg.linear.z  = 0  # Control mode
        
        # Deadman is un-pressed
        else:
            
            self.cmd_msg.linear.x = 0 
            self.cmd_msg.linear.y = 0
            self.cmd_msg.linear.z = -1            
            self.cmd_msg.angular.x = 0
            self.cmd_msg.angular.y = 0
            self.cmd_msg.angular.z = 0 


        # Publish cmd msg
        self.pub_cmd.publish( self.cmd_msg )
            

#########################################
if __name__ == '__main__':
    
    rospy.init_node('teleop',anonymous=False)
    node = teleop()
    rospy.spin()
