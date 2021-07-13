#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from collections import deque

# A class to receive IMU msgs, interact with the PID controller nodes and return cmd_vel also based on velocity commands sent by independent sources (joystick, move_base, ...)
class Self_Balance:
    def __init__(self):
        
        #Get IMU data and return cmd_vel
        rospy.Subscriber("imu", Imu, self.__balancing_state_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
        # Balancing_PID: Input: desired pitch; Control effort: velocity commands; Feedback: current pitch angle 
        self.b_state_pub = rospy.Publisher("b_state",Float64,queue_size=10)
        self.b_setpoint_pub = rospy.Publisher("b_setpoint", Float64, queue_size=1) #Used only once!
        rospy.Subscriber("b_control_effort", Float64, self.__balancing_control_effort_callback)
        self.current_angle = 0
        
        # Velocity_PID: Input: desired average of velocity commands; Control effort: the Balancing_PID setpoint; Feedback: current average velocity commands
        self.v_state_pub = rospy.Publisher("v_state",Float64,queue_size=10)
        self.v_setpoint_pub = rospy.Publisher("v_setpoint", Float64, queue_size=1)
        self.v_setpoint = 0.0001
        self.ctrl_eff_memory_size = 100
        self.ctrl_eff_memory = deque([0 for i in range(self.ctrl_eff_memory_size)], maxlen=self.ctrl_eff_memory_size)
        self.ctrl_eff_average = 0.0
        
        # Teleop: get Twist msgs and control the angle the robot should balance around in order to move forwards or backwards
        rospy.Subscriber("teleop", Twist, self.__teleop_callback)
        self.max_linear_vel = 1.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        #Aux variables
        self.enable_balancing_PID = False #Allow publishing the setpoint for the balancing_PID once

    # Get IMU msgs; Publish the balancing_PID and the velocity_PID current state; 
    def __balancing_state_callback(self,data):
        quaternion = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w] #get orientation quaternion
        self.current_angle = euler_from_quaternion(quaternion)[0] #extract pitch angle
        self.b_state_pub.publish(Float64(data=self.current_angle)) #publish balancing_PID current state
        
        self.v_setpoint_pub.publish(Float64(data=self.v_setpoint)) #Publish velocity_PID setpoint
        self.v_state_pub.publish(Float64(data=self.ctrl_eff_average)) #publish velocity_PID current state
        
        if not self.enable_balancing_PID: #Publishes the balancing_PID setpoint only once
            self.b_setpoint_pub.publish(Float64(data=0.078)) #Publish balancing_PID setpoint
            self.enable_balancing_PID = True # When True this node will not publish the setpoint for the balancing_PID
    
    # Get the balancing_PID current control effort and publish cmd_vel msgs. Update the control effort average   
    def __balancing_control_effort_callback(self, data):
        # Make Twist msg
        vel = Twist()
        if self.current_angle > 1.35 or self.current_angle < -1.35: # Apply a threshold for not sending cmd_vel msgs if the robot has fallen
            vel.linear.x, vel.linear.y, vel.linear.z = 0.0, 0.0, 0.0 
            vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, 0.0
        else:   
            vel.linear.x, vel.linear.y, vel.linear.z = data.data, 0.0, 0.0 
            vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, self.angular_vel
        
        # Publish cmd_vel msgs        
        self.cmd_vel_pub.publish(vel)
        
        self.ctrl_eff_memory.append(data.data) # Update the balancing_PID control effort readings
        self.ctrl_eff_average = sum(self.ctrl_eff_memory)/self.ctrl_eff_memory_size # Update balancing_PID control effort avarage

    
    #Store velocity commands sent by other sources (joystick, move_base, ...)
    def __teleop_callback(self,data):
        self.angular_vel = data.angular.z*0.5 #This constant is arbitrary
        self.linear_vel = data.linear.x*0.8
        if self.linear_vel > self.max_linear_vel: self.v_setpoint = self.max_linear_vel
        elif self.linear_vel < -self.max_linear_vel: self.v_setpoint = -self.max_linear_vel
        else: self.v_setpoint = self.linear_vel
        


if __name__ == '__main__':
    rospy.init_node('Self_Balance', anonymous=True)
    self_balance = Self_Balance()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS ")
