#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class PID_redirection:
    def __init__(self):
        self.state_pub = rospy.Publisher("/state",Float64,queue_size=10)
        self.setpoint_pub = rospy.Publisher("/setpoint", Float64, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
        self.current_angle = 0
    
        self.setpoint = rospy.get_param("~setpoint")
        self.setpoint_limit = 0.5  
        self.state_verifier = 0
        self.state_limit = 10
        self.old_pitch = 0
        self.setpoint_pub.publish(Float64(data=self.setpoint)) #Publish setpoint

    def __state_callback(self,data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard IMU")
        
        quaternion = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w] #get orientation quaternion
        
        self.current_angle = euler_from_quaternion(quaternion)[0] #extract pitch angle
        
        self.state_pub.publish(Float64(data=self.current_angle))
        
        # #TODO Setpoint control
        # if self.current_angle > self.old_pitch: self.state_verifier +=1
        # else: self.state_verifier -=1
        
        # if self.state_verifier >= self.state_limit and self.setpoint >= -self.setpoint_limit: 
        #     self.setpoint -= 1
        #     self.state_verifier = 0
        #     rospy.loginfo("Setpoint -> {}".format(self.setpoint))
        # elif self.state_verifier <= -self.state_limit and self.setpoint <= self.setpoint_limit:
        #     self.setpoint += 1
        #     self.state_verifier = 0
        # self.old_pitch = self.current_angle    
        
        self.setpoint = rospy.get_param("~setpoint")
        self.setpoint_pub.publish(Float64(data=self.setpoint)) #Publish setpoint
        # rospy.loginfo(rospy.get_caller_id() + "Published state")
        
    def __control_effort_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard control effort")
        vel = Twist()
        if self.current_angle > 1.35 or self.current_angle < -1.35:
            vel.linear.x, vel.linear.y, vel.linear.z = 0.0, 0.0, 0.0 
            vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, 0.0
        else:   
            vel.linear.x, vel.linear.y, vel.linear.z = data.data, 0.0, 0.0 
            vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, 0.0
                
        self.cmd_vel_pub.publish(vel)
        # rospy.loginfo(rospy.get_caller_id() + "Published cmd_vel")
        
    def redirect(self):

        rospy.Subscriber("/control_effort", Float64, self.__control_effort_callback)
        rospy.Subscriber("imu", Imu, self.__state_callback)
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('PID_redirect', anonymous=True)
    
    balancing_PID = PID_redirection()
    balancing_PID.redirect()