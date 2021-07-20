#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

#A class to make a controlled frequency publisher
class Freq_Pub: 
    def __init__(self):
        self.pub = rospy.Publisher("pub_topic", Float64, queue_size=1)
        self.sub = rospy.Subscriber("sub_topic", Float64, self.callback)
        self.msg = Float64(data=0.085) #0.085 is just a convenient initial value
    
    def callback(self, data):
        self.msg = Float64(data=data.data)
        
    def publish_msg(self):
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('freq_pub', anonymous=True)
    fp = Freq_Pub()
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        fp.publish_msg()
        rate.sleep()