#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np

pub = rospy.Publisher('/command', String, queue_size=10)
rospy.init_node('rotation_pub')
r = rospy.Rate(10) # 10hz

pub.publish("0,"+str(30*np.cos(47*np.pi/180))+"," + str(-30*np.sin(47*np.pi/180))+",1,0,0,0,1,1,0,9,1,80")
rospy.spin()
