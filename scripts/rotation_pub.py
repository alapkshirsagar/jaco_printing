#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np

pub = rospy.Publisher('/command', String, queue_size=10)
rospy.init_node('rotation_pub')
r = rospy.Rate(10) # 10hz

#pub.publish("0,"+str(30*np.cos(60*np.pi/180))+"," + str(-30*np.sin(60*np.pi/180))+",40,0.707,0,0.707,0,1,0,9,1,80")
pub.publish("0,"+str(30*np.cos(21*np.pi/180))+"," + str(-30*np.sin(21*np.pi/180))+",40,0,0,0,1,0,0,2,1,80")

rospy.spin()
