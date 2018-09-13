#!/usr/bin/env python
import sys
import copy
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String
from tf import transformations as homogeneous
import numpy as np
from kinova_msgs.msg import *
from kinova_msgs.srv import *
from sensor_msgs.msg import JointState
import actionlib
import psutil

PROCNAME = "jaco2moveit.py"





## Class to listen to sensor callbacks during execution of motion
class userSafety():

    jointData = []
    jointState = []

    def __init__(self):
            """Variables"""

######################### Subscribers ######################################################
            #Topic for getting joint torques
            #rospy.Subscriber('/j2s7s300_driver/out/joint_torques', JointAngles,self.monitorJointTorques)
            #Topic for getting cartesian force on end effector
            rospy.Subscriber('/j2s7s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, self.monitorToolWrench)




######################### Publishers #######################################################


######################### Actions ##################################################

######################## Services #################################################
            # Service for homing the arm
            home_arm_service = '/j2s7s300_driver/in/home_arm'
            self.homeArmService = rospy.ServiceProxy(home_arm_service, HomeArm)
            print 'Waiting for Stop service'
            rospy.wait_for_service(home_arm_service)
            print 'Stop service server connected'

            #Service for emergency stop
            emergency_service = '/j2s7s300_driver/in/stop'
            self.emergencyStop = rospy.ServiceProxy(emergency_service, Stop)
            print 'Waiting for Stop service'
            rospy.wait_for_service(emergency_service)
            print 'Stop service server connected'

            #Service for restarting the arm
            start_service = '/j2s7s300_driver/in/start'
            self.startArm = rospy.ServiceProxy(start_service, Start)
            print 'Waiting for Start service'
            rospy.wait_for_service(start_service)
            print 'Start service server connected'




####################################################################################
            rospy.spin()



####################Callback functions#########################################################

    # This callback function monitors the Joint Torques and stops the current execution if the Joint Torques exceed certain value
    def monitorJointTorques(self,torques):
        if abs(torques.joint1) > 1:
            self.emergencyStopClient() #Stop arm driver
            rospy.sleep(1.0)
            #self.group.stop() #Stop moveit execution

    # This callback function monitors the Joint Wrench and stops the current
    # execution if the Joint Wrench exceeds certain value
    def monitorToolWrench(self, wrenchStamped):
        toolwrench = abs(wrenchStamped.wrench.force.x**2 + wrenchStamped.wrench.force.y**2 + wrenchStamped.wrench.force.z**2)
        print toolwrench
        if toolwrench > 100:
            self.emergencyStopClient()  # Stop arm driver



#################### Service Clients ##################################################
    ## Home arm service client
    def homeArmClient(self):
        try:
            status = 0
            response = self.homeArmService()
            print response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    ## Emergency Stop Service Client
    def emergencyStopClient(self):
        try:
            response = self.emergencyStop()
            print response
            self.armStopped = True
            for proc in psutil.process_iter():
                print proc
                # check whether the process name matches
                if proc.name == "python":
                    print "Killed"
                    proc.kill()
                    break
            self.startArmClient()
            self.homeArmClient()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    ## Start Service Client
    def startArmClient(self):
        try:
            response = self.startArm()
            print response
            self.armStopped = False
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e




######################################## Methods #######################################



if __name__=='__main__':
    #Initialize node
    rospy.init_node('jaco2moveit',anonymous=True)

    try:
        brain = userSafety()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
