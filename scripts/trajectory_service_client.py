#!/usr/bin/env python

import sys
import rospy
from kinova_msgs.srv import *
from kinova_msgs.msg import *
from std_msgs.msg import String

""" Global variable """
currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq

def clear_trajectories_client():
    rospy.wait_for_service('/j2s7s300_driver/in/clear_trajectories')
    try:
        clear_trajectories = rospy.ServiceProxy('/j2s7s300_driver/in/clear_trajectories', ClearTrajectories)
        resp1 = clear_trajectories()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def add_pose_to_Cartesian_trajectory_client(targetPose):
    rospy.wait_for_service('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory')
    try:
        add_pose_to_Cartesian_trajectory = rospy.ServiceProxy('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory', AddPoseToCartesianTrajectory)
        resp1 = add_pose_to_Cartesian_trajectory(targetPose.X,targetPose.Y,targetPose.Z,targetPose.ThetaX,targetPose.ThetaY,targetPose.ThetaZ)
        print resp1
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getcurrentCartesianCommand():
    # wait to get current position
    topic_address = '/j2s7s300_driver/out/cartesian_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, setcurrentCartesianCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
    print 'position listener obtained message for Cartesian pose. '

def setcurrentCartesianCommand(feedback):
    global currentCartesianCommand

    currentCartesianCommand_str_list = str(feedback).split("\n")

    for index in range(0,len(currentCartesianCommand_str_list)):
        temp_str=currentCartesianCommand_str_list[index].split(": ")
        currentCartesianCommand[index] = float(temp_str[1])
    # the following directly reading only read once and didn't update the value.
    # currentCartesianCommand = [feedback.X, feedback.Y, feedback.Z, feedback.ThetaX, feedback.ThetaY, feedback.Z]
    # print 'currentCartesianCommand in setcurrentCartesianCommand is: ', currentCartesianCommand

def windowsSubscriber():
    topic_address = '/chatter'
    rospy.Subscriber(topic_address,String,call_trajectory_client)
    rospy.spin()

def call_trajectory_client(feedback):
    fields = feedback.data.split(',')
    targetPose.X = currentCartesianCommand[0]+ float(fields[0])
    targetPose.Y = currentCartesianCommand[1]+ float(fields[1])
    targetPose.Z = currentCartesianCommand[2]+ float(fields[2])
    targetPose.ThetaX = currentCartesianCommand[3]+ float(fields[3])
    targetPose.ThetaY = currentCartesianCommand[4]+ float(fields[4])
    targetPose.ThetaZ = currentCartesianCommand[5]+ float(fields[5])
    print "Requesting to add point to trajectory"
    add_pose_to_Cartesian_trajectory_client(targetPose)



if __name__ == "__main__":
    rospy.init_node('trajectory_service_client')
    #getcurrentCartesianCommand()
    #print currentCartesianCommand

    #print "Requesting to clear trajectories"
    #clear_trajectories_client()

    windowsSubscriber()
    # targetPose = KinovaPose()
    # file = open('poses.txt')
    # for line in file:
    #     fields = line.strip().split()
    #     print fields[0], fields[1], fields[2], fields[3], fields[4], fields[5]
    #     targetPose.X = currentCartesianCommand[0]+ float(fields[0])
    #     targetPose.Y = currentCartesianCommand[1]+ float(fields[1])
    #     targetPose.Z = currentCartesianCommand[2]+ float(fields[2])
    #     targetPose.ThetaX = currentCartesianCommand[3]+ float(fields[3])
    #     targetPose.ThetaY = currentCartesianCommand[4]+ float(fields[4])
    #     targetPose.ThetaZ = currentCartesianCommand[5]+ float(fields[5])
    #     print "Requesting to add point to trajectory"
    #     add_pose_to_Cartesian_trajectory_client(targetPose)
    #     #
    # targetPose.X = 0.21
    # targetPose.Y = -0.25
    # targetPose.Z = 0.51
    # targetPose.ThetaX = 1.64
    # targetPose.ThetaY = 1.11
    # targetPose.ThetaZ = 0.13
    # print "Requesting to add point to trajectory"
    # add_pose_to_Cartesian_trajectory_client(targetPose)
