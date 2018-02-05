#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from kinova_msgs.srv import *
from kinova_msgs.msg import *

class subscribeAndPublish():
    def __init__(self):
        """ Global variable """
        self.currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq

        #Topic for publishing
        self.pub = rospy.Publisher('/command_response', String, queue_size=10)

        #Topic for subscribing
        self.sub = rospy.Subscriber('/command',String, self.callback)

        #Target pose for commanding the robot
        self.targetPose = KinovaPose()
        self.targetPose.X = self.currentCartesianCommand[0]
        self.targetPose.Y = self.currentCartesianCommand[1]
        self.targetPose.Z = self.currentCartesianCommand[2]
        self.targetPose.ThetaX = self.currentCartesianCommand[3]
        self.targetPose.ThetaY = self.currentCartesianCommand[4]
        self.targetPose.ThetaZ = self.currentCartesianCommand[5]

    #Send the target position to add_pose_to_Cartesian_trajectory. The fields are: 0:NA, 1-9: Rotation Matrix, 10-12: Position, 13: Extrusion, 14: Cooling, 15:Pause, 16: Speed, 17: TypeOfCurve
    def callback(self, message):
        print 'Receieved:%s'%message.data
        fields = message.data.split(',')
        if len(fields) == 18:
            self.targetPose.X = fields[10]
            self.targetPose.Y = fields[11]
            self.targetPose.Z = fields[12]
            pause = fields[15]

            #Add targetPose to robot trajectory using the service
            print 'Requesting add_pose_to_Cartesian_trajectory service'
            self.addPoseToCartesianTrajectoryClient(self.targetPose)

            #Pause robot
            rospy.sleep(rospy.Duration(int(pause),0))

        self.pub.publish('Motion.\r')

    def addPoseToCartesianTrajectoryClient(self,targetPose):
        print 'Waiting for the server...'
        rospy.wait_for_service('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory')
        try:
            addPoseToCartesianTrajectory = rospy.ServiceProxy('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory', AddPoseToCartesianTrajectory)
            resp1 = addPoseToCartesianTrajectory(targetPose.X,targetPose.Y,targetPose.Z,targetPose.ThetaX,targetPose.ThetaY,targetPose.ThetaZ)
            print resp1
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ =='__main__':
    #Initialize node
    rospy.init_node('SubscribeAndPublish')

    try:
        subPub = subscribeAndPublish()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
