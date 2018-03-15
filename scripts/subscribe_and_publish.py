#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from kinova_msgs.srv import *
from kinova_msgs.msg import *

class subscribeAndPublish():
    def __init__(self):
        """ Global variable """
        self.homePosition = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq

        #Topic for publishing
        self.pub = rospy.Publisher('/command_response', String, queue_size=10)

        #Topic for subscribing
        self.sub = rospy.Subscriber('/command',String, self.callback)

        #Target pose for commanding the robot
        self.targetPose = KinovaPose()
        self.targetPose.X = self.homePosition[0]
        self.targetPose.Y = self.homePosition[1]
        self.targetPose.Z = self.homePosition[2]
        self.targetPose.ThetaX = self.homePosition[3]
        self.targetPose.ThetaY = self.homePosition[4]
        self.targetPose.ThetaZ = self.homePosition[5]
        self.receivedCounter = 0
        self.sentCounter = 0

        #Service for sending trajectory points to the robot
        self.addPoseToCartesianTrajectory = rospy.ServiceProxy('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory', AddPoseToCartesianTrajectory)



    #Send the target position to add_pose_to_Cartesian_trajectory. The fields are: 0:NA, 1-9: Rotation Matrix, 10-12: Position, 13: Extrusion, 14: Cooling, 15:Pause, 16: Speed, 17: TypeOfCurve
    def callback(self, message):
        self.receivedCounter = self.receivedCounter+1
        print 'Received Counter:%i'%self.receivedCounter
        fields = message.data.split(',')
        if len(fields) == 10:
            self.targetPose.X = self.homePosition[0]+float(fields[1])/1000
            self.targetPose.Y = self.homePosition[1]+float(fields[2])/1000
            self.targetPose.Z = self.homePosition[2]+float(fields[3])/1000
            self.targetPose.ThetaX = float(fields[4])*3.14/180
            self.targetPose.ThetaY = float(fields[5])*3.14/180
            self.targetPose.ThetaZ = float(fields[6])*3.14/180
            pause = fields[7]
            MaxTranslationVelocity = 0.05

            #Add targetPose to robot trajectory using the service
            #print 'Requesting add_pose_to_Cartesian_trajectory service'
            self.addPoseToCartesianTrajectoryClient(self.targetPose,MaxTranslationVelocity,pause)


    def addPoseToCartesianTrajectoryClient(self,targetPose,MaxTranslationVelocity,pause):
        try:
            resp1 = self.addPoseToCartesianTrajectory(targetPose.X,targetPose.Y,targetPose.Z,targetPose.ThetaX,targetPose.ThetaY,targetPose.ThetaZ,MaxTranslationVelocity)
            #print resp1
            #Pause robot
            print "Pausing for"
            print rospy.Duration(int(pause),0)
            rospy.sleep(rospy.Duration(int(pause),0))
            self.pub.publish('Motion.\r')
            self.sentCounter = self.sentCounter+1
            print 'Sent Counter:%i'%self.sentCounter
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ =='__main__':
    #Initialize node
    rospy.init_node('SubscribeAndPublish')
    print 'Waiting for the server...'
    rospy.wait_for_service('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory')
    print 'Server Connected'

    try:
        subPub = subscribeAndPublish()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
