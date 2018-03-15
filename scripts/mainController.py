#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from kinova_msgs.srv import *
from kinova_msgs.msg import *

## Class to communicate between all components: Rhino Plugin, Jaco2, Arduino
class mainController():
    def __init__(self):
        """ Global variable """
        self.homePosition = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home position of jaco2 in unit mq

        # Target pose for commanding the robot
        self.targetPose = KinovaPose()
        self.targetPose.X = self.homePosition[0]
        self.targetPose.Y = self.homePosition[1]
        self.targetPose.Z = self.homePosition[2]
        self.targetPose.ThetaX = self.homePosition[3]
        self.targetPose.ThetaY = self.homePosition[4]
        self.targetPose.ThetaZ = self.homePosition[5]

        self.receivedCounter = 0 #Keep count of messages received from RhinoPlugin
        self.sentCounter = 0 #Keep count of responses sent to RhinoPlugin
        self.extruderTemperature = 0 #Temperature of extruder TODO Unit and Initial Value?


############# Publishers #######################################################
        #Topic for sending response to RhinoPlugin
        self.rhinoPub = rospy.Publisher('/command_response', String, queue_size=10)

        #Topic for sending commands to Arduino
        self.arduinoPub = rospy.Publisher('/extruder', String, queue_size=10)


############ Subscribers ######################################################
        #Topic for getting commands from RhinoPlugin
        rospy.Subscriber('/command',String, self.getRhinoCommands)

        #Topic for getting response from Arduino
        rospy.Subscriber('/chatter',String, self.getArduinoResponse)

########### Services ##########################################################
        #Service for sending trajectory points to the robot
        self.addPoseToCartesianTrajectory = rospy.ServiceProxy('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory', AddPoseToCartesianTrajectory)

###############################################################################

    #Send the target position to add_pose_to_Cartesian_trajectory. The fields are: 0:NA, 1-9: Rotation Matrix, 10-12: Position, 13: Extrusion, 14: Cooling, 15:Pause, 16: Speed, 17: TypeOfCurve
    def getRhinoCommands(self, message):
        self.receivedCounter = self.receivedCounter+1
        print 'Received Counter:%i'%self.receivedCounter
        fields = message.data.split(',')
        if len(fields) == 10:
            # Send extruder commands to arduino
            self.arduinoPub.publish("G1 E"+fields[13])

            # Send motion commands to robot
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
            self.rhinoPub.publish('Motion.\r')
            self.sentCounter = self.sentCounter+1
            print 'Sent Counter:%i'%self.sentCounter
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def getArduinoResponse(self,message):
        #TODO Update temperature of extruder
        print message

    def heatExtruder(self):
        self.arduinoPub.publish("M104 S270")
        #TODO wait till temperature reaches 270


if __name__ =='__main__':
    #Initialize node
    rospy.init_node('mainController')
    print 'Waiting for the server...'
    rospy.wait_for_service('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory')
    print 'Server Connected'

    try:
        brain = mainController()
        brain.heatExtruder()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
