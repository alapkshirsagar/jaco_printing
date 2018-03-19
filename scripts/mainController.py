#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import tf
from std_msgs.msg import String
from kinova_msgs.srv import *
from kinova_msgs.msg import *

## Class to communicate between all components: Rhino Plugin, Jaco2, Arduino
class mainController():
    def __init__(self):
        """ Global variable """
        self.homePosition = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home position of jaco2 in unit md
        self.newOrigin = [-0.0132637824863,-0.532912909985,0.553722143173,np.pi/2,0,0] # new origin corresponds to table height and extruder pointing downward

        # Target pose for commanding the robot
        self.targetPose = KinovaPose()
        self.targetPose.X = self.newOrigin[0]
        self.targetPose.Y = self.newOrigin[1]
        self.targetPose.Z = self.newOrigin[2]
        self.targetPose.ThetaX = self.newOrigin[3]
        self.targetPose.ThetaY = self.newOrigin[4]
        self.targetPose.ThetaZ = self.newOrigin[5]

        self.receivedCounter = 0 #Keep count of messages received from RhinoPlugin
        self.sentCounter = 0 #Keep count of responses sent to RhinoPlugin
        self.extruderTemperatureFlag = False #True if "CurrentTemperature" message is received
        self.extruderTemperature = 0 #Temperature of extruder
        self.extruderDesiredTemperature = 270 #Desired temperature of extruder

        self.rospack = rospkg.RosPack()
        self.packagePath = self.rospack.get_path('jaco_printing') #Get directory path to jaco_printing package



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
        # Startup Procedure : Heat Extruder
        if rospy.get_param('~heatExtruder'):
            self.heatExtruder()

        # Startup Procedure: Move Robot to New Origin
        if rospy.get_param('~moveArm'):
            self.addPoseToCartesianTrajectoryClient(self.targetPose,0.05,0)

            # Continue with rest of the procedure
            if rospy.get_param('~rhinoPlugin'):
                #//TODO Call rhino plugin connection
                print "Rhino Plugin needs to be connected"
            else:
                self.commandJacoTextFile()
        rospy.spin()
###############################################################################
####################Callback functions#########################################

    #This callback function is used to control Jaco and Extruder when Rhino Plugin in connected.
    #Send the target position to add_pose_to_Cartesian_trajectory. The fields of 'message' are: (0,X,Y,Z,Rx,Ry,Rz,Extrusion,Cooling,Pause,Speed,TypeOfCurve)
    def getRhinoCommands(self, message):
        self.receivedCounter = self.receivedCounter+1
        print 'Received Counter:%i'%self.receivedCounter
        fields = message.data.split(',')
        if len(fields) == 12:
            print fields
            self.targetPose.X = self.newOrigin[0]+float(fields[1])/1000
            self.targetPose.Y = self.newOrigin[1]+float(fields[2])/1000
            self.targetPose.Z = self.newOrigin[2]+float(fields[3])/1000
            self.targetPose.ThetaX = self.newOrigin[3]+float(fields[4])*3.14/180
            self.targetPose.ThetaY = self.newOrigin[4]+float(fields[5])*3.14/180
            self.targetPose.ThetaZ = self.newOrigin[5]+float(fields[6])*3.14/180
            pause = fields[9]
            MaxTranslationVelocity = 0.05

            #Add targetPose to robot trajectory using the service
            #print 'Requesting add_pose_to_Cartesian_trajectory service'
            self.addPoseToCartesianTrajectoryClient(self.targetPose,MaxTranslationVelocity,pause)

            # Send extruder commands to arduino
            self.arduinoPub.publish("G1 E"+fields[7])

    #This callback function is used to control Jaco and Extruder when testing using a text file.
    #Send the target position to add_pose_to_Cartesian_trajectory. The fields of text file are: (0,X,Y,Z,Rx,Ry,Rz,Extrusion,Cooling,Pause,Speed,TypeOfCurve)
    def commandJacoTextFile(self):
        file = open(self.packagePath+'/scripts/pointlog(1).txt')
        for line in file:
            line = line.strip()
            fields = line.split(',')
            if len(fields) == 12:
                print fields
                self.targetPose.X = self.newOrigin[0]+float(fields[1])/1000
                self.targetPose.Y = self.newOrigin[1]+float(fields[2])/1000
                self.targetPose.Z = self.newOrigin[2]+float(fields[3])/1000
                self.targetPose.ThetaX = self.newOrigin[3]+float(fields[4])*3.14/180
                self.targetPose.ThetaY = self.newOrigin[4]+float(fields[5])*3.14/180
                self.targetPose.ThetaZ = self.newOrigin[5]+float(fields[6])*3.14/180
                pause = fields[9]
                MaxTranslationVelocity = 0.05

                #Add targetPose to robot trajectory using the service
                #print 'Requesting add_pose_to_Cartesian_trajectory service'
                self.addPoseToCartesianTrajectoryClient(self.targetPose,MaxTranslationVelocity,pause)

                # Send extruder commands to arduino
                self.arduinoPub.publish("G1 E"+fields[7])

    #This callback function is used to get responses from Arduino
    def getArduinoResponse(self,message):
        #Update temperature of extruder if previous message was "CurrentTemperature"
        if self.extruderTemperatureFlag:
            self.extruderTemperature = float(message.data)
            print self.extruderTemperature
            self.extruderTemperatureFlag = False
        if message.data == "CurrentTemperature":
            self.extruderTemperatureFlag = True

################################################################################
    #Client function for addPoseToCartesianTrajectory service
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

    #Heat Extruder to desired temperature
    def heatExtruder(self):
        r = rospy.Rate(10)
        # Publish desired temperature message for 1second
        for i in range(0,10):
            self.arduinoPub.publish("M104 S"+str(self.extruderDesiredTemperature))
            print "Requested to set desired temperature"
            r.sleep()

        # Wait till extruder temperature reaches desired temperature
        r = rospy.Rate(1)
        while self.extruderTemperature < 0.95*self.extruderDesiredTemperature and not rospy.is_shutdown():
            self.arduinoPub.publish("M114")
            r.sleep()


if __name__ =='__main__':
    #Initialize node
    rospy.init_node('mainController')
    print 'Waiting for the server...'
    rospy.wait_for_service('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory')
    print 'Server Connected'

    try:
        brain = mainController()

    except rospy.ROSInterruptException:
        pass
