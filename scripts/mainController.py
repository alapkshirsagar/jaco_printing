#!/usr/bin/env python

import rospy
import rospkg
import actionlib
import numpy as np
import tf
import math
from std_msgs.msg import String
from kinova_msgs.srv import *
from kinova_msgs.msg import *

## Class to communicate between all components: Rhino Plugin, Jaco2, Arduino
class mainController():
    def __init__(self):
        """ Global variable """
        self.homePosition = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home position of jaco2 in unit md
        self.newOrigin = [0.015-0.001,-0.51,0.496+0.1,np.pi/2,0,0] # new origin corresponds to table height and extruder pointing downward


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

########### Services and Actions ##############################################
        #Service for sending trajectory points to the robot
        service_address = '/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory'
        self.addPoseToCartesianTrajectory = rospy.ServiceProxy(service_address, AddPoseToCartesianTrajectory)
        print 'Waiting for the service server...'
        rospy.wait_for_service('/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory')
        print 'Service Server Connected'


        """Send a cartesian goal to the action server."""
        action_address = '/j2s7s300_driver/pose_action/tool_pose'
        self.poseActionClient = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
        print 'Waiting for action server...'
        self.poseActionClient.wait_for_server()
        print 'Service Server Connected'


###############################################################################
        # Startup Procedure : Heat Extruder
        if rospy.get_param('~heatExtruder'):
            self.heatExtruder()

        #self.cartesian_pose_client([self.targetPose.X,self.targetPose.Y,self.targetPose.Z],[self.targetPose.ThetaX,self.targetPose.ThetaY,self.targetPose.ThetaZ]);

        # Startup Procedure: Move Robot to New Origin
        if rospy.get_param('~moveArm'):
            self.addPoseToCartesianTrajectoryClient(self.targetPose,0.05,2)

            # Continue with rest of the procedure
            if rospy.get_param('~rhinoPlugin'):
                #//TODO Call rhino plugin connection
                print "Rhino Plugin needs to be connected"
            else:
                print "Rhino plugin not needed"
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
        file = open(self.packagePath+'/scripts/pointlog.txt')
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
                pause = int (fields[9])
                MaxTranslationVelocity = 0.05

                if pause == 0:
                    #Add targetPose to robot trajectory using the service
                    #print 'Requesting add_pose_to_Cartesian_trajectory service'
                    self.addPoseToCartesianTrajectoryClient(self.targetPose,MaxTranslationVelocity,pause)
                else:
                    #Use action client to move the robot and pause
                    self.cartesian_pose_client([self.targetPose.X,self.targetPose.Y,self.targetPose.Z],[self.targetPose.ThetaX,self.targetPose.ThetaY,self.targetPose.ThetaZ])

                    #Pause robot
                    print "Pausing for"
                    print rospy.Duration(pause,0)
                    rospy.sleep(rospy.Duration(pause,0))

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
            print targetPose
            resp1 = self.addPoseToCartesianTrajectory(targetPose.X,targetPose.Y,targetPose.Z,targetPose.ThetaX,targetPose.ThetaY,targetPose.ThetaZ,MaxTranslationVelocity)
            #print resp1

            self.rhinoPub.publish('Motion.\r')
            self.sentCounter = self.sentCounter+1
            print 'Sent Counter:%i'%self.sentCounter
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Cartesian Pose action client
    def cartesian_pose_client(self,position, orientation):
        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=('j2s7s300_link_base'))
        orientation_q = self.EulerXYZ2Quaternion(orientation)
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation_q[0], y=orientation_q[1], z=orientation_q[2], w=orientation_q[3])

        #print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        self.poseActionClient.send_goal(goal)
        if self.poseActionClient.wait_for_result(rospy.Duration(10.0)):
            print 'Reached Position'
            return True
        else:
            self.poseActionClient.cancel_all_goals()
            print('        the cartesian action timed-out')
            return None



    # Convert Euler angles to Quaternions
    def EulerXYZ2Quaternion(self,EulerXYZ_):
        tx_, ty_, tz_ = EulerXYZ_[0:3]
        sx = math.sin(0.5 * tx_)
        cx = math.cos(0.5 * tx_)
        sy = math.sin(0.5 * ty_)
        cy = math.cos(0.5 * ty_)
        sz = math.sin(0.5 * tz_)
        cz = math.cos(0.5 * tz_)

        qx_ = sx * cy * cz + cx * sy * sz
        qy_ = -sx * cy * sz + cx * sy * cz
        qz_ = sx * sy * cz + cx * cy * sz
        qw_ = -sx * sy * sz + cx * cy * cz

        Q_ = [qx_, qy_, qz_, qw_]
        return Q_



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

    try:
        brain = mainController()

    except rospy.ROSInterruptException:
        pass
