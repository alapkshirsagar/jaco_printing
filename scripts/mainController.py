#!/usr/bin/env python

import rospy
import rospkg
import actionlib
import numpy as np
import tf
from tf import transformations as homogeneous
import math
from std_msgs.msg import String
from kinova_msgs.srv import *
from kinova_msgs.msg import *

## Class to communicate between all components: Rhino Plugin, Jaco2, Arduino
class mainController():
    def __init__(self):
        """ Global variable """
        self.homePosition = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home position of jaco2 in unit md
        #self.newOrigin = [0,-0.51+0.085,0.756-0.093,0,0,0] # new origin corresponds to table height and extruder pointing downward
        self.newOrigin = [-0.032+0.01,-0.55+0.01,0.452+0.007,0,0,0]
        self.offset = geometry_msgs.msg.Vector3(0,-0.102,-0.105) # Geometrical distance between gripper and extruder
        self.jacoToRhino = [self.newOrigin[0],self.newOrigin[1],self.newOrigin[2]]

        self.armStopped = False #To track whether arm was stopped in emergency mode

        # Target pose for commanding the robot
        self.targetPose = KinovaPose()
        self.targetPose.X = self.newOrigin[0]
        self.targetPose.Y = self.newOrigin[1]
        self.targetPose.Z = self.newOrigin[2]
        self.targetPose.ThetaX = self.newOrigin[3]
        self.targetPose.ThetaY = self.newOrigin[4]
        self.targetPose.ThetaZ = self.newOrigin[5]

        self.MaxTranslationVelocity = 0.2
        self.MaxRotationalVelocity = 0.2

        self.receivedCounter = 0 #Keep count of messages received from RhinoPlugin
        self.sentCounter = 0 #Keep count of responses sent to RhinoPlugin
        self.extruderTemperatureFlag = False #True if "CurrentTemperature" message is received
        self.extruderTemperature = 0 #Temperature of extruder
        self.extruderDesiredTemperature = 270 #Desired temperature of extruder

        self.rospack = rospkg.RosPack()
        self.packagePath = self.rospack.get_path('jaco_printing') #Get directory path to jaco_printing package

        self.JtoR = homogeneous.identity_matrix() #homogeneous transformation from Jaco base frame to Rhino base frame
        self.EtoG = homogeneous.identity_matrix() #homogeneous transformation from Extruder to Gripper



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

        #Topic for getting joint torques
        #rospy.Subscriber('/j2s7s300_driver/out/joint_torques', JointAngles,self.monitorJointTorques)

########### Services and Actions ##############################################
        #Service for sending trajectory points to the robot
        service_address = '/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory'
        self.addPoseToCartesianTrajectory = rospy.ServiceProxy(service_address, AddPoseToCartesianTrajectory)
        print 'Waiting for addPoseToCartesianTrajectory service server...'
        rospy.wait_for_service(service_address)
        print 'addPoseToCartesianTrajectory service server Connected'


        #Action Client for cartesian position control
        action_address = '/j2s7s300_driver/pose_action/tool_pose'
        self.poseActionClient = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
        print 'Waiting for ArmPoseAction server...'
        self.poseActionClient.wait_for_server()
        print 'ArmPoseAction Server Connected'

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

        #Service for setting end-effector offset
        offset_service = '/j2s7s300_driver/in/set_end_effector_offset'
        self.offsetService = rospy.ServiceProxy(offset_service, SetEndEffectorOffset)
        print 'Waiting for Start service'
        rospy.wait_for_service(offset_service)
        print 'Start service server connected'



###############################################################################
        ## Startup Procedure : Heat Extruder
        if rospy.get_param('~heatExtruder'):
            self.heatExtruder()

        ## Calculate transformation Matrices
        self.RhinoToJacoTransformationMatrix()

        ## Startup Procedure: Move Robot to New Origin
        self.startArmClient() #Remove Emergency Stop flag

        ## Set offset
        self.offsetClient()
        rospy.sleep(1.0)

        # Move arm up from home position by 10cm
        #self.cartesian_pose_client([self.homePosition[0],self.homePosition[1],self.homePosition[2]+0.1],[self.homePosition[3],self.homePosition[4],self.homePosition[5]],self.MaxTranslationVelocity,self.MaxRotationalVelocity);

        # Move arm up to target position using action
        #self.cartesian_pose_client([self.targetPose.X,self.targetPose.Y,self.targetPose.Z],[self.targetPose.ThetaX,self.targetPose.ThetaY,self.targetPose.ThetaZ],self.MaxTranslationVelocity,self.MaxRotationalVelocity);
        self.cartesian_pose_client([0,0,-0.05],self.EulerXYZ2Quaternion([0,0,0]),self.MaxTranslationVelocity,self.MaxRotationalVelocity)
        #self.cartesian_pose_client([0,0,0],self.EulerXYZ2Quaternion([0,-np.pi/4,0]),self.MaxTranslationVelocity,self.MaxRotationalVelocity)
        #self.cartesian_pose_client([0,0,0],self.EulerXYZ2Quaternion([np.pi/4,0,0]),self.MaxTranslationVelocity,self.MaxRotationalVelocity)

        # Move arm up to target position using service
        #for i in range(0,5):
        self.addPoseToCartesianTrajectoryClient([0,0,-0.05],self.EulerXYZ2Quaternion([0,0,0]),self.MaxTranslationVelocity,self.MaxRotationalVelocity,2)
        rospy.sleep(2.0)


        if rospy.get_param('~moveArm'):
            # Continue with rest of the procedure
            if rospy.get_param('~rhinoPlugin'):
                #//TODO Call rhino plugin connection
                print "Rhino Plugin needs to be connected"
            else:
                print "Rhino plugin not needed"
                self.commandJacoTextFile()
            rospy.spin()

####################Callback functions#########################################################

    #This callback function monitors the Joint Torques and calls the emergencyStop service if the Joint Torques exceed certain value
    def monitorJointTorques(self,torques):
        if abs(torques.joint1) > 1:
            self.emergencyStopClient()
        elif self.armStopped:
            self.startArmClient()

    #This callback function is used to control Jaco and Extruder when Rhino Plugin in connected.
    #Send the target position to add_pose_to_Cartesian_trajectory. The fields of 'message' are: (0,X,Y,Z,Rx,Ry,Rz,Rw,Extrusion,Cooling,Pause,Speed,TypeOfCurve)
    def getRhinoCommands(self, message):
        self.receivedCounter = self.receivedCounter+1
        print 'Received Counter:%i'%self.receivedCounter
        fields = message.data.split(',')
        if len(fields) == 13:
            # print fields
            # self.targetPose.X = self.newOrigin[0]+float(fields[2])/1000
            # self.targetPose.Y = self.newOrigin[1]+float(fields[1])/1000
            # self.targetPose.Z = self.newOrigin[2]+float(fields[3])/1000
            # self.targetPose.ThetaX = self.newOrigin[3]+float(fields[4])*3.14/180
            # self.targetPose.ThetaY = self.newOrigin[4]+float(fields[5])*3.14/180
            # self.targetPose.ThetaZ = self.newOrigin[5]+float(fields[6])*3.14/180
            position = [-1*float(fields[1])/1000, -1*float(fields[2])/1000, -1*float(fields[3])/1000]
            orientation = [float(fields[4]),float(fields[5]),float(fields[6]),float(fields[7])]
            print orientation
            pause = int (fields[10])
            MaxTranslationVelocity = float(fields[11])/1000
            MaxRotationalVelocity = float(fields[11])/1000

            # Send cooling commands to arduino
            if int(fields[9]) == 1:
                self.arduinoPub.publish("M106 S1")
            else:
                self.arduinoPub.publish("M106 S0")




            if pause == 0:
                #Add targetPose to robot trajectory using the service
                #print 'Requesting add_pose_to_Cartesian_trajectory service'
                self.addPoseToCartesianTrajectoryClient(position,orientation,MaxTranslationVelocity,MaxRotationalVelocity,2)
                self.cartesian_pose_client(position,orientation,MaxTranslationVelocity,MaxRotationalVelocity)

            else:
                self.addPoseToCartesianTrajectoryClient(position,orientation,MaxTranslationVelocity,MaxRotationalVelocity,2)

                #Use action client to move the robot and pause
                self.cartesian_pose_client(position,orientation,MaxTranslationVelocity,MaxRotationalVelocity)
                pause = pause + 0

            # Send extruder commands to arduino
            self.arduinoPub.publish("G1 E"+fields[8])

            #Pause robot
            print "Pausing for"
            print rospy.Duration(pause,0)
            if not rospy.is_shutdown():
                rospy.sleep(rospy.Duration(pause,0))

            #Rend reponse to Rhino
            self.rhinoPub.publish('Motion.\r')
            self.sentCounter = self.sentCounter+1
            print 'Sent Counter:%i'%self.sentCounter

    #This callback function is used to get responses from Arduino
    def getArduinoResponse(self,message):
        #Update temperature of extruder if previous message was "CurrentTemperature"
        if self.extruderTemperatureFlag:
            self.extruderTemperature = float(message.data)
            print self.extruderTemperature
            self.extruderTemperatureFlag = False
        if message.data == "CurrentTemperature":
            self.extruderTemperatureFlag = True

################################## Service and Action Clients #######################################
    #Client function for addPoseToCartesianTrajectory service. Orientation in
    def addPoseToCartesianTrajectoryClient(self,position,orientation,MaxTranslationVelocity,MaxRotationalVelocity,pause):
        try:
            JtoG = self.RhinoToJacoTransformation(position,orientation)
            position =  homogeneous.translation_from_matrix(JtoG)
            orientation = homogeneous.quaternion_from_matrix(JtoG)
            orientation = self.Quaternion2EulerXYZ(orientation)

            #print "Requested pose = "
            print position
            print orientation

            resp1 = self.addPoseToCartesianTrajectory(position[0],position[1],position[2],orientation[0],orientation[1],orientation[2],MaxTranslationVelocity,MaxRotationalVelocity) #self.addPoseToCartesianTrajectory(targetPose.X,targetPose.Y,targetPose.Z,targetPose.ThetaX,targetPose.ThetaY,targetPose.ThetaZ,MaxTranslationVelocity,MaxRotationalVelocity)

            #print resp1

            self.rhinoPub.publish('Motion.\r')
            self.sentCounter = self.sentCounter+1
            print 'Sent Counter:%i'%self.sentCounter
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    ## Cartesian Pose action client
    def cartesian_pose_client(self,position, orientation,MaxTranslationVelocity,MaxRotationalVelocity):
        JtoG = self.RhinoToJacoTransformation(position,orientation)
        position =  homogeneous.translation_from_matrix(JtoG)
        orientation = homogeneous.quaternion_from_matrix(JtoG)

        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=('j2s7s300_link_base'))
        goal.MaxTranslationVelocity = MaxTranslationVelocity;
        goal.MaxRotationalVelocity = MaxRotationalVelocity;
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        self.poseActionClient.send_goal(goal)
        self.poseActionClient.wait_for_result()

    ## Emergency Stop Action Client
    def emergencyStopClient(self):
        try:
            response = self.emergencyStop()
            print response
            self.armStopped = True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    ## Emergency Stop Action Client
    def startArmClient(self):
        try:
            response = self.startArm()
            print response
            self.armStopped = False
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    ## End effector offset Service Client
    def offsetClient(self):
        try:
            status = 1
            response = self.offsetService(status,self.offset)
            print response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e



#################################### Methods ###############################################
    #This function is used to control Jaco and Extruder when testing using a text file.
    #Send the target position to add_pose_to_Cartesian_trajectory. The fields of text file are: (0,X,Y,Z,Rx,Ry,Rz,Rw,Extrusion,Cooling,Pause,Speed,TypeOfCurve)
    def commandJacoTextFile(self):
        file = open(self.packagePath+'/scripts/multi-points.txt')
        line = file.readline()
        #print line
        while line and not rospy.is_shutdown():
            line = line.strip()
            fields = line.split(',')
            if len(fields) == 13:
                #print fields
                position = [-1*float(fields[1])/1000, -1*float(fields[2])/1000, -1*float(fields[3])/1000]
                #print position
                #self.targetPose.Y = self.newOrigin[1]+float(fields[2])/1000
                #self.targetPose.Z = self.newOrigin[2]+float(fields[3])/1000
                #self.targetPose.ThetaX = self.newOrigin[3]+float(fields[4])*3.14/180
                #self.targetPose.ThetaY = self.newOrigin[4]+float(fields[5])*3.14/180
                #self.targetPose.ThetaZ = self.newOrigin[5]+float(fields[6])*3.14/180
                orientation = [float(fields[4]),float(fields[5]),float(fields[6]),float(fields[7])]
                #print orientation
                pause = int (fields[10])
                MaxTranslationVelocity = float(fields[11])/1000
                MaxRotationalVelocity = float(fields[11])/1000

                # Send cooling commands to arduino
                if int(fields[9]) == 1:
                    self.arduinoPub.publish("M106 S1")
                else:
                    self.arduinoPub.publish("M106 S0")




                if pause == 0:
                    #Add targetPose to robot trajectory using the service
                    #print 'Requesting add_pose_to_Cartesian_trajectory service'
                    self.addPoseToCartesianTrajectoryClient(position,orientation,MaxTranslationVelocity,MaxRotationalVelocity,2)
                    self.cartesian_pose_client(position,orientation,MaxTranslationVelocity,MaxRotationalVelocity)

                else:
                    self.addPoseToCartesianTrajectoryClient(position,orientation,MaxTranslationVelocity,MaxRotationalVelocity,2)

                    #Use action client to move the robot and pause
                    self.cartesian_pose_client(position,orientation,MaxTranslationVelocity,MaxRotationalVelocity)
                    pause = pause + 0

                # Send extruder commands to arduino
                self.arduinoPub.publish("G1 E"+fields[8])

                #Pause robot
                print "Pausing for"
                print rospy.Duration(pause,0)
                if not rospy.is_shutdown():
                    rospy.sleep(rospy.Duration(pause,0))
            line = file.readline()
        file.close()


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

    def QuaternionNorm(self,Q_raw):
        qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
        qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
        qx_ = qx_temp/qnorm
        qy_ = qy_temp/qnorm
        qz_ = qz_temp/qnorm
        qw_ = qw_temp/qnorm
        Q_normed_ = [qx_, qy_, qz_, qw_]
        return Q_normed_


    def Quaternion2EulerXYZ(self,Q_raw):
        Q_normed = self.QuaternionNorm(Q_raw)
        qx_ = Q_normed[0]
        qy_ = Q_normed[1]
        qz_ = Q_normed[2]
        qw_ = Q_normed[3]

        tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
        ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
        tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
        EulerXYZ_ = [tx_,ty_,tz_]
        return EulerXYZ_



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


    # Get position and quaternions from RhinoPlugin and convert them to position and Quaternions for Robot End Effector
    def RhinoToJacoTransformation(self,positionRhino,quaternionRhino):
        #homogeneous transformation from Rhino to Extruder
        RtoE = homogeneous.concatenate_matrices(homogeneous.translation_matrix(positionRhino),homogeneous.quaternion_matrix(quaternionRhino))
        #print 'RtoE='
        #print RtoE

        #Premultiply with Jaco to Rhino and Postmultiply with Extruder to Gripper
        JtoG = homogeneous.concatenate_matrices(self.JtoR, RtoE, self.EtoG)
        #print 'JtoG='
        #print JtoG
        #Convert homogeneous matrix to Quaternions and Positions
        return JtoG

    # Calculate transformation matrices for Jaco to Rhino and End-Effector to Gripper frames
    def RhinoToJacoTransformationMatrix(self):
        self.JtoR = homogeneous.concatenate_matrices(homogeneous.translation_matrix(self.jacoToRhino), homogeneous.euler_matrix(np.pi,0,np.pi/2,'rxyz'))
        print 'JtoR='
        print self.JtoR
        self.EtoG = homogeneous.euler_matrix(0,np.pi/2,-np.pi/2,'rxyz')
        print 'EtoG='
        print self.EtoG




if __name__ =='__main__':
    #Initialize node
    rospy.init_node('mainController')

    try:
        brain = mainController()

    except rospy.ROSInterruptException:
        pass
