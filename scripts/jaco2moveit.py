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




## Class to control jaco using MoveIt. Also communicate with Rhino and Arduino
class jaco2moveit():

    jointData = []
    jointState = []
    def jointcallback(msg,joints):
        global jointData
        jointData = [joints.joint1,joints.joint2,joints.joint3,joints.joint4,joints.joint5,joints.joint6,joints.joint7]

    def jointstatecallback(msg,joints):
        global jointState
        jointState = joints.position

    def __init__(self):
            """Variables"""
            ##Home Position for Jaco
            self.homePosition = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home position of jaco2 in unit md
            self.newOrigin = [-0.070+0.012,-0.5+0.017,0.464,0,0,0] #In Jaco base frame. JtoR
            self.jacoToRhino = [self.newOrigin[0],self.newOrigin[1],self.newOrigin[2]]

            self.offsetVertical = [-0.106,0,0.100] # Geometrical distance between gripper and extruder. In Extruder frame. Use this when the extruder is pointing downward (0,0,0,1)
            self.offsetHorizontal = [-0.116,0,0.108] # Geometrical distance between gripper and extruder. In Extruder frame. Use this when the extruder is horizontal (0.707,0,0,707,0)


            ##Target pose for commanding the robot
            self.targetPose = geometry_msgs.msg.Pose()
            self.targetPose.position.x = 0.212322831154
            self.targetPose.position.y = -0.257197618484
            self.targetPose.position.z = 0.509646713734
            Orientation = self.EulerXYZ2Quaternion([1.63771402836,1.11316478252, 0.134094119072])
            self.targetPose.orientation.x = Orientation[0]
            self.targetPose.orientation.y = Orientation[1]
            self.targetPose.orientation.z = Orientation[2]
            self.targetPose.orientation.w = Orientation[3]

            ## Maximum velocity. This number is used to get velocity scaling factor. The robot will not move by this exact velocity
            self.MaxTranslationVelocity = 200

            ## Trajectory for commanding the robot
            self.waypoints = []

            self.receivedCounter = 0 #Keep count of messages received from RhinoPlugin
            self.sentCounter = 0 #Keep count of responses sent to RhinoPlugin
            self.extruderTemperatureFlag = False #True if "CurrentTemperature" message is received
            self.touchPlatformFlag = False #True if "TouchPlatform" message is received
            self.extruderTemperature = 0 #Temperature of extruder
            self.extruderDesiredTemperature = 270 #Desired temperature of extruder
            self.autoAngleValue = "zero" #Platform rotation angle when robot takes control of platform

            self.rospack = rospkg.RosPack()
            self.packagePath = self.rospack.get_path('jaco_printing') #Get directory path to jaco_printing package

            self.armStopped = False #To track whether arm was stopped in emergency mode


######################### Subscribers ######################################################
            self.sub_jointAngle = rospy.Subscriber('/j2s7s300_driver/out/joint_angles', JointAngles, self.jointcallback)
            self.sub_jointState = rospy.Subscriber('/j2s7s300_driver/out/joint_state', JointState, self.jointstatecallback)
            #Topic for getting response from Arduino
            rospy.Subscriber('/chatter',String, self.getArduinoResponse)

            #Topic for getting commands from RhinoPlugin
            rospy.Subscriber('/command',String, self.getRhinoCommands)

            #Topic for getting joint torques
            #rospy.Subscriber('/j2s7s300_driver/out/joint_torques', JointAngles,self.monitorJointTorques)
            #Topic for getting cartesian force on end effector
            rospy.Subscriber('/j2s7s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, self.monitorToolWrench)




######################### Publishers #######################################################
            #Topic for sending response to RhinoPlugin
            self.rhinoPub = rospy.Publisher('/command_response', String, queue_size=10)

            #Topic for sending commands to Arduino
            self.arduinoPub = rospy.Publisher('/extruder', String, queue_size=10)


######################### Actions ##################################################
            #Action Client for joint control
            action_address = '/j2s7s300_driver/joints_action/joint_angles'
            self.jointActionClient = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmJointAnglesAction)
            print 'Waiting for ArmJointAnglesAction server...'
            self.jointActionClient.wait_for_server()
            print 'ArmJointAnglesAction Server Connected'

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
            ## First initialize moveit_commander.
            print "============ Starting jaco2moveit setup"
            moveit_commander.roscpp_initialize(sys.argv)
            ## Configure MoveIt planning interface for robot
            ## Instantiate a RobotCommander object.  This object is an interface to
            #  the robot as a whole.
            self.robot = moveit_commander.RobotCommander()

            ## Instantiate a PlanningSceneInterface object.  This object is an interface
            ## to the world surrounding the robot.
            self.scene = moveit_commander.PlanningSceneInterface()

            rospy.sleep(2.0)

            ## Add a barrier to prevent the robot from colliding with human
            p = geometry_msgs.msg.PoseStamped()
            p.header.frame_id = self.robot.get_planning_frame()
            p.pose.position.x = self.newOrigin[0]
            p.pose.position.y = self.newOrigin[1]-0.5
            p.pose.position.z = self.newOrigin[2]
            self.scene.add_box("barrier", p, (1, 0.1, 1))

            ## Instantiate a MoveGroupCommander object.  This object is an interface
            ## to one group of joints.  In this case the group is the joints in the left
            ## arm.  This interface can be used to plan and execute motions on the left
            ## arm.
            self.group = moveit_commander.MoveGroupCommander("arm")

            ## We create this DisplayTrajectory publisher which is used below to publish
            ## trajectories for RVIZ to visualize.
            self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 10)

            ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
            print "============ Waiting for RVIZ..."
            rospy.sleep(1)
            print "============ Starting jaco2moveit "

            ## Getting Basic Information
            ## ^^^^^^^^^^^^^^^^^^^^^^^^^
            ##
            ## We can get the name of the reference frame for this robot
            print "============ Reference frame: %s" % self.group.get_planning_frame()

            ## We can also print the name of the end-effector link for this group
            print "============ Reference frame: %s" % self.group.get_end_effector_link()

            ## We can get a list of all the groups in the robot
            print "============ Robot Groups:"
            print self.robot.get_group_names()

            ## Sometimes for debugging it is useful to print the entire state of the
            ## robot.
            #print "============ Printing robot state"
            #print self.robot.get_current_state()
            print "============"
###############################################################################
            ## Startup Procedure : Heat Extruder
            if rospy.get_param('~heatExtruder'):
                self.heatExtruder()

            ## Set rotation angle to zero
            self.arduinoPub.publish("M15")

            ## Calculate transformation Matrices
            self.RhinoToJacoTransformationMatrix()

            ## Start Arm
            self.startArmClient() #Remove Emergency Stop flag

            ## Home arm
            #self.homeArmClient()

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
        #print toolwrench
        if toolwrench > 100:
            self.emergencyStopClient()  # Stop arm driver


    # This callback function is used to get responses from Arduino
    def getArduinoResponse(self, message):
        # Update temperature of extruder if previous message was
        # "CurrentTemperature"
        print message

        if "CurrentTemperature" in message.data:
            self.extruderTemperature = float(message.data[18:])
            print self.extruderTemperature
            self.extruderTemperatureFlag = False
        if message.data == "TouchPlatform  1.00":
            self.touchPlatformFlag = True
            # If Human has touched the platform get the robot back to home
            # position
            print "user touched platform"
            self.rhinoPub.publish('Touched')
            wpose = geometry_msgs.msg.Pose()
            self.waypoints = [] # Clean waypoints list
            position = [0.2,0,0.2]
            orientation = [0,0,0,1]
            ##Convert poses to Robot's frame
            JtoG = self.RhinoToJacoTransformation(position, orientation)
            position = homogeneous.translation_from_matrix(JtoG)
            orientation = homogeneous.quaternion_from_matrix(JtoG)
            wpose.position.x = position[0]
            wpose.position.y = position[1]
            wpose.position.z = position[2]
            wpose.orientation.x = orientation[0]
            wpose.orientation.y = orientation[1]
            wpose.orientation.z = orientation[2]
            wpose.orientation.w = orientation[3]
            self.waypoints.append(copy.deepcopy(wpose))

            plan = self.planTrajectory(0.5,-1, 0)  # Use pose move interface
            self.moveTrajectory(plan,-1)
            self.arduinoPub.publish("M18") # Disable platform motor
            self.rhinoPub.publish('RotationEnabled')
        elif message.data == "TouchPlatform  0.00":
            self.touchPlatformFlag = False
            wpose = geometry_msgs.msg.Pose()
            self.waypoints = [] # Clean waypoints list
            position = [0,0,0.15]
            orientation = [0,0,0,1]
            ##Convert poses to Robot's frame
            JtoG = self.RhinoToJacoTransformation(position, orientation)
            position = homogeneous.translation_from_matrix(JtoG)
            orientation = homogeneous.quaternion_from_matrix(JtoG)
            wpose.position.x = position[0]
            wpose.position.y = position[1]
            wpose.position.z = position[2]
            wpose.orientation.x = orientation[0]
            wpose.orientation.y = orientation[1]
            wpose.orientation.z = orientation[2]
            wpose.orientation.w = orientation[3]
            self.waypoints.append(copy.deepcopy(wpose))

            plan = self.planTrajectory(0.5,-1, 0)  # Use pose move interface
            self.moveTrajectory(plan,-1)


        # Send angle of platform to RhinoPlugin
        fields = message.data.split(':')
        if fields[0] == 'AutoAngle':
            self.rhinoPub.publish('Rot: ' + fields[1])

        # Send autoAngle to RhinoPlugin only when the previous value is same as current value
        fields = message.data.split(':')
        if fields[0] == 'Angle':
            if self.autoAngleValue != fields[1]:
                self.autoAngleValue = fields[1]
                rospy.sleep(0.1)
                self.arduinoPub.publish("M16")
            else:
                self.rhinoPub.publish('Rot: ' + fields[1])




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
            wpose = geometry_msgs.msg.Pose()
            wpose.position.x = 300
            wpose.position.y = 0
            wpose.position.z = -50
            wpose.orientation.x = 0
            wpose.orientation.y = 0
            wpose.orientation.z = 0
            wpose.orientation.w = 1
            self.moveToPose(wpose)  # Use pose move interface
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
    # Heat Extruder to desired temperature
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

    def jointActionClientMethod(self,angle_set):
        goal = kinova_msgs.msg.ArmJointAnglesGoal()

        goal.angles.joint1 = angle_set[0]
        goal.angles.joint2 = angle_set[1]
        goal.angles.joint3 = angle_set[2]
        goal.angles.joint4 = angle_set[3]
        goal.angles.joint5 = angle_set[4]
        goal.angles.joint6 = angle_set[5]
        goal.angles.joint7 = angle_set[6]

        self.jointActionClient.send_goal(goal)
        if self.jointActionClient.wait_for_result(rospy.Duration(20.0)):
            return self.jointActionClient.get_result()
        else:
            print('        the joint angle action timed-out')
            self.jointActionClient.cancel_all_goals()
            return None


    def moveToPose(self, targetPose):
        ## Set a scaling factor for reducing the maximum joint velocity. Allowed values are in (0,1].
        #self.group.set_max_velocity_scaling_factor(1.0)

        ## Planning to a Pose goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector
        print "============ Generating plan 1"
        self.group.set_pose_target(targetPose)

        ## Now, we call the planner to compute the plan
        ## and visualize it if successful
        ## Note that we are just planning, not asking move_group
        ## to actually move the robot
        plan1 = self.group.plan()

        # print "================================ Moveit's plan ==================="
        # print plan1
        # print "\n"
        #
        #
        # print "============ Waiting while RVIZ displays plan1..."
        # #rospy.sleep(5)

        ## Moving to a pose goal
        ## ^^^^^^^^^^^^^^^^^^^^^
        ptPos = plan1.joint_trajectory.points[-1].positions
        print "=================================="
        print "Last point of the current trajectory: "
        angle_set = list()
        for i in range (len(ptPos)):
            tempPos = ptPos[i]*180/3.14
            if tempPos < 0:
                tempPos += 360
            angle_set.append(tempPos)
            print "data" + str(i+1) + ": " + str(tempPos)
        print "\n"
        self.group.go(wait=True)
        #self.jointActionClientMethod(angle_set)
        print "current joint angle data from encoder:"
        # # in degree
        print jointData
        print "\n"


    def planTrajectory(self,velocity_scaling_factor, typeOfCurve, extruderLength):
    ## We want the cartesian path to be interpolated at a resolution of 1 mm
    ## which is why we will specify 0.001 as the eef_step in cartesian
    ## translation.  We will specify the jump threshold as 0.0, effectively
    ## disabling it.

        #self.group.set_max_velocity_scaling_factor(0.1)

        (plan3, fraction) = self.group.compute_cartesian_path(
                           self.waypoints,   # waypoints to follow
                           0.001,        # eef_step
                           0.0) # jump_threshold
        #Retime the trajectory to apply velocity_scaling_factor
        plan4 = self.group.retime_trajectory(self.robot.get_current_state(),plan3,velocity_scaling_factor)
        time = plan4.joint_trajectory.points[-1].time_from_start.secs + plan4.joint_trajectory.points[-1].time_from_start.nsecs/1000000000.0
        velocity = extruderLength/time

        while velocity > 5:
                velocity_scaling_factor = velocity_scaling_factor*0.9
                (plan3, fraction) = self.group.compute_cartesian_path(
                                   self.waypoints,   # waypoints to follow
                                   0.001,        # eef_step
                                   0.0) # jump_threshold
                #Retime the trajectory to apply velocity_scaling_factor
                plan4 = self.group.retime_trajectory(self.robot.get_current_state(),plan3,velocity_scaling_factor)
                time = plan4.joint_trajectory.points[-1].time_from_start.secs + plan4.joint_trajectory.points[-1].time_from_start.nsecs/1000000000.0
                velocity = extruderLength/time

        print "Velocity of printing"
        print velocity
        #print plan4
        return plan4

    def moveTrajectory(self, plan, typeOfCurve):
    ## Move the arm along the trajectory given by plan

        self.group.execute(plan, wait=True)


        #print plan3
        rospy.sleep(0.1)
        #print "================================ Moveit's plan ==================="
        #print plan3
        #print "\n"

        ptPos = plan.joint_trajectory.points[-1].positions
        #print "=================================="
        #print "Last point of the current trajectory: "
        angle_set = list()

        for i in range (len(ptPos)):
            tempPos = ptPos[i]*180/np.pi + int(round((jointData[i] - ptPos[i]*180/np.pi)/(360)))*360
            angle_set.append(tempPos)
            #if tempPos < 0:
                #tempPos += 360
            #print "data" + str(i+1) + ": " + str(tempPos)
        #print "\n"

        #print "current joint angle data from encoder:"
        # in degree
        #print jointData
        #print "\n"
        #if typeOfCurve in [18,19,20]:
        #    print "Bottom Line"
        #else:
        if typeOfCurve is not -1:
            self.jointActionClientMethod(angle_set)
        #rospy.sleep(1)

        # in radians
        #for i in range (len(jointState)-3):
        #    print "joint: " + str(i+1) + ": " + str(jointState[i])
        #print "\n"

        self.waypoints = []


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


    # Calculate transformation matrices for Jaco to Rhino and End-Effector to Gripper frames
    def RhinoToJacoTransformationMatrix(self):
        self.JtoR = homogeneous.concatenate_matrices(homogeneous.translation_matrix(self.jacoToRhino), homogeneous.euler_matrix(0,0,np.pi/2,'rxyz'))
        print 'JtoR='
        print self.JtoR
        self.EtoG_V = homogeneous.concatenate_matrices(homogeneous.translation_matrix(self.offsetVertical), homogeneous.euler_matrix(0,-np.pi/2,0,'rxyz'))
        self.EtoG_H = homogeneous.concatenate_matrices(homogeneous.translation_matrix(self.offsetHorizontal), homogeneous.euler_matrix(0,-np.pi/2,0,'rxyz'))
        print 'EtoG_Vertical='
        print self.EtoG_V
        print 'EtoG_Horizontal='
        print self.EtoG_H

    # Get position and quaternions from RhinoPlugin and convert them to position and Quaternions for Robot End Effector
    def RhinoToJacoTransformation(self,positionRhino,quaternionRhino):
        #homogeneous transformation from Rhino to Extruder
        RtoE = homogeneous.concatenate_matrices(homogeneous.translation_matrix(positionRhino),homogeneous.quaternion_matrix(quaternionRhino))
        # print 'RtoE='
        # print RtoE

        # Premultiply with Jaco to Rhino and Postmultiply with Extruder to Gripper.
        if quaternionRhino[0] < 0.5:
            JtoG = homogeneous.concatenate_matrices(self.JtoR, RtoE, self.EtoG_V)
        else:
            JtoG = homogeneous.concatenate_matrices(self.JtoR, RtoE, self.EtoG_H)
        # print 'JtoG='
        # print JtoG
        #Parking Motion.\r Convert homogeneous matrix to Quaternions and Positions
        return JtoG

    # This function is used to control Jaco and Extruder when testing using a text file.
    # Send the target position to moveitTrajectoryPlanner.
    # The fields of text file are: (0,X,Y,Z,Rx,Ry,Rz,Rw,Extrusion,Cooling,Pause,Speed,TypeOfCurve)
    def commandJacoTextFile(self):
        file = open(self.packagePath+'/scripts/sample_points.txt')
        line = file.readline()
        # print line
        while line and not rospy.is_shutdown():
            line = line.strip()
            fields = line.split(',')
            if len(fields) == 13:
                # print fields
                position = [-1*float(fields[1])/1000, -1*float(fields[2])/1000, -1*float(fields[3])/1000]
                orientation = [float(fields[4]), float(fields[5]), float(fields[6]), float(fields[7])]
                pause = int(fields[10])
                typeOfCurve = int(fields[10])
                MaxTranslationVelocity = float(fields[11])
                MaxRotationalVelocity = float(fields[11])
                velocity_scaling_factor = MaxTranslationVelocity/self.MaxTranslationVelocity

                # Convert poses to Robot's frame
                JtoG = self.RhinoToJacoTransformation(position, orientation)
                position = homogeneous.translation_from_matrix(JtoG)
                orientation = homogeneous.quaternion_from_matrix(JtoG)

                # Add pose to waypoints
                wpose = geometry_msgs.msg.Pose()
                wpose.position.x = position[0]
                wpose.position.y = position[1]
                wpose.position.z = position[2]
                wpose.orientation.x = orientation[0]
                wpose.orientation.y = orientation[1]
                wpose.orientation.z = orientation[2]
                wpose.orientation.w = orientation[3]
                self.waypoints.append(copy.deepcopy(wpose))
                # self.moveToPose(wpose) ## Use pose move interface

                # Send cooling commands to arduino
                if int(fields[9]) == 1:
                    self.arduinoPub.publish("M106 S1")
                else:
                    self.arduinoPub.publish("M106 S0")

                # Move arm when pause is not zero
                if pause != 0:
                    plan = self.planTrajectory(velocity_scaling_factor, typeOfCurve, float(fields[8]))
                    self.moveTrajectory(plan, typeOfCurve)

                # Send extruder commands to arduino
                self.arduinoPub.publish("G1 E"+fields[8])

                # Pause robot
                print "Pausing for"
                print rospy.Duration(pause, 0)
                if not rospy.is_shutdown():
                    rospy.sleep(rospy.Duration(pause, 0))
                if rospy.is_shutdown():
                    print 'Goodbye!'
            line = file.readline()
        file.close()

    # This callback function is used to control Jaco and Extruder when Rhino Plugin in connected.
    # Send the target position to add_pose_to_Cartesian_trajectory. The fields of 'message' are: (0,X,Y,Z,Rx,Ry,Rz,Rw,Extrusion,Cooling,TypeOfCurve,Pause,Speed)
    def getRhinoCommands(self, message):
        self.startArmClient()
        if message.data[:4] == "M117":
            # Send platform rotation command to Arduino
            self.arduinoPub.publish(message)
            # Ask rotation angle from arduino
            self.arduinoPub.publish("M16")

        self.receivedCounter = self.receivedCounter+1
        print 'Received Counter:%i' % self.receivedCounter
        fields = message.data.split(',')
        print message.data

        if message.data == "1,":
            # If Human moves away from the platform get the robot back to home
            # position and
            print "user moved away from the platform"
            wpose = geometry_msgs.msg.Pose()
            self.waypoints = []  # Clean waypoints list
            position = [0.3, 0, 0.3]
            orientation = [0, 0, 0, 1]
            # Convert poses to Robot's frame
            JtoG = self.RhinoToJacoTransformation(position, orientation)
            position = homogeneous.translation_from_matrix(JtoG)
            orientation = homogeneous.quaternion_from_matrix(JtoG)
            wpose.position.x = position[0]
            wpose.position.y = position[1]
            wpose.position.z = position[2]
            wpose.orientation.x = orientation[0]
            wpose.orientation.y = orientation[1]
            wpose.orientation.z = orientation[2]
            wpose.orientation.w = orientation[3]
            self.waypoints.append(copy.deepcopy(wpose))

            plan = self.planTrajectory(0.5, -1, 0)  # Use pose move interface
            self.moveTrajectory(plan, -1)

            # Send reponse to Rhino
            self.rhinoPub.publish('Parking Motion.\r')

        if message.data == "reset":
            # If we press the 'Reset' button in Rhino Plugin the robot should
            # go to partking position and rotation angle should be reset to 0

            # Send reset angle command to arduino and Rhino
            self.arduinoPub.publish('M15')
            self.rhinoPub.publish('Rot: ' + str(0))

            # Move robot to parking position
            wpose = geometry_msgs.msg.Pose()
            self.waypoints = []  # Clean waypoints list
            position = [0.3, 0, 0.15]  # Parking position
            orientation = [0, 0, 0, 1]

            # Convert poses to Robot's frame
            JtoG = self.RhinoToJacoTransformation(position, orientation)
            position = homogeneous.translation_from_matrix(JtoG)
            orientation = homogeneous.quaternion_from_matrix(JtoG)
            wpose.position.x = position[0]
            wpose.position.y = position[1]
            wpose.position.z = position[2]
            wpose.orientation.x = orientation[0]
            wpose.orientation.y = orientation[1]
            wpose.orientation.z = orientation[2]
            wpose.orientation.w = orientation[3]
            self.waypoints.append(copy.deepcopy(wpose))

            plan = self.planTrajectory(0.5, -1, 0)  # Use pose move interface
            self.moveTrajectory(plan, -1)


            # Send response to Rhino
            self.rhinoPub.publish('Reset Done.\r')


        if len(fields) == 13:
            # print fields
            position = [float(fields[1])/1000, float(fields[2])/1000, float(fields[3])/1000]
            orientation = [float(fields[4]), float(fields[5]), float(fields[6]), float(fields[7])]
            typeOfCurve = int(fields[10])
            pause = int(fields[11])
            MaxTranslationVelocity = float(fields[12])
            MaxRotationalVelocity = float(fields[12])
            velocity_scaling_factor = MaxTranslationVelocity/self.MaxTranslationVelocity
            print "Velocity factor :"
            print velocity_scaling_factor

            # Convert poses to Robot's frame
            JtoG = self.RhinoToJacoTransformation(position, orientation)
            position = homogeneous.translation_from_matrix(JtoG)
            orientation = homogeneous.quaternion_from_matrix(JtoG)

            # Add pose to waypoints
            wpose = geometry_msgs.msg.Pose()
            wpose.position.x = position[0]
            wpose.position.y = position[1]
            wpose.position.z = position[2]
            wpose.orientation.x = orientation[0]
            wpose.orientation.y = orientation[1]
            wpose.orientation.z = orientation[2]
            wpose.orientation.w = orientation[3]
            self.waypoints.append(copy.deepcopy(wpose))
            # self.moveToPose(wpose) ## Use pose move interface

            # Extrude extra material at the beginning of a segment
            if typeOfCurve in (9, 11, 18):
                self.arduinoPub.publish("G1 E12 T0.5")
                rospy.sleep(0.5)

            # Move arm when pause is not zero
            if pause != 0:
                plan = self.planTrajectory(velocity_scaling_factor, typeOfCurve, float(fields[8]))
                time = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs/1000000000.0
                print time
                # Send extruder commands to arduino
                if fields[8] is not "0.00" and time > 1:
                    self.arduinoPub.publish("G1 E"+fields[8]+" T"+str(time))
                rospy.sleep(0.3)
                # Move Arm
                self.moveTrajectory(plan, typeOfCurve)

                # Send cooling commands to arduino
                if int(fields[9]) == 1:
                    self.arduinoPub.publish("M106 S1")
                else:
                    self.arduinoPub.publish("M106 S0")

                # Pause robot
                print "Pausing for"
                print rospy.Duration(pause, 0)
                if not rospy.is_shutdown():
                    rospy.sleep(rospy.Duration(pause, 0))
                if rospy.is_shutdown():
                    print 'Goodbye!'

            # Send cooling commands to arduino
            if int(fields[9]) == 1:
                self.arduinoPub.publish("M106 S1")
            else:
                self.arduinoPub.publish("M106 S0")

            while self.touchPlatformFlag is True:
                print 'Waiting for user to release the platform'
                rospy.sleep(0.1)

            # Send reponse to Rhino
            self.rhinoPub.publish('Motion.\r')
            self.sentCounter = self.sentCounter+1
            print 'Sent Counter:%i' % self.sentCounter


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('jaco2moveit', anonymous=True)

    try:
        brain = jaco2moveit()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
