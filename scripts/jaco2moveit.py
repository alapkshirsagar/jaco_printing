#!/usr/bin/env python
import sys
import copy
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import math
from std_msgs.msg import String
from tf import transformations as homogeneous
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kinova_msgs.msg import *
from sensor_msgs.msg import JointState
import actionlib




## Class to control jaco using MoveIt. Also communicate with Rhino and Arduino
class jaco2moveit():

    jointData = []
    jointState = []
    def jointcallback(msg,joints):
        global jointData
        jointData = joints

    def jointstatecallback(msg,joints):
        global jointState
        jointState = joints.position

    def __init__(self):
            """Variables"""
            ##Home Position for Jaco
            self.homePosition = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home position of jaco2 in unit md
            self.newOrigin = [-0.032+0.01,-0.55+0.01-0.102,0.45+0.1+0.02,0,0,0]
            self.jacoToRhino = [self.newOrigin[0],self.newOrigin[1],self.newOrigin[2]]

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

            ## Trajectory for commanding the robot
            self.waypoints = []

            self.receivedCounter = 0 #Keep count of messages received from RhinoPlugin
            self.sentCounter = 0 #Keep count of responses sent to RhinoPlugin
            self.extruderTemperatureFlag = False #True if "CurrentTemperature" message is received
            self.extruderTemperature = 0 #Temperature of extruder
            self.extruderDesiredTemperature = 270 #Desired temperature of extruder

            self.rospack = rospkg.RosPack()
            self.packagePath = self.rospack.get_path('jaco_printing') #Get directory path to jaco_printing package

######################### Subscribers ######################################################
            self.sub_jointAngle = rospy.Subscriber('/j2s7s300_driver/out/joint_angles', JointAngles, self.jointcallback)
            self.sub_jointState = rospy.Subscriber('/j2s7s300_driver/out/joint_state', JointState, self.jointstatecallback)
            #Topic for getting response from Arduino
            rospy.Subscriber('/chatter',String, self.getArduinoResponse)


######################### Publishers #######################################################
            #Topic for sending response to RhinoPlugin
            self.rhinoPub = rospy.Publisher('/command_response', String, queue_size=10)

            #Topic for sending commands to Arduino
            self.arduinoPub = rospy.Publisher('/extruder', String, queue_size=10)


######################### Action Client ##################################################
            #Action Client for joint control
            action_address = '/j2s7s300_driver/joints_action/joint_angles'
            self.jointActionClient = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmJointAnglesAction)
            print 'Waiting for ArmJointAnglesAction server...'
            self.jointActionClient.wait_for_server()
            print 'ArmJointAnglesAction Server Connected'


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

            ## Calculate transformation Matrices
            self.RhinoToJacoTransformationMatrix()

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
    #This callback function is used to get responses from Arduino
    def getArduinoResponse(self,message):
        #Update temperature of extruder if previous message was "CurrentTemperature"
        if self.extruderTemperatureFlag:
            self.extruderTemperature = float(message.data)
            print self.extruderTemperature
            self.extruderTemperatureFlag = False
        if message.data == "CurrentTemperature":
            self.extruderTemperatureFlag = True



######################################## Methods #######################################
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


    def moveToPose(self,targetPose):
        ## Set a scaling factor for reducing the maximum joint velocity. Allowed values are in (0,1].
        self.group.set_max_velocity_scaling_factor(1.0)

        ## Planning to a Pose goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector
        print "============ Generating plan 1"
        self.group.set_pose_target(targetPose);

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
        rospy.sleep(1.0)
        #self.jointActionClientMethod(angle_set)
        rospy.sleep(1.0)
        print "current joint angle data from encoder:"
        # # in degree
        print jointData
        print "\n"


    def moveTrajectory(self):
    ## Set a scaling factor for reducing the maximum joint velocity. Allowed values are in (0,1].
        self.group.set_max_velocity_scaling_factor(1.0)
    ## We want the cartesian path to be interpolated at a resolution of 1 mm
    ## which is why we will specify 0.001 as the eef_step in cartesian
    ## translation.  We will specify the jump threshold as 0.0, effectively
    ## disabling it.

        #self.group.set_max_velocity_scaling_factor(0.1)

        (plan3, fraction) = self.group.compute_cartesian_path(
                           self.waypoints,   # waypoints to follow
                           0.001,        # eef_step
                           0.0) # jump_threshold
        self.group.execute(plan3)


        #print plan3
        rospy.sleep(1)
        print "================================ Moveit's plan ==================="
        print plan3
        print "\n"

        ptPos = plan3.joint_trajectory.points[-1].positions
        print "=================================="
        print "Last point of the current trajectory: "
        angle_set = list()

        for i in range (len(ptPos)):
            tempPos = ptPos[i]*180/3.14
            angle_set.append(tempPos)
            #if tempPos < 0:
                #tempPos += 360
            print "data" + str(i+1) + ": " + str(tempPos)
        print "\n"

        print "current joint angle data from encoder:"
        # in degree
        print jointData
        print "\n"

        self.jointActionClientMethod(angle_set)
        rospy.sleep(1)

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
        self.JtoR = homogeneous.concatenate_matrices(homogeneous.translation_matrix(self.jacoToRhino), homogeneous.euler_matrix(np.pi,0,np.pi/2,'rxyz'))
        print 'JtoR='
        print self.JtoR
        self.EtoG = homogeneous.euler_matrix(0,np.pi/2,0,'rxyz')
        print 'EtoG='
        print self.EtoG

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


    #This function is used to control Jaco and Extruder when testing using a text file.
    #Send the target position to moveitTrajectoryPlanner.
    #The fields of text file are: (0,X,Y,Z,Rx,Ry,Rz,Rw,Extrusion,Cooling,Pause,Speed,TypeOfCurve)
    def commandJacoTextFile(self):
        file = open(self.packagePath+'/scripts/multi-points_p1.txt')
        line = file.readline()
        #print line
        while line and not rospy.is_shutdown():
            line = line.strip()
            fields = line.split(',')
            if len(fields) == 13:
                #print fields
                position = [-1*float(fields[1])/1000, -1*float(fields[2])/1000, -1*float(fields[3])/1000]
                orientation = [float(fields[4]),float(fields[5]),float(fields[6]),float(fields[7])]
                pause = int (fields[10])
                MaxTranslationVelocity = float(fields[11])/3000
                MaxRotationalVelocity = float(fields[11])/3000

                ##Convert poses to Robot's frame
                JtoG = self.RhinoToJacoTransformation(position,orientation)
                position =  homogeneous.translation_from_matrix(JtoG)
                orientation = homogeneous.quaternion_from_matrix(JtoG)

                ##Add pose to waypoints
                wpose = geometry_msgs.msg.Pose()
                wpose.position.x = position[0]
                wpose.position.y = position[1]
                wpose.position.z = position[2]
                wpose.orientation.x = orientation[0]
                wpose.orientation.y = orientation[1]
                wpose.orientation.z = orientation[2]
                wpose.orientation.w = orientation[3]
                self.waypoints.append(copy.deepcopy(wpose))
                #self.moveToPose(wpose) ## Use pose move interface

                # Send cooling commands to arduino
                if int(fields[9]) == 1:
                    self.arduinoPub.publish("M106 S1")
                else:
                    self.arduinoPub.publish("M106 S0")

                ## Move arm when pause is not zero
                if pause != 0:
                    self.moveTrajectory()

                # Send extruder commands to arduino
                self.arduinoPub.publish("G1 E"+fields[8])

                #Pause robot
                print "Pausing for"
                print rospy.Duration(pause,0)
                if not rospy.is_shutdown():
                    rospy.sleep(rospy.Duration(pause,0))
                if rospy.is_shutdown():
                    print 'Goodbye!'
            line = file.readline()
        file.close()



if __name__=='__main__':
    #Initialize node
    rospy.init_node('jaco2moveit',anonymous=True)

    try:
        brain = jaco2moveit()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
