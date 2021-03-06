## Testing Complete System (reading data from text file)
Connect and turn on Jaco.

Connect Arduino (with extruder setup).

```
Terminal 1: roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300
Terminal 2: rosrun rosserial_python serial_node.py /dev/ttyACM0
Terminal 3: roslaunch jaco_printing mainController.launch
```
### Cool Down Extruder
```
Terminal 1: rosrun rosserial_python serial_node.py /dev/ttyACM0
Terminal 2: rostopic pub /extruder std_msgs/String "M104 S20"
```

## Pump
rostopic pub /extruder std_msgs/String "M106 S0"
rostopic pub /extruder std_msgs/String "M106 S1"

## Procedure to run JACO2 using ROS Laptop and Rhino Plugin (on a Windows PC):
Ensure that the ROS Laptop and Windows PC are connected to the same network *(TP-LINK_AP_F1_CE)*

[Windows ROS.net](https://github.com/uml-robotics/ROS.NET) : Change ROS_MASTER_URI to http://192.168.0.100:11311 (IP address of Ubuntu PC) and ROS_IP to the IP of Windows PC.

### Ubuntu Laptop (ROS):
```
Terminal 1:
export ROS_MASTER_URI=http://192.168.1.3:11311
export ROS_IP=192.168.1.3
cd catkin_ws_kinova
source devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300

Terminal 2:
cd catkin_ws_kinova
source devel/setup.bash
export ROS_IP=192.168.43.246
roslaunch jaco_printing mainController.launch

Terminal 3 (for testing):
export ROS_IP=192.168.43.246
rostopic pub /command std_msgs/String '0,8.02,-29.94,150,0,0,0,1,1,0,0, 80, 9'


```

## Connecting Arduino to ROS

Follow instructions from: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
Example subscriber: http://wiki.ros.org/rosserial_arduino/Tutorials/Blink

*Instead of /dev/ttyUSB0 it may be /dev/ttyACM0 for some laptops*

```
Terminal 1: roscore
Terminal 2: rosrun rosserial_python serial_node.py /dev/ttyACM0
Terminal 3: rostopic pub /extruder std_msgs/String "G1 E5 T2"
Terminal 4: rostopic echo /chatter
```

**Heat up the extruder: **
```
rostopic pub /extruder std_msgs/String "M104 S270"
```

**Cool down the extruder: **
```
rostopic pub /extruder std_msgs/String "M104 S20"
```

**Stop cooling : **
```
rostopic pub /extruder std_msgs/String "M106 S0"
```

**Get temperature of extruder (on /chatter topic):**
```
rostopic pub /extruder std_msgs/String "M114"
```
**Get Rotation angle:**
```
rostopic pub /extruder std_msgs/String "M16"

```

## Kinova Demo testing
rosrun kinova_demo pose_action_client.py -v -r j2s7s300 mdeg -- 0.01 0 0 0 10 10

## Jaco2 with MoveIt
There are two ways of controlling Jaco2 with MoveIt.
1. Move to one Cartesian Pose at a time ('moveToPose' method in jaco2moveit.py)
2. Move along a Cartesian Trajectory ('moveTrajectory' method in jaco2moveit.py')
```
Terminal 1: roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300
Terminal 2: roslaunch j2s7s300_moveit_config j2s7s300_demo.launch
Terminal 3: roslaunch jaco_printing jaco2moveit.launch
```

### Ubuntu Laptop (ROS):
```
Terminal 1 (Kinova Driver):
export ROS_MASTER_URI=http://192.168.1.3:11311
export ROS_IP=192.168.1.3
cd catkin_ws_kinova
source devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300

Terminal 2 (Arduino Connection):
export ROS_IP=192.168.1.3
cd catkin_ws_kinova
source devel/setup.bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

Terminal 3 (MoveIt):
cd catkin_ws_kinova
source devel/setup.bash
export ROS_IP=192.168.1.3
roslaunch j2s7s300_moveit_config j2s7s300_demo.launch

Terminal 4 (Move Jaco using MoveIt):
cd catkin_ws_kinova
source devel/setup.bash
export ROS_IP=192.168.1.3
roslaunch jaco_printing jaco2moveit.launch

Terminal 5 (Joint torque feedback):
cd catkin_ws_kinova
source devel/setup.bash
export ROS_IP=192.168.1.3
rosrun jaco_printing jaco2moveit_node2.py

Terminal 6 (for testing):
export ROS_IP=192.168.1.3
cd catkin_ws_kinova
source devel/setup.bash
rostopic pub /command std_msgs/String '0,0,0,150,0,0,0,1,0,0,2,1,80'
rostopic pub /command std_msgs/String '0,100,0,-150,0.6483,0,0.7614,0,1,0,9,1,80'
rostopic pub /command std_msgs/String '0,0,0,150,0,0,0,1,1,0,9,1,80'
rostopic pub /extruder std_msgs/String "G1 E10 T2"

```
Kill:
rosnode kill Talker


### Joint Position Control
```
rosrun kinova_demo joints_action_client.py -v -r j2s7s300 degree -- 0 0 0 0 0 0 0
```

Joint Angles for Origin: [95.6198272705,167.071685791,179.600875854,80.4585952759,13.7654829025,157.414764404,167.328979492]

```
rosrun kinova_demo joints_action_client.py -v j2s7s300 degree -- 95.6198272705 167.071685791 179.600875854 80.4585952759 13.7654829025 157.414764404 167.328979492
```
