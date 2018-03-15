## Procedure to run JACO2 using ROS Laptop and Rhino Plugin (on a Windows PC):
Ensure that the ROS Laptop and Windows PC are connected to the same network *(TP-LINK_AP_F1_CE)*

[Windows ROS.net](https://github.com/uml-robotics/ROS.NET) : Change ROS_MASTER_URI to http://192.168.0.100:11311 (IP address of Ubuntu PC) and ROS_IP to the IP of Windows PC.

### Ubuntu Laptop (ROS):
```
Terminal 1:
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.100
cd catkin_ws_kinova
source devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300

Terminal 2:
cd catkin_ws_kinova
source devel/setup.bash
export ROS_IP=192.168.0.100
rosrun jaco_printing mainController.py

Terminal 3 (for testing):
Terminal 3 (for testing):
export ROS_IP=192.168.0.100
rostopic pub /command std_msgs/String '0,100,0,-10,0.07,-0.01,-1,0.1,-09,0,0,0,-10,2807,2,0,20, 4'

```

## Connecting Arduino to ROS

Follow instructions from: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
Example subscriber: http://wiki.ros.org/rosserial_arduino/Tutorials/Blink

*Instead of /dev/ttyUSB0 it may be /dev/ttyACM0 for some laptops*

```
Terminal 1: roscore
Terminal 2: rosrun rosserial_python serial_node.py /dev/ttyACM0
Terminal 3: rostopic pub /extruder std_msgs/String "G1 E20"
Terminal 4: rostopic echo /chatter
```

**Heat up the extruder: **
```
rostopic pub /extruder std_msgs/String "M104 S270"
```

**Get temperature of extruder (on /chatter topic):**
```
rostopic pub /extruder std_msgs/String "M114"
```
