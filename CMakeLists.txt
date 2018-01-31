cmake_minimum_required(VERSION 2.8.3)
project(jaco_printing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  kinova_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  geometric_shapes
)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(Eigen REQUIRED)

find_package(kinova_driver REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(${kinova_driver_INCLUDE_DIRS})


add_executable(trajectory_client src/trajectory_client.cpp)
add_dependencies(trajectory_client kinova_msgs_generate_messages_cpp)
target_link_libraries(trajectory_client ${catkin_LIBRARIES})

add_executable(moveit_trajectory src/moveit_trajectory.cpp)
target_link_libraries(moveit_trajectory ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS moveit_trajectory DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#############
## Install ##
#############



#############
## Testing ##
#############
