cmake_minimum_required(VERSION 2.8.3)
project(jaco_simulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    controller_manager
    hardware_interface
    pr_ros_controllers
    pr_hardware_interfaces
    sensor_msgs
    gazebo_ros_control
    pluginlib
    )

find_package(Boost REQUIRED COMPONENTS system)
find_package(GAZEBO REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    std_msgs
    roscpp
    controller_manager
    hardware_interface
    sensor_msgs
    gazebo_ros_control
    pluginlib
)

###########
## Build ##
###########

## Specify additional locations of header files
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${libkindrv_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
add_library(jaco_simulation src/jaco_robot_hw_sim.cpp)
target_link_libraries(jaco_simulation ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(FILES robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
