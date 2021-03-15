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
link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})


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

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(jaco_hardware_node jaco_hardware_generate_messages_cpp)
set(KINOVA_ARM_LIBS ${catkin_LIBRARIES})
set(KINOVA_ARM_LIBS
    ${KINOVA_ARM_LIBS}
    -l:USBCommLayerUbuntu.so
    -l:USBCommandLayerUbuntu.so)


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#    ${KINOVA_ARM_LIBS}
#  )

install(FILES robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
