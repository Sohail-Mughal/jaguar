cmake_minimum_required(VERSION 2.8.3)
project(drrobot_player_driver3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs nav_msgs geometry_msgs message_generation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   GPSInfo.msg
   IMUData.msg
   MotorBoardInfo.msg
   MotorBoardInfoArray.msg
   MotorData.msg
   MotorDataArray.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 
   nav_msgs 
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES drrobot_player_driver3
  CATKIN_DEPENDS roscpp rospy tf std_msgs nav_msgs geometry_msgs message_runtime
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(drrobot_player_driver3
#   src/${PROJECT_NAME}/drrobot_player_driver3.cpp
# )

## Declare a cpp executable
add_executable(drrobot_player_driver3_node src/drrobot_player.cpp src/DrRobotMotionSensorDriver.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(drrobot_player_driver3_node ${catkin_EXPORTED_TARGETS})
#add_dependencies(DrRobotMotionSensorDriver ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(drrobot_player_driver3_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${ROS_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS drrobot_player_driver3 drrobot_player_driver3_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

