cmake_minimum_required(VERSION 3.0.2)
project(reule_aux)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
  actionlib
  move_base_msgs
  tf2
  control_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
    BP_Res.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs geometry_msgs actionlib_msgs # Or other packages containing msgs
 )


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
)

# Add your executable
#add_executable(reuleaux_bp_to_nav src/reuleaux_bp_to_nav.cpp)
# Link against the required libraries
#target_link_libraries(reuleaux_bp_to_nav  ${catkin_LIBRARIES})
# Install targets
#install(TARGETS reuleaux_bp_to_nav  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
