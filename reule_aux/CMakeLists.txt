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
  tf2_ros
  tf2_geometry_msgs
  control_msgs
  nav_msgs
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
   std_msgs geometry_msgs actionlib_msgs nav_msgs # Or other packages containing msgs
 )


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
)

# Add your executable
add_executable(move_to_bp_result src/move_to_bp_result.cpp)
# Link against the required libraries
target_link_libraries(move_to_bp_result  ${catkin_LIBRARIES})
# Install targets
install(TARGETS move_to_bp_result  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Add your executable
add_executable(move_to_bp_result_TIAGO src/move_to_bp_result_TIAGO.cpp)
# Link against the required libraries
target_link_libraries(move_to_bp_result_TIAGO  ${catkin_LIBRARIES})
# Install targets
install(TARGETS move_to_bp_result_TIAGO  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#Add your executable
add_executable(costmap_node src/costmap_node.cpp)
# Link against the required libraries
target_link_libraries(costmap_node  ${catkin_LIBRARIES})
# Install targets
install(TARGETS costmap_node  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
