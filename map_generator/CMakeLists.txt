cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  eigen_conversions
  geometry_msgs
  visualization_msgs
  std_msgs
  tf
  tf2
  rviz_visual_tools
  pcl_ros
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_core
  interactive_markers
  nav_msgs)

find_package(octomap REQUIRED)
find_package(HDF5 REQUIRED)
find_package(MPI REQUIRED)

add_message_files(
 FILES
 WsSphere.msg
 WorkSpace.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS
      include
  LIBRARIES
      discretization
      kinematics
      hdf5_dataset
      utility
      reachability
      centering
      map_generation
      roscpp
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS system_lib #?
)


link_directories(${HDF5_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR} ${PCL_INCLUDE_DIR})

add_library(discretization   src/${PROJECT_NAME}/discretization.cpp )
add_library(kinematics   src/${PROJECT_NAME}/kinematics.cpp )
add_library(hdf5_dataset src/${PROJECT_NAME}/hdf5_dataset.cpp)
add_library(utility src/${PROJECT_NAME}/utility.cpp)
add_library(reachability  src/${PROJECT_NAME}/reachability.cpp)
add_library(centering  src/${PROJECT_NAME}/centering.cpp)
add_library(map_generation  src/${PROJECT_NAME}/map_generation.cpp)

add_dependencies(discretization map_generator_generate_messages_cpp)
add_dependencies(utility map_generator_generate_messages_cpp)

target_link_libraries(discretization  ${catkin_LIBRARIES} utility reachability ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(kinematics  ${catkin_LIBRARIES} utility -llapack)
target_link_libraries(utility  ${catkin_LIBRARIES})
target_link_libraries(reachability  ${catkin_LIBRARIES} utility)
target_link_libraries(centering  ${catkin_LIBRARIES} utility)
target_link_libraries(hdf5_dataset  ${catkin_LIBRARIES} utility ${HDF5_LIBRARIES} ${MPI_CXX_LIBRARIES})
target_link_libraries(map_generation  ${catkin_LIBRARIES} discretization hdf5_dataset utility reachability centering)

install(TARGETS discretization kinematics hdf5_dataset utility reachability centering map_generation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(create_reachability_map_node src/nodes/create_reachability_map_node.cpp)
add_dependencies(create_reachability_map_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_reachability_map_node map_generation ${catkin_LIBRARIES})

add_executable(create_inverse_reachability_map src/nodes/create_inverse_reachability_map.cpp)
add_dependencies(create_inverse_reachability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_inverse_reachability_map discretization kinematics  hdf5_dataset ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(load_reachability_map src/nodes/load_reachability_map.cpp)
add_dependencies(load_reachability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_reachability_map hdf5_dataset ${catkin_LIBRARIES})

add_executable(OG_create_RM src/nodes/OG_create_RM.cpp)
add_dependencies(OG_create_RM ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OG_create_RM discretization kinematics hdf5_dataset ${HDF5_INCLUDE_DIR} ${catkin_LIBRARIES})


