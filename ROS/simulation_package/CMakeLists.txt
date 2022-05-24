cmake_minimum_required(VERSION 3.0.2)
project(simulation_package)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulation_package
   CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(initiate_drone_node src/initiate_drone_node.cpp)
target_link_libraries(initiate_drone_node ${catkin_LIBRARIES})
