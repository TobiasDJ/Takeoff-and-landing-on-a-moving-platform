cmake_minimum_required(VERSION 3.0.2)
project(mission_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  path_estimator
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mission_controller
   CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs message_runtime path_estimator
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_controller_node src/mission_controller_node.cpp)
target_link_libraries(mission_controller_node ${catkin_LIBRARIES})
