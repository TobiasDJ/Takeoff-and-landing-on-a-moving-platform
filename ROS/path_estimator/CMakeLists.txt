cmake_minimum_required(VERSION 3.0.2)
project(path_estimator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  landing_pad_detector
  std_msgs
  mavros_msgs
  geometry_msgs
  message_generation
  gazebo_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg FILES
  Estimated_path.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_estimator
   CATKIN_DEPENDS roscpp landing_pad_detector std_msgs message_runtime mavros_msgs geometry_msgs gazebo_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(path_estimator_node src/path_estimator_node.cpp)
target_link_libraries(path_estimator_node ${catkin_LIBRARIES})
add_dependencies(path_estimator_node path_estimator_generate_messages_cpp)

