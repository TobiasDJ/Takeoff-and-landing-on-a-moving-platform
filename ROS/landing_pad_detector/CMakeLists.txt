cmake_minimum_required(VERSION 3.0.2)
project(landing_pad_detector)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  message_generation
)


find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

ADD_DEFINITIONS(${QT_DEFINITIONS})


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg FILES
  Detected_position.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES landing_pad_detector
   CATKIN_DEPENDS roscpp std_msgs tf message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(position_publisher_node src/position_publisher_node.cpp)
target_link_libraries(position_publisher_node ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(position_publisher_node landing_pad_detector_generate_messages_cpp)
