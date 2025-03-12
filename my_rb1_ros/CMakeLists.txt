cmake_minimum_required(VERSION 3.0.2)
project(my_rb1_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Declare service files
add_service_files(
  FILES
  Rotate.srv  # Update this to match the actual file name
)


## Generate messages and services
generate_messages(
  DEPENDENCIES
   std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Service server
add_executable(my_rb1_ros src/rotate_service.cpp)
target_link_libraries(my_rb1_ros ${catkin_LIBRARIES})
add_dependencies(my_rb1_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
