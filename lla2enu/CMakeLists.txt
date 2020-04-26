cmake_minimum_required(VERSION 2.8.3)
project(lla2enu)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geodesy sensor_msgs tf nav_msgs message_generation)

## Custom message commands
add_message_files(
  FILES
  Dist.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(lla2enu_front src/lla2enu_front.cpp)
target_link_libraries(lla2enu_front ${catkin_LIBRARIES})

add_executable(lla2enu_obs src/lla2enu_obs.cpp)
target_link_libraries(lla2enu_obs ${catkin_LIBRARIES})

## Distance Node
add_executable(distance src/distance.cpp)
add_dependencies(distance lla2enu_generate_messages_cpp)
target_link_libraries(distance ${catkin_LIBRARIES})
