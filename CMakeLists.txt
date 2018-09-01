cmake_minimum_required(VERSION 2.8.3)
project(rrt_implement)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
)


# Generate messages in the 'msg' folder
add_message_files (
  FILES
  position.msg
  ellipsoid.msg
  trajectory.msg
  world.msg
  point.msg
  pointsArray.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  ellipsoid_points.srv
  collision_detection.srv
)


# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   std_msgs   visualization_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt_implement
  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs roscpp rospy std_msgs tf visualization_msgs moveit_core moveit_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(rrt_implement
  src/rrt.cpp
#  src/planner.cpp
)


add_executable(rrtPlanner src/planner.cpp)
add_dependencies(rrtPlanner rrt_implement)
target_link_libraries(rrtPlanner rrt_implement ${catkin_LIBRARIES})


add_executable(environment_pub src/environment_publisher.cpp)
add_dependencies(environment_pub rrt_implement)
target_link_libraries(environment_pub rrt_implement ${catkin_LIBRARIES})


add_executable(trajectory_pub src/trajectory_publisher.cpp)
add_dependencies(trajectory_pub rrt_implement)
target_link_libraries(trajectory_pub rrt_implement ${catkin_LIBRARIES})


add_executable(ellipsoid_point_gen_srv src/ellipsoid_point_gen.cpp)
add_dependencies(ellipsoid_point_gen_srv rrt_implement)
target_link_libraries(ellipsoid_point_gen_srv rrt_implement ${catkin_LIBRARIES})


add_executable(collision_detection_srv src/collision_detect.cpp)
add_dependencies(collision_detection_srv rrt_implement)
target_link_libraries(collision_detection_srv rrt_implement ${catkin_LIBRARIES})


