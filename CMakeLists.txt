cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  # message_generation
  roscpp
  std_msgs
  pcl_ros
  laser_geometry
)


# generate_messages(
#  DEPENDENCIES
#  std_msgs
#)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controller
#  DEPENDS system_lib
 CATKIN_DEPENDS 
 roscpp 
 std_msgs
# message_runtime 

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar src/lidar.cpp)
target_link_libraries(lidar  ${catkin_LIBRARIES})
