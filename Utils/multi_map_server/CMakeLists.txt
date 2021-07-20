cmake_minimum_required(VERSION 2.8.3)
project(multi_map_server)

cmake_policy(SET CMP0046 NEW)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pose_utils
  message_generation
  roscpp
  visualization_msgs
  tf
  std_srvs
  laser_geometry
  pose_utils
  quadrotor_msgs
)


add_message_files(DIRECTORY msg
   FILES
   MultiOccupancyGrid.msg
   MultiSparseMap3D.msg
   SparseMap3D.msg
   VerticalOccupancyGridList.msg
   #plan_cmd.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES irobot_msgs
   CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs quadrotor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(multi_map_visualization src/multi_map_visualization.cc)

add_dependencies(multi_map_visualization multi_map_server_messages_cpp)

target_link_libraries(multi_map_visualization 
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   ${pose_utils_LIBRARIES}
)
