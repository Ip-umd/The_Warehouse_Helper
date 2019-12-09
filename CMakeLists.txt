cmake_minimum_required(VERSION 2.8.3)
project(The_Warehouse_Helper)
add_compile_options(-std=c++11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   move_base_msgs#   sensor_msgs#   std_msgs
# )

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES The_Warehouse_Helper
#  CATKIN_DEPENDS geometry_msgs move_base_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(${The_Warehouse_Helper}
#   src/${The_Warehouse_Helper}/The_Warehouse_Helper.cpp
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/The_Warehouse_Helper_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${The_Warehouse_Helper}-test test/main.cpp test/ObstacleMapTest.cpp test/NodeParamTest.cpp)
#if(TARGET ${The_Warehouse_Helper}-test)
#   target_link_libraries(${The_Warehouse_Helper}-test ${The_Warehouse_Helper})
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(warehouseHelper src/main.cpp src/NodeParam.cpp src/AStar.cpp src/ObstacleMap.cpp)

target_link_libraries(warehouseHelper
  ${catkin_LIBRARIES}
)
target_link_libraries(warehouseHelper ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_rostest_gtest(
  warehouseHelperTest
  test/warehouseHelperTest.launch  
  test/main.cpp
  test/NodeParamTest.cpp
  test/ObstacleMapTest.cpp
  test/AStarTest.cpp
  
  src/NodeParam.cpp
  src/ObstacleMap.cpp
  src/AStar.cpp
  )

  target_link_libraries(
  warehouseHelperTest
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
  add_dependencies(
  warehouseHelper
  warehouseHelperTest
  ${catkin_EXPORTED_TARGETS}
  )
endif()


