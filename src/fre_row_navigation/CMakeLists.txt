cmake_minimum_required(VERSION 3.0.2)
project(fre_row_navigation)

## Compile as C++17
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "-std=c++17")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support.")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  genmsg
  actionlib
  actionlib_msgs
  visualization_msgs
  laser_geometry
  tf2
  tf2_ros
  pcl_ros
  dynamic_reconfigure
  messages
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  CrawlRow.action

)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/crawl_row_param.cfg
  cfg/PidTester.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
 LIBRARIES  fre_row_navigation #messages
  CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(fre_row_navigation
   src/client.cpp
   src/pid.cpp			#uso queste classi anche fuori da questo package quindi devo metterle nell'add_library
   src/rectangle.cpp
 )

add_executable(crawl_row_node 
  src/crawl_row_node.cpp 
  src/pid.cpp 
  src/rectangle.cpp 		
  src/cone.cpp
)

add_dependencies(crawl_row_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(crawl_row_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pid_tester src/pid_tester.cpp src/pid.cpp )
add_dependencies(pid_tester   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_tester   ${catkin_LIBRARIES} ${PCL_LIBRARIES})
