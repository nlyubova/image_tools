cmake_minimum_required(VERSION 2.8.3)
project(image_tools)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp cv_bridge image_transport
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

SET(SRC
  src/image_convert_node.cpp
  src/converter.cpp
  include/image_tools/converter.hpp
)

add_executable(image_convert_node ${SRC})

 target_link_libraries(image_convert_node
   ${catkin_LIBRARIES}
 )
