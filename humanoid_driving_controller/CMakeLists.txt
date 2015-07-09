cmake_minimum_required(VERSION 2.8.3)
project(humanoid_driving_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion -std=c++11")

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp  
  joy
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

set(MSG_DEPS
  std_msgs
)

set(MSG_FILES
  DrivingCommand.msg
  DrivingState.msg
)

set(SOURCE_FILES
  src/driving_controller_node.cpp
  src/driving_controller.cpp
  include/humanoid_driving_controller/driving_controller.h
)

add_message_files(DIRECTORY msg FILES ${MSG_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
  INCLUDE_DIRS include
CATKIN_DEPENDS
  DEPENDS joy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(driving_controller ${SOURCE_FILES})
add_dependencies(driving_controller humanoid_driving_controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(driving_controller
   ${catkin_LIBRARIES}
 )
