cmake_minimum_required(VERSION 2.8.3)
project(thor_mang_driving_controller)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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

add_message_files(DIRECTORY msg FILES ${MSG_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vigir_drake_matlab
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
add_executable(driving_controller src/thor_mang_driving_controller_node.cpp src/thor_mang_driving_controller.cpp)
add_dependencies(driving_controller thor_mang_driving_controller_generate_messages_cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(thor_driving_controller vigir_planning_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(driving_controller
   ${catkin_LIBRARIES}
 )
