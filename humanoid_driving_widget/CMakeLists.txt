cmake_minimum_required(VERSION 2.8.3)
project(humanoid_driving_widget)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs humanoid_driving_controller)

## Declare things to be passed to other projects
catkin_package(
    INCLUDE_DIRS src include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp roslib std_msgs
)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/humanoid_driving_widget/driving_widget.h
)


## Here we add QtCreator ui files
qt4_wrap_ui(UIC_FILES
  src/driving_widget.ui
)

QT4_ADD_RESOURCES(driving_widget_RESOURCES_RCC src/res/resources.qrc)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.


set(SOURCE_FILES
  ${MOC_FILES}
  src/driving_widget_node.cpp
  src/driving_widget.cpp
  include/humanoid_driving_widget/driving_widget.h
)
include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} src)

add_executable(driving_widget ${UIC_FILES} ${SOURCE_FILES} ${driving_widget_RESOURCES_RCC})
target_link_libraries(driving_widget ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(driving_widget ${catkin_EXPORTED_TARGETS})
add_dependencies(driving_widget humanoid_driving_controller_generate_messages_cpp)

add_library(${PROJECT_NAME} ${UIC_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install( TARGETS ${PROJECT_NAME} driving_widget
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


