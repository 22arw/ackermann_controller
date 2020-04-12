cmake_minimum_required(VERSION 2.8.3)
project(ackermann_controller)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    controller_interface
    hardware_interface
    realtime_tools
    urdf
    nav_msgs)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ackermann_controller
 CATKIN_DEPENDS hardware_interface controller_interface urdf
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(ackermann_controller
  src/ackermann_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

add_dependencies(ackermann_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ackermann_controller ${catkin_LIBRARIES})

install(TARGETS ackermann_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  ackermann_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)