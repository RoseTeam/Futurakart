cmake_minimum_required(VERSION 2.8.3)
project(futurakart_control)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  roscpp
  roslaunch
  std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(twist_to_ackermann src/twist_to_ackermann.cpp)
add_executable(twist_to_ackermann src/joy_interpreter_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(twist_to_ackermann
  ${catkin_LIBRARIES}
  
  target_link_libraries(joy_interpreter_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS twist_to_ackermann
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



roslaunch_add_file_check(launch/control.launch)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
