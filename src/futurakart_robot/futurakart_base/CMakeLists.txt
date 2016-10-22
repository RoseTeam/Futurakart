cmake_minimum_required(VERSION 2.8.3)
project(futurakart_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diagnostic_updater
  geometry_msgs
  futurakart_msgs
  hardware_interface
  roscpp
  roslaunch
  roslint
  sensor_msgs
  std_msgs
  topic_tools
  dynamic_reconfigure
)
find_package(Boost REQUIRED COMPONENTS thread chrono)

generate_dynamic_reconfigure_options(
  cfg/PidCoeff.cfg
)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(futurakart_node
  src/futurakart_base.cpp
  src/futurakart_hardware.cpp
  src/futurakart_diagnostic_updater.cpp)
target_link_libraries(futurakart_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(futurakart_node futurakart_msgs_gencpp rosserial_msgs_gencpp ${PROJECT_NAME}_gencfg)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

roslaunch_add_file_check(launch/base.launch)
roslint_cpp()
roslint_add_test()
