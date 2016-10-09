cmake_minimum_required(VERSION 2.8.3)
project(futurakart_bringup)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package()

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch/futurakart.launch)


install(PROGRAMS scripts/start_vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)