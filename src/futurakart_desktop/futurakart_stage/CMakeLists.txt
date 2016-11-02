cmake_minimum_required(VERSION 2.8.3)
project(futurakart_stage)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/start.launch)

install(DIRECTORY launch maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
