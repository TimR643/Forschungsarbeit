cmake_minimum_required(VERSION 3.0.2)
project(panda_spacemouse_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/spacemouse_twist_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
