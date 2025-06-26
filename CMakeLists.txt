cmake_minimum_required(VERSION 3.0.2)
project(antropomorphic_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()


catkin_package(
 CATKIN_DEPENDS rospy
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/antropomorphic_project/ik_antropomorphic_arm.py
  src/antropomorphic_project/fk_antropomorphic_arm.py
  src/antropomorphic_project/generate_matrixes.py
  src/antropomorphic_project/ellipsoidal_motion.py
  src/antropomorphic_project/move_joints.py
  src/antropomorphic_project/rviz_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
