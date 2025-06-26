cmake_minimum_required(VERSION 3.0.2)
project(antropomorphic_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  geometry_msgs
)
catkin_python_setup()


add_message_files(
      FILES
      EndEffector.msg 
)
generate_messages(
      DEPENDENCIES
      std_msgs
      geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime 

)

include_directories(
	launch
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/antropomorphic_project/antropomorphic_end_effector_mover.py
  src/antropomorphic_project/ik_antropomorphic_arm.py
  src/antropomorphic_project/fk_antropomorphic_arm.py
  src/antropomorphic_project/generate_matrixes.py
  src/antropomorphic_project/ellipsoidal_motion.py
  src/antropomorphic_project/move_joints.py
  src/antropomorphic_project/rviz_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
