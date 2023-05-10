cmake_minimum_required(VERSION 3.0)
project(path_follower)

find_package(catkin REQUIRED COMPONENTS 
	rospy
	std_msgs
  	robp_msgs	
	nav_msgs
	tf2
  	tf2_ros
  	tf_conversions
	
)


catkin_package(
	CATKIN_DEPENDS 
	rospy 
	message_runtime 
	std_msgs
  	robp_msgs
	nav_msgs
	tf2
  	tf2_ros
  	tf_conversions
)

catkin_install_python(PROGRAMS 
	scripts/pure_pursuit.py
	scripts/generate_trajectory.py
	scripts/cartesian_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
