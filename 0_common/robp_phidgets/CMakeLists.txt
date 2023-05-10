cmake_minimum_required(VERSION 3.0.2)

project(robp_phidgets
  VERSION 1.0.0
  DESCRIPTION ""
  LANGUAGES C CXX
)

add_compile_options(-std=c++17)
set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
    STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(catkin REQUIRED COMPONENTS
  angles
  dynamic_reconfigure
  imu_filter_madgwick
  message_generation
  nodelet
  robp_msgs
  roscpp
  roslaunch
  roslib
  sensor_msgs
  std_srvs
  tf2_geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Encoder.cfg
  cfg/Logging.cfg
  cfg/Motor.cfg
  cfg/Spatial.cfg
  cfg/Temperature.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robp_phidgets_util robp_phidgets_motor robp_phidgets_motors robp_phidgets_encoder robp_phidgets_encoders robp_phidgets_spatial robp_phidgets_temperature
  CATKIN_DEPENDS angles nodelet dynamic_reconfigure robp_msgs roscpp roslib sensor_msgs std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(robp_phidgets_util src/util.cpp src/error.cpp)
add_library(robp_phidgets_logging_nodelet src/logging_nodelet.cpp)
add_library(robp_phidgets_motor src/motor.cpp)
add_library(robp_phidgets_motors src/motors.cpp)
add_library(robp_phidgets_motors_nodelet src/motors_nodelet.cpp)
add_library(robp_phidgets_encoder src/encoder.cpp)
add_library(robp_phidgets_encoders src/encoders.cpp)
add_library(robp_phidgets_encoders_nodelet src/encoders_nodelet.cpp)
add_library(robp_phidgets_spatial src/spatial.cpp)
add_library(robp_phidgets_spatial_nodelet src/spatial_nodelet.cpp)
add_library(robp_phidgets_temperature src/temperature.cpp)
add_library(robp_phidgets_temperature_nodelet src/temperature_nodelet.cpp)

add_dependencies(robp_phidgets_logging_nodelet ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(robp_phidgets_motor ${catkin_EXPORTED_TARGETS})
add_dependencies(robp_phidgets_motors ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(robp_phidgets_motors_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(robp_phidgets_encoder ${catkin_EXPORTED_TARGETS})
add_dependencies(robp_phidgets_encoders ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(robp_phidgets_encoders_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(robp_phidgets_spatial ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(robp_phidgets_spatial_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(robp_phidgets_temperature ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(robp_phidgets_temperature_nodelet ${catkin_EXPORTED_TARGETS})

target_link_libraries(robp_phidgets_logging_nodelet ${catkin_LIBRARIES} phidget22)
target_link_libraries(robp_phidgets_motor ${catkin_LIBRARIES} phidget22 robp_phidgets_util)
target_link_libraries(robp_phidgets_motors ${catkin_LIBRARIES} robp_phidgets_motor)
target_link_libraries(robp_phidgets_motors_nodelet ${catkin_LIBRARIES} robp_phidgets_motors)
target_link_libraries(robp_phidgets_encoder ${catkin_LIBRARIES} phidget22 robp_phidgets_util)
target_link_libraries(robp_phidgets_encoders ${catkin_LIBRARIES} robp_phidgets_encoder)
target_link_libraries(robp_phidgets_encoders_nodelet ${catkin_LIBRARIES} robp_phidgets_encoders)
target_link_libraries(robp_phidgets_spatial ${catkin_LIBRARIES} phidget22 robp_phidgets_util)
target_link_libraries(robp_phidgets_spatial_nodelet ${catkin_LIBRARIES} robp_phidgets_spatial)
target_link_libraries(robp_phidgets_temperature ${catkin_LIBRARIES} phidget22 robp_phidgets_util)
target_link_libraries(robp_phidgets_temperature_nodelet ${catkin_LIBRARIES} robp_phidgets_temperature)

install(TARGETS robp_phidgets_logging_nodelet robp_phidgets_motor robp_phidgets_motors robp_phidgets_motors_nodelet robp_phidgets_encoder robp_phidgets_encoders robp_phidgets_encoders_nodelet robp_phidgets_spatial robp_phidgets_spatial_nodelet robp_phidgets_temperature robp_phidgets_temperature_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch)
