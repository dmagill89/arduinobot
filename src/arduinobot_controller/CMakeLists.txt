cmake_minimum_required(VERSION 3.0.2)
project(arduinobot_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  AnglesConverter.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(angles_converter_cpp src/angles_converter.cpp)
add_dependencies(angles_converter_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(angles_converter_cpp ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS 
    scripts/angles_converter.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)