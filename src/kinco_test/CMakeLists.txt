cmake_minimum_required(VERSION 3.0.2)
project(kinco_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(pitch_driver src/pitch_driver.cpp)
target_link_libraries(pitch_driver ${catkin_LIBRARIES})
add_dependencies(pitch_driver ${catkin_EXPORTED_TARGETS})

add_executable(multiple_pitch src/multiple_pitch.cpp)
target_link_libraries(multiple_pitch ${catkin_LIBRARIES})
add_dependencies(multiple_pitch ${catkin_EXPORTED_TARGETS})

add_executable(pitch_logger src/pitch_logger.cpp)
target_link_libraries(pitch_logger ${catkin_LIBRARIES})
add_dependencies(pitch_logger ${catkin_EXPORTED_TARGETS})
