cmake_minimum_required(VERSION 3.0.2)
project(odt2025)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pitch_controller src/pitch_controller.cpp)
target_link_libraries(pitch_controller ${catkin_LIBRARIES})
add_dependencies(pitch_controller ${catkin_EXPORTED_TARGETS})

add_executable(yaw_controller src/yaw_controller.cpp)
target_link_libraries(yaw_controller ${catkin_LIBRARIES})
add_dependencies(yaw_controller ${catkin_EXPORTED_TARGETS})

add_executable(velocity_calculator src/velocity_calculator.cpp)
target_link_libraries(velocity_calculator ${catkin_LIBRARIES})
add_dependencies(velocity_calculator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
