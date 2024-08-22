cmake_minimum_required(VERSION 3.0.2)
project(quadruped_keyboard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quadruped_keyboard
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ramped_keyboard
    src/main.cpp 
    )
target_link_libraries(ramped_keyboard ${catkin_LIBRARIES})
