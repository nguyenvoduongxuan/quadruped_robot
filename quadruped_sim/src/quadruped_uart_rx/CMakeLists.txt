cmake_minimum_required(VERSION 3.0.2)
project(quadruped_uart_rx)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quadruped_uart_rx
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(uart_rx
    src/main.cpp 
    )
target_link_libraries(uart_rx ${catkin_LIBRARIES})
