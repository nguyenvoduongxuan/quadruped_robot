cmake_minimum_required(VERSION 3.0.2)
project(quadruped_controller)

## Find ROS related packages/libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

## Find other packages/libraries
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quadruped_controller
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(robot_controller_gazebo 
    src/main.cpp 
    src/InverseKinematics/InverseKinematics.cpp 
    src/RoboticsUtilities/Transformations.cpp
    src/RobotController/RobotController.cpp
    src/RobotController/TrotGaitController.cpp
    src/RobotController/GaitController.cpp
    )

target_link_libraries(robot_controller_gazebo Eigen3::Eigen ${catkin_LIBRARIES})
