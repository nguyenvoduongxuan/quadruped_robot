cmake_minimum_required(VERSION 3.0.2)
project(before_start_everything)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  LIBRARIES before_start_everything
)
