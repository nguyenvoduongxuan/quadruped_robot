/*
 *  StateCommand.hpp
 *  Author: nguyenvdx
 */

#pragma once
#include <Eigen/Core>

struct State
{
    // robot's velocity
    float velocity[2];

    // robot's yaw rate
    float yaw_rate;

    // robot's height
    float robot_height;

    // current foot locations in the base_link_world coordinate frame
    Eigen::Matrix<float, 3, 4> foot_locations;

    // position of the base_link coordinate frame in base_link_world
    float body_local_position[3];

    // orientation of the base_link coordinate frame in base_link_world
    float body_local_orientation[3];

    int ticks;

    // Constructor
    State(float default_height)
    : velocity{0.0, 0.0}, body_local_position{0.0, 0.0, 0.0},
      body_local_orientation{0.0, 0.0, 0.0}
    {
        yaw_rate = 0.0;
        robot_height = -default_height;
        foot_locations.setZero();
        ticks = 0;
    }
};

struct Command
{
    // commanded velocity
    float velocity[2];

    // commanded yaw rate
    float yaw_rate;

    // commanded robot height
    float robot_height;

    // trot request
    bool trot_event;

    // Constructor
    Command(float default_height)
    : velocity{0.0, 0.0}
    {
        yaw_rate = 0.0;
        robot_height = -default_height;
        trot_event = false;
    }
};
