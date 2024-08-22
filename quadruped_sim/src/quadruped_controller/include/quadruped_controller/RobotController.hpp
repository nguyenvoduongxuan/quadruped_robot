/*
 *  RobotController.hpp
 *  Author: nguyenvdx
 */

#pragma once
#include <Eigen/Core>

#include "quadruped_controller/StateCommand.hpp"
#include "quadruped_controller/TrotGaitController.hpp"

class RobotController
{
    public:

        // CrawlGaitController class constructor - set body and leg dimensions
        RobotController(const float body[], const float legs[]);

        // Main run function, return leg positions in the base_link_world frame
        Eigen::Matrix<float, 3, 4> run();

		// ROS keyboard callback
        void keyboard_command(const sensor_msgs::Joy::ConstPtr& msg);

        // robot's state
        State state;

    private:
        // variables
        float body[2];
        float legs[4];

        float delta_x;
        float delta_y;
        float x_shift_front;
        float x_shift_back;

        // trot gait controller
        TrotGaitController trotGaitController;

        Command command;

        // return default_stance
        Eigen::Matrix<float, 3, 4> default_stance();
};
