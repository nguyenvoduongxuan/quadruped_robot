/*
 *  RobotController.cpp
 *  Author: nguyenvdx
 */

#include "quadruped_controller/RobotController.hpp"
#include "quadruped_controller/StateCommand.hpp"

#include <tf/transform_datatypes.h>

#define ROBOT_HEIGHT 0.15
#define X_SHIFT_FRONT 0.0075
#define X_SHIFT_BACK -0.03

///////////////////////////////////////////////////////////////////////////////
RobotController::RobotController(const float body[], const float legs[])
: state(ROBOT_HEIGHT), command(ROBOT_HEIGHT),
  delta_x(body[0] * 0.5), delta_y(body[1]*0.5 + legs[1]),
  x_shift_front(X_SHIFT_FRONT), x_shift_back(X_SHIFT_BACK),
  trotGaitController(default_stance(),0.18, 0.24, 0.02)
{
    // body dimensions
    this->body[0] = body[0];
    this->body[1] = body[1];

    // leg dimensions
    this->legs[0] = legs[0];

    this->legs[1] = legs[1];
    this->legs[2] = legs[2];
    this->legs[3] = legs[3];

    state.foot_locations = default_stance();
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RobotController::default_stance()
{
    // FR, FL, RR, RL
    Eigen::Matrix<float, 3, 4> default_coordinates;
    default_coordinates <<   delta_x + x_shift_front, // FR - x
                             delta_x + x_shift_front, // FL - x
                            -delta_x + x_shift_back,  // RR - x
                            -delta_x + x_shift_back,  // RL - x

                            -delta_y,   // FR - y
                             delta_y,   // FL - y
                            -delta_y,   // RR - y
                             delta_y,   // RL - y

                             0,     // FR - z
                             0,     // FL - z
                             0,     // RR - z
                             0;     // RL - z

    return default_coordinates;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RobotController::run()
{
    return(trotGaitController.run(state, command)); 
}

///////////////////////////////////////////////////////////////////////////////
void RobotController::keyboard_command(const sensor_msgs::Joy::ConstPtr& msg)
{
    trotGaitController.updateStateCommand(msg, state, command);
}

