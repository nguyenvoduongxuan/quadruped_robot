/*
 *  main.cpp
 *  Author: nguyenvdx
 */

#include <ros/ros.h>
#include <Eigen/Dense>
#include "quadruped_controller/InverseKinematics.hpp"
#include "quadruped_controller/Transformations.hpp"
#include "quadruped_controller/RobotController.hpp"
#include <vector>
#include <string>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>

#define RATE 50
int main(int argc, char* argv[])
{
    // robot body dimensions - body length, body width
    const float body_dimensions[] = {0.23, 0.112};

    // robot leg dimensions - l1, l2, l3, l4
    const float leg_dimensions[] = {0.0, 0.04, 0.100, 0.094333};

    // gazebo command topics
    std::string command_topics[] = 
    {
          "/quadruped_controller/FR1_joint/command",
          "/quadruped_controller/FR2_joint/command",
          "/quadruped_controller/FR3_joint/command",
          "/quadruped_controller/FL1_joint/command",
          "/quadruped_controller/FL2_joint/command",
          "/quadruped_controller/FL3_joint/command",
          "/quadruped_controller/RR1_joint/command",
          "/quadruped_controller/RR2_joint/command",
          "/quadruped_controller/RR3_joint/command",
          "/quadruped_controller/RL1_joint/command",
          "/quadruped_controller/RL2_joint/command",
          "/quadruped_controller/RL3_joint/command",
    };

    
    // ROS node initialization
    ros::init(argc, argv, "Robot_Controller");
    ros::NodeHandle node_handle;

    // RobotController
    RobotController quadruped(body_dimensions, leg_dimensions);
    ros::Subscriber dsd = node_handle.subscribe("quadruped_joy/joy_ramped", 1,
            &RobotController::keyboard_command, &quadruped);

    // Inverse Kinematics 
    InverseKinematics quadruped_IK(body_dimensions, leg_dimensions);

    // Gazebo command publishers
    std::vector<ros::Publisher> publishers;
    for(int i = 0; i < 12; i++)
    {
        ros::Publisher new_publisher = node_handle.advertise<std_msgs::Float64>
            (command_topics[i],1);
        publishers.push_back(new_publisher);
    }

    // main while loop rate
    ros::Rate loop_rate(RATE);
    while(ros::ok())
    {
        // new leg positions
        Eigen::Matrix<float, 3, 4> leg_positions = quadruped.run();

        // body local position
        float dx = quadruped.state.body_local_position[0];
        float dy = quadruped.state.body_local_position[1];
        float dz = quadruped.state.body_local_position[2];

        // body local orientation
        float roll = quadruped.state.body_local_orientation[0];
        float pitch = quadruped.state.body_local_orientation[1];
        float yaw = quadruped.state.body_local_orientation[2];

        // inverse kinematics -> joint angles
        std::vector<double> angles = quadruped_IK.inverse_kinematics(leg_positions,
                dx, dy, dz, roll, pitch, yaw);

        // publish joint angle commands
        for(int i = 0; i < 12; i++)
        {
            if(!isnan(angles[i]))
            {
                std_msgs::Float64 command_message;
                command_message.data = angles[i];
                publishers[i].publish(command_message);
            }
        }
        
        // spin
        ros::spinOnce();
        
        // sleep
        loop_rate.sleep();
    }

    return 0;
}
