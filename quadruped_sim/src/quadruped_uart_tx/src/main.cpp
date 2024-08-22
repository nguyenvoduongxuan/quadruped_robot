/*
 *  main.cpp
 *  Author: nguyenvdx
 */
#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <boost/bind.hpp>
#include <vector>
#include "main.hpp"

#define RATE 50

void angle_callback(const control_msgs::JointControllerState::ConstPtr& msg, int index)
{
	Angle_rad[index] = msg->process_value;
    uint16_t signv = 0;
    if (Angle_rad[index] < 0) 
    {
        Angle_rad[index] = -Angle_rad[index];
        signv = 1;
    }
	PWM_data[index] = (uint16_t)(Angle_rad[index]*240.0/M_PI);
    PWM_data[index] |= (signv << 8);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "uart_tx"); 
    ros::NodeHandle node_handle;
    
    // gazebo value topics
    std::string value_topics[] = 
    {
          "/quadruped_controller/FR1_joint/state",
          "/quadruped_controller/FR2_joint/state",
          "/quadruped_controller/FR3_joint/state",
          "/quadruped_controller/RR1_joint/state",
          "/quadruped_controller/RR2_joint/state",
          "/quadruped_controller/RR3_joint/state",
          "/quadruped_controller/RL1_joint/state",
          "/quadruped_controller/RL2_joint/state",
          "/quadruped_controller/RL3_joint/state",
          "/quadruped_controller/FL1_joint/state",
          "/quadruped_controller/FL2_joint/state",
          "/quadruped_controller/FL3_joint/state",
    };

    // Subscriber that read angles value
    std::vector<ros::Subscriber> subscribers;
    for(int i = 0; i < 12; i++)
    {
        ros::Subscriber new_subscriber = node_handle.subscribe<control_msgs::JointControllerState>(value_topics[i], 1000,
			boost::bind(angle_callback, _1, i));
        subscribers.push_back(new_subscriber);
    }

    UART_Config_Jetson();
    
    ros::Rate loop_rate(RATE);
    while(ros::ok())
    {
        for (int i = 0; i < 12; i++)
        {
            Data_Trans[i+1] = PWM_data[i];
        }
        write(fid, Data_Trans, 14*sizeof(uint16_t));
    	ros::spinOnce();
		loop_rate.sleep();
    }
    close(fid);

    return 0;
}
