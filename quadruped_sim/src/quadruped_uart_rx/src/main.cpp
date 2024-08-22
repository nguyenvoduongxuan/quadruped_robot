/*
 *  main.cpp
 *  Author: nguyenvdx
 */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include "main.hpp"

#define RATE 50

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "uart_rx"); 
    ros::NodeHandle node_handle;
    
    // target joystick state
    sensor_msgs::Joy target_joy;
	sensor_msgs::Imu target_imu;
    target_joy.axes = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    target_joy.buttons = {0,0,0,0,0,0,0,0,0,0,0};    
    	
	UART_Config_Jetson();
	tcflush(fid, TCIOFLUSH);

    // /quadruped_joy/joy_ramped publisher
    ros::Publisher joy_ramped_pub = node_handle.advertise<sensor_msgs::Joy>("quadruped_joy/joy_ramped", 1);
    ros::Publisher imu_pub = node_handle.advertise<sensor_msgs::Imu>("quadruped_imu/base_link_orientation", 1);

    ros::Rate loop_rate(RATE);
    while(ros::ok())
    {
		while (fid != -1)
		{
			rx_length = read(fid, (void*)Data_RX_Raw, sizeof(Data_RX_Raw));
			if (rx_length == sizeof(Data_RX_Raw))
			{
				if ((Data_RX_Raw[0] == 'P') && (Data_RX_Raw[9] == 'I') && (Data_RX_Raw[27] == 'E')) 
				{
					for (int i = 0; i < 8; i++)
					{
						Data_RXP[i] = Data_RX_Raw[i+1];
					}
					for (int i = 0; i < 4; i++)
					{
						btf[i].input[0] = Data_RX_Raw[10 + i*4];
						btf[i].input[1] = Data_RX_Raw[11 + i*4];
						btf[i].input[2] = Data_RX_Raw[12 + i*4];
						btf[i].input[3] = Data_RX_Raw[13 + i*4];
						Data_RXI[i] = btf[i].output;
						if ((Data_RX_Raw[26] >> i) & 0x1) Data_RXI[i] = -Data_RXI[i];
					}
					break;
				}
			}
		}
   		if (Data_RXP[0] == 0x61)	{target_joy.axes[4] = 0.25; target_joy.axes[0] = 0.0;} 
    	if (Data_RXP[0] == 0x64)	{target_joy.axes[4] = -0.25; target_joy.axes[0] = 0.0;}    
    	if (Data_RXP[0] == 0x68)	{target_joy.axes[4] = 0.0; target_joy.axes[0] = 0.25;}    
    	if (Data_RXP[0] == 0x62)	{target_joy.axes[4] = 0.0; target_joy.axes[0] = -0.25;}   
    	if (Data_RXP[1] == 0x10)	{target_joy.axes[4] = 0.0; target_joy.axes[0] = 0.0;}   
		joy_ramped_pub.publish(target_joy);    

		target_imu.orientation.x = Data_RXI[0]*2.0;
		target_imu.orientation.y = Data_RXI[1]*2.0;
		target_imu.orientation.z = Data_RXI[2]*2.0;
		target_imu.orientation.w = Data_RXI[3];
		imu_pub.publish(target_imu);    

    	ros::spinOnce();
		loop_rate.sleep();
    }
    close(fid);

    return 0;
}
