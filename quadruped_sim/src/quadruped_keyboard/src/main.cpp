/*
 *  main.cpp
 *  Author: nguyenvdx
 */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "main.hpp"

#define RATE 50

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "keyboard_ramped"); 
    ros::NodeHandle node_handle;
    
    // target joystick state
    sensor_msgs::Joy target_joy;
    target_joy.axes = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    target_joy.buttons = {0,0,0,0,0,0,0,0,0,0,0};    
    	
    char key = ' ';
    // /quadruped_joy/joy_ramped publisher
    ros::Publisher joy_ramped_pub = node_handle.advertise<sensor_msgs::Joy>("quadruped_joy/joy_ramped", 1);
    std::cout << "Press X to start:" << std::endl;
  //   while(1)
  //   {
  //   	if ((key == 'X')||(key == 'x')) {
  //   	    std::cout << "Ready to start!" << std::endl;
  //   	    break;
	// }
  //   	key = getch();
  //   }
    ros::Rate loop_rate(RATE);
    while(ros::ok())
    {
		target_joy.axes[4] = 0.25; target_joy.axes[0] = 0.0;
    // 	key = ' ';
    // 	key = getch();  
    // 	if ((key == 'W')||(key == 'w'))	{target_joy.axes[4] = 0.25; target_joy.axes[0] = 0.0;} 
    // 	if ((key == 'S')||(key == 's'))	{target_joy.axes[4] = -0.25; target_joy.axes[0] = 0.0;}    
    // 	if ((key == 'A')||(key == 'a'))	{target_joy.axes[4] = 0.0; target_joy.axes[0] = 0.25;}    
    // 	if ((key == 'D')||(key == 'd'))	{target_joy.axes[4] = 0.0; target_joy.axes[0] = -0.25;}   
    // 	if ((key == 'X')||(key == 'x'))	{target_joy.axes[4] = 0.0; target_joy.axes[0] = 0.0;}   
		joy_ramped_pub.publish(target_joy);    
    // 	if (key == '\x03') {
    //   	    std::cout << "Stop receiving from keyboard!" << std::endl;
    //   	    break;
    //   	}  	
    	ros::spinOnce();
	loop_rate.sleep();
    }

    return 0;
}
