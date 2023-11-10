#ifndef __locosim_robot_interface_H__
#define __locosim_robot_interface_H__
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>
#include <iostream>

#define BUFFER_LENGHT 100
#define JOINTS_NUMBER 5

using namespace std;

class Locosim_robot_interface {
    private:
        static string names[];
        static float positions[];
        static float velocities[];
        static float efforts[];
        static ros::Subscriber subscriber;
    public:
        static void initialize(ros::NodeHandle &node);
        static void messageReadHandler(const sensor_msgs::JointState &msg);    
        static float getVelocity(const int jointNumber);
        static string getName(const int jointNumber);
        static float getEffort(const int jointNumber);
        static float getPosition(const int jointNumber);

};

string Locosim_robot_interface::names[JOINTS_NUMBER];
float Locosim_robot_interface::positions[JOINTS_NUMBER];
float Locosim_robot_interface::velocities[JOINTS_NUMBER];
float Locosim_robot_interface::efforts[JOINTS_NUMBER];
ros::Subscriber Locosim_robot_interface::subscriber;


#endif
