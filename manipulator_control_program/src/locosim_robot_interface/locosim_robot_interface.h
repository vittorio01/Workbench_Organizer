#ifndef __locosim_robot_interface_H__
#define __locosim_robot_interface_H__
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define BUFFER_LENGHT                   1
#define GRIPPER_BUFFER_LENGHT           10
#define JOINTS_NUMBER                   6
#define GRIPPER_NUMBER                  2

#define ELBOW_NAME          "elbow_joint"
#define SHOULDER_LIFT_NAME  "shoulder_lift_joint"
#define SHOULDER_PAN_NAME   "shoulder_pan_joint"
#define WRIST_1_NAME        "wrist_1_joint"
#define WRIST_2_NAME        "wrist_2_joint"
#define WRIST_3_NAME        "wrist_3_joint" 



using namespace std;
typedef Eigen::Matrix<double, JOINTS_NUMBER,1> JointVector;

class Locosim_robot_interface {
    private:
        static const string names[];
        static JointVector positions;
        static JointVector velocities;
        static JointVector efforts;
        static ros::Subscriber subscriber;
        static ros::Publisher sender;
        static ros::Publisher gripper_sender;
        static bool online;

        static double lastGripperValues[];
        static double lastJointPosition[];
    public:
        static void initialize(ros::NodeHandle &node);
        static bool getSystemStatus();
        static void messageReadHandler(const sensor_msgs::JointState &msg);    
        static JointVector getPositions();
        static void setPosition(const JointVector& targetPosition);
        static void setGripperValue(const double gripperValues[]);
        friend ostream& operator<<(ostream& os,const Locosim_robot_interface& interface);


};

const string Locosim_robot_interface::names[JOINTS_NUMBER]={SHOULDER_PAN_NAME,SHOULDER_LIFT_NAME,ELBOW_NAME,WRIST_1_NAME,WRIST_2_NAME,WRIST_3_NAME};
JointVector Locosim_robot_interface::positions;
JointVector Locosim_robot_interface::velocities;
JointVector Locosim_robot_interface::efforts;
ros::Subscriber Locosim_robot_interface::subscriber;
ros::Publisher Locosim_robot_interface::gripper_sender;
ros::Publisher Locosim_robot_interface::sender;
double Locosim_robot_interface::lastGripperValues[GRIPPER_NUMBER];
double Locosim_robot_interface::lastJointPosition[JOINTS_NUMBER];
bool Locosim_robot_interface::online=false;

ostream& operator<<(ostream& os,const Locosim_robot_interface& interface);

#endif
