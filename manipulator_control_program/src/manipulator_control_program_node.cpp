#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>

#include "ur5/UR5.h"
#include "ur5/UR5.cpp"
#include "locosim_robot_interface/locosim_robot_interface.h"
#include "locosim_robot_interface/locosim_robot_interface.cpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char** argv) {
    cout << "started manipulator_control_program ROS node\n" ;

    ros::init(argc,argv,"data_listener");
    ros::NodeHandle node;
    
    Locosim_robot_interface interface;
    interface.initialize(node);

    UR5 manipulator;

    cout << manipulator <<endl;

    ros::Rate loop_rate(2);
    while (ros::ok() && !interface.getSystemStatus()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    
    if (!ros::ok()) return 0;

    end_effector_coordinates end_effector;

    JointVector v;
    v << M_PI_2, -M_PI_4, M_PI_2, 0, 0, M_PI;
    interface.setPosition(v);
    while (ros::ok()) {
        
        cout << interface << endl;
        cout << "end_effector_position: "<<manipulator.compute_direct_kinematics(interface.getPositions()).position.transpose()<<endl<< endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

