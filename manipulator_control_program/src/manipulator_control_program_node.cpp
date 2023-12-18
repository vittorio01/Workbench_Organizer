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

int main(int argc, char** argv) {
    cout << "started manipulator_control_program ROS node\n" ;

    ros::init(argc,argv,"data_listener");
    ros::NodeHandle node;
    
    Locosim_robot_interface interface;
    interface.initialize(node);

    UR5 manipulator;

    cout << manipulator <<endl;

    ros::Rate loop_rate(1000);
    while (ros::ok() && !interface.getSystemStatus()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    
    if (!ros::ok()) return 0;
    jointVector homePosition; 
    homePosition << 1.5,-0.75,0.7,-0.2,0.75,-1.5;

    interface.setPosition(homePosition);
    trajectoryPointVector startPosition=manipulator.get_end_effector_position(homePosition);
    trajectoryPointVector targetPosition;
    targetPosition << -0.82,0.23,-0.06,3.14,0,1;// 0.16,0.24,-0.68,3.14,0,1;
    
    trajectoryJointMatrix trajectory=manipulator.compute_trajectory(targetPosition,homePosition,5);
    int step=-1000;
    while (ros::ok()) {
        if (step>=0 && step<trajectory.cols()) {
            cout <<"current position: "<< manipulator.get_end_effector_position(interface.getPositions()).transpose()<< endl;
            cout << "current angles: "<<interface.getPositions().transpose() << endl;
            interface.setPosition(trajectory.col(step));
            cout << "new position: "<<manipulator.get_end_effector_position(trajectory.col(step)).transpose()<<endl;
            cout << "new angles: "<<trajectory.col(step).transpose() << endl;
            cout << "--------------------------------------------------"<<endl;
            
        }
        step=step+1;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

