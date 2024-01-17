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

#define NODE_FREQUENCY 200

#define UR5_SPAWN_POSITION      0.5,0.35,1.8
#define UR5_SPAWN_ORIENTATION   0,0,0

using namespace std;

int main(int argc, char** argv) {
    cout << "started manipulator_control_program ROS node\n" ;

    ros::init(argc,argv,"data_listener");
    ros::NodeHandle node;
    
    Locosim_robot_interface interface;
    interface.initialize(node);

    UR5 manipulator(pointVector(UR5_SPAWN_POSITION),Eigen::Matrix<double,3,1>(UR5_SPAWN_ORIENTATION));

    cout << manipulator <<endl;

    ros::Rate loop_rate(NODE_FREQUENCY);
    while (ros::ok() && !interface.getSystemStatus()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    
    if (!ros::ok()) return 0;
    jointVector homePosition=manipulator.get_joint_home_position();
    jointVector inverseHomePosition=manipulator.get_joint_home_position();
    inverseHomePosition(0)=-homePosition(0);
    interface.setPosition(homePosition);
    trajectoryPointVector startPosition=manipulator.get_end_effector_position(homePosition);
    trajectoryPointVector targetPosition;
    targetPosition << manipulator.get_end_effector_position(inverseHomePosition);  //0.82,0.57,1.10,3.14,0,0;
    trajectoryJointMatrix trajectory=manipulator.compute_trajectory(targetPosition,homePosition,NODE_FREQUENCY,0.1);
    targetPosition << 0.82,0.57,1.3,3.14,0,0;   //0.2,0.2,1.4,0,0,0; //0.82,0.57,0.90,3.14,0,0; 
    trajectoryJointMatrix trajectory2=manipulator.compute_trajectory(targetPosition,trajectory.col(trajectory.cols()-1),NODE_FREQUENCY,0.1);
    int step=-NODE_FREQUENCY;
    
    while (ros::ok()) {
        if (step>=0 && step<trajectory.cols()) {
            //cout <<"current position: "<< manipulator.get_end_effector_position(interface.getPositions()).transpose()<< endl;
            //cout << "current angles: "<<interface.getPositions().transpose() << endl;
            interface.setPosition(trajectory.col(step));
            //cout << "new position: "<<manipulator.get_end_effector_position(trajectory.col(step)).transpose()<<endl;
            //cout << "new angles: "<<trajectory.col(step).transpose() << endl;
            //cout << "--------------------------------------------------"<<endl;
            
        }else {
            if (step>=trajectory.cols() && step <(trajectory.cols()+trajectory2.cols())) {
                //cout <<"current position: "<< manipulator.get_end_effector_position(interface.getPositions()).transpose()<< endl;
                //cout << "current angles: "<<interface.getPositions().transpose() << endl;
                interface.setPosition(trajectory2.col((step-trajectory.cols())));
                //cout << "new position: "<<manipulator.get_end_effector_position(trajectory2.col((step-trajectory.cols()))).transpose()<<endl;
                //cout << "new angles: "<<trajectory2.col((step-trajectory.cols())).transpose() << endl;
                //cout << "--------------------------------------------------"<<endl;
            }
        }
        step=step+1;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

