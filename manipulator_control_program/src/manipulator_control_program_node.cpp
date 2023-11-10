#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>

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


    ros::Rate loop_rate(2000);
    while (ros::ok()) {
        cout << "[";
        for (int i=0;i<5;i++) {
            cout << interface.getPosition(i) << ", ";
        }
        cout << "]"<<endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    




    //vector3d base(0,0,0);
    //manipulator_ur5 robot(base);
    //cout << robot <<endl;
    
    return 0;
}

