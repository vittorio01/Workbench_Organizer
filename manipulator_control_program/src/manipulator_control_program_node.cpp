#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "manipulator.h"
#include "manipulator.cpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char** argv) {
    cout << "started manipulator_control_program ROS node\n";
    vector3d base(0,0,0);
    manipulator_ur5 robot(base);
    cout << robot <<endl;

    return 0;
}

