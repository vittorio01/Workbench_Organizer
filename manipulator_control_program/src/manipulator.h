#ifndef __MANIPULATOR_STRUCTURE__
#define __MANIPULATOR_STRUCTURE__

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "revolute_joint.h"
#include "revolute_joint.cpp"
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#define _USE_MATH_DEFINES

#define BASE_JOINT 0
#define SHOULDER_JOINT 1
#define ELBOW_JOINT 2
#define WRIST1_JOINT 3
#define WRIST2_JOINT 4
#define END_EFFECTOR_JOINT 5

#define JOINT_NUMBER 6

#define D1 2
#define A2 3
#define A3 2
#define D4 4
#define D5 2
#define D6 6

#define BASE_JOINT_VALUES 0, 0
#define SHOULDER_JOINT_VALUES D1, 0
#define ELBOW_JOINT_VALUES 0, -A2
#define WRIST1_JOINT_VALUES D4, -A3
#define WRIST2_JOINT_VALUES -D5, 0
#define END_EFFECTOR_JOINT_VALUES D6, 0

#define BASE_JOINT_ORIENTATION 0
#define SHOULDER_JOINT_ORIENTATION M_PI_2
#define ELBOW_JOINT_ORIENTATION 0
#define WRIST1_JOINT_ORIENTATION 0
#define WRIST2_JOINT_ORIENTATION M_PI_2
#define END_EFFECTOR_JOINT_ORIENTATION M_PI_2

typedef Eigen::Vector3f vector3d;
typedef Eigen::Matrix3f rotationalMatrix;
typedef Eigen::Matrix4f transformationMatrix;
typedef Eigen::Vector4f vector4d;
typedef Eigen::Vector2f vector2d;

class manipulator_ur5
{
private:
    revolute_joint *joint_list[6];
    const vector3d robot_base_position;

public:
    manipulator_ur5(const vector3d _robot_base_position);
    ~manipulator_ur5();
    vector3d get_end_effector_position();
    vector2d get_end_effector_angles();
    bool set_end_effector_position(vector3d position, float angle1, float angle2);
    friend ostream &operator<<(ostream &ostream, manipulator_ur5 manipulator);
};

ostream &operator<<(ostream &ostream, manipulator_ur5 manipulator)
{
    ostream << "Manipulator type: UR5" << endl << "current end effector position: [";
    vector3d end_effector = manipulator.get_end_effector_position();
    for (int i = 0; i < 3; i++)
    {
        ostream << end_effector(i) << " ";
    }
    ostream << "]" << endl << "Manipulator joint_list: " << endl;
    for (int i = 0; i < JOINT_NUMBER; i++)
    {
        ostream << *manipulator.joint_list[i] << endl;
    }
    return ostream;
}
#endif
