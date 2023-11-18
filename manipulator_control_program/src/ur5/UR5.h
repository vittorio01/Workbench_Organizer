#ifndef __MANIPULATOR_STRUCTURE__
#define __MANIPULATOR_STRUCTURE__

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "joints/revolute_joint.h"
#include "joints/revolute_joint.cpp"
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

#define D0 -0.163
#define A0 0

#define D1 0
#define A1 0

#define D2 0
#define A2 -0.42500

#define D3 0.134
#define A3 -0.39225

#define D4 0.100
#define A4 0

#define D5 0.100    
#define A5 0

#define BASE_JOINT_NAME "Base joint"
#define SHOULDER_JOINT_NAME "Shoulder joint"
#define ELBOW_JOINT_NAME "Elbow joint"
#define WRIST1_JOINT_NAME "Wrist1 joint"
#define WRIST2_JOINT_NAME "Wrist2 joint"
#define END_EFFECTOR_JOINT_NAME "End effector joint"

#define BASE_JOINT_VALUES D0, A0
#define SHOULDER_JOINT_VALUES D1, A1
#define ELBOW_JOINT_VALUES D2, A2
#define WRIST1_JOINT_VALUES D3, A3
#define WRIST2_JOINT_VALUES D4, A4
#define END_EFFECTOR_JOINT_VALUES D5, A5

#define BASE_JOINT_ORIENTATION M_PI
#define SHOULDER_JOINT_ORIENTATION M_PI_2
#define ELBOW_JOINT_ORIENTATION 0
#define WRIST1_JOINT_ORIENTATION M_PI_2
#define WRIST2_JOINT_ORIENTATION -M_PI_2
#define END_EFFECTOR_JOINT_ORIENTATION 0

#define BASE_JOINT_RANGE -M_2_PI,M_2_PI
#define SHOULDER_JOINT_RANGE -M_PI,M_PI
#define ELBOW_JOINT_RANGE -M_PI,M_PI
#define WRIST1_JOINT_RANGE -M_2_PI,M_2_PI
#define WRIST2_JOINT_RANGE -M_2_PI,M_2_PI
#define END_EFFECTOR_JOINT_RANGE -M_2_PI,M_2_PI

typedef Eigen::Vector3f vector3d;
typedef Eigen::Matrix3f rotationalMatrix;
typedef Eigen::Matrix4f transformationMatrix;
typedef Eigen::Vector4f vector4d;
typedef Eigen::Vector2f vector2d;

typedef struct end_effector_coordinates {
    vector3d orientation;
    vector3d position;
} end_effector_coordinates;

class UR5
{
private:
    revolute_joint *joint_list[6];
public:
    const unsigned short joints_number=JOINT_NUMBER;
    UR5();
    ~UR5();
    end_effector_coordinates compute_direct_kinematics(const VectorXd &jointAngles);
    
    friend ostream &operator<<(ostream &ostream, UR5 &manipulator);
};

ostream &operator<<(ostream &ostream, UR5 &manipulator);
#endif
