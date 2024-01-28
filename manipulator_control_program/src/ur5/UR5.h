#ifndef __MANIPULATOR_STRUCTURE__
#define __MANIPULATOR_STRUCTURE__

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "joints/revolute_joint.h"
#include "joints/revolute_joint.cpp"
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#define _USE_MATH_DEFINES

#define M_PI_8 (M_PI_4/2.0)

/*----- Direct kinematics definitions -----*/
//Joint number assignment (0 for first joint, 5 for last joint)
#define BASE_JOINT 0
#define SHOULDER_JOINT 1
#define ELBOW_JOINT 2
#define WRIST1_JOINT 3
#define WRIST2_JOINT 4
#define END_EFFECTOR_JOINT 5

#define JOINT_NUMBER 6

//Joint name assignment 
#define BASE_JOINT_NAME "Base joint"
#define SHOULDER_JOINT_NAME "Shoulder joint"
#define ELBOW_JOINT_NAME "Elbow joint"
#define WRIST1_JOINT_NAME "Wrist1 joint"
#define WRIST2_JOINT_NAME "Wrist2 joint"
#define END_EFFECTOR_JOINT_NAME "End effector joint"

//DH convention definitions for each joints
//Link lenghts
#define D0 0.163
#define A0 0

#define D1 0
#define A1 0

#define D2 0
#define A2 -0.42500

#define D3 0.134
#define A3 -0.39225

#define D4 0.100
#define A4 0

#define D5 0.100+0.115
#define A5 0

#define BASE_JOINT_VALUES D0, A0
#define SHOULDER_JOINT_VALUES D1, A1
#define ELBOW_JOINT_VALUES D2, A2
#define WRIST1_JOINT_VALUES D3, A3
#define WRIST2_JOINT_VALUES D4, A4
#define END_EFFECTOR_JOINT_VALUES D5, A5

//Link orientation

#define BASE_JOINT_ORIENTATION M_PI
#define SHOULDER_JOINT_ORIENTATION M_PI_2
#define ELBOW_JOINT_ORIENTATION 0
#define WRIST1_JOINT_ORIENTATION 0 
#define WRIST2_JOINT_ORIENTATION M_PI_2
#define END_EFFECTOR_JOINT_ORIENTATION -M_PI_2

//Joint angle ranges

#define BASE_JOINT_RANGE -M_2_PI,M_2_PI
#define SHOULDER_JOINT_RANGE -M_PI,M_PI
#define ELBOW_JOINT_RANGE -M_PI,M_PI
#define WRIST1_JOINT_RANGE -M_2_PI,M_2_PI
#define WRIST2_JOINT_RANGE -M_2_PI,M_2_PI
#define END_EFFECTOR_JOINT_RANGE -M_2_PI,M_2_PI

/*Collision avoidance parameters*/
#define ELBOW_JOINT_RADIUS 0.05     
#define WRIST1_JOINT_RADIUS 0.06   
#define END_EFFECTOR_COLLISION_AREA 0.1  
#define ELBOW_JOINT_CORRECTION_MATRIX_VALUE    0.135
#define WRIST1_JOINT_CORRECTION_MATRIX_VALUE   -0.135

/*Trajectory parameters definitions*/
#define TRAJECTORY_ERROR_WEIGHT         0.6
#define JACOBIAN_PRECISION_COEFFICIENT  0.32
#define JACOBIAN_SINGULARITY_THRSHOLD   0.005
#define TRAJECTORY_MAX_ACCEPTED_ERROR   0.001,0.001,0.001,0.001,0.001,0.001

#define TRAJECTORY_TIME_PARAMETER 2.5
#define TRAJECTORY_MIN_REQUIRED_TIME 0.2

/*Home position angle values*/
#define BASE_JOINT_HOME_POSITION            -1.57
#define SHOULDER_JOINT_HOME_POSITION        -0.785
#define ELBOW_JOINT_HOME_POSITION           -2.355
#define WRIST1_JOINT_HOME_POSITION          -1.56
#define WRIST_2_JOINT_HOME_POSITION         -1.56
#define END_EFFECTOR_JOINT_HOME_POSITION    1.56

#define UR5_JOINT_HOME_POSITION BASE_JOINT_HOME_POSITION,SHOULDER_JOINT_HOME_POSITION,ELBOW_JOINT_HOME_POSITION,WRIST1_JOINT_HOME_POSITION,WRIST_2_JOINT_HOME_POSITION,END_EFFECTOR_JOINT_HOME_POSITION

typedef Eigen::Matrix<double,3,3> rotationalMatrix;
typedef Eigen::Matrix<double,4,4> transformationMatrix;
typedef Eigen::Matrix<double,6,6> jacobianMatrix;
typedef Eigen::Matrix<double,6,1> jointVelocityVector;
typedef Eigen::Matrix<double,6,1> pointVelocityVector;
typedef Eigen::Matrix<double,3,1> pointVector;
typedef Eigen::Matrix<double,JOINT_NUMBER,1> jointVector; 

typedef Eigen::Matrix<double,6,Eigen::Dynamic> trajectoryJointMatrix;
typedef Eigen::Matrix<double,6,Eigen::Dynamic> trajectoryEndEffectorMatrix;
typedef Eigen::Matrix<double,6,1> trajectoryPointVector;

class UR5
{
private:
    revolute_joint *joint_list[6];
    const unsigned short joints_number=JOINT_NUMBER;
    const pointVector basePosition;
    const Eigen::Matrix<double,3,1> baseOrientation;

    int verifyEndEffectorCollision(const jointVector &jointAngles);
    void adjustJointTrajectory(trajectoryJointMatrix &jointTrajectory, int selectedJointIndex, int node_frequency, int firstIndex, int borderIndex, int lastIndex, int borderIndexPersistence, double borderCustomJointValue);
    
    transformationMatrix compute_direct_kinematics(const jointVector &jointAngles,const int end_joint_number);
    jacobianMatrix compute_direct_differential_kinematics(const jointVector &jointAngles);
    jointVelocityVector compute_joints_velocities(const pointVelocityVector &end_joint_velocity,const jointVector &currentJointAngles);
    transformationMatrix getBaseTransformationMatrix();
    
public:
    

    UR5(const pointVector __basePosition,const Eigen::Matrix<double,3,1> __baseOrientation);
    ~UR5();

    trajectoryJointMatrix compute_trajectory(const trajectoryPointVector &endPosition, const jointVector &jointAngles,const int node_frequency);
    trajectoryPointVector get_end_effector_position(const jointVector &jointAngles);
    jointVector get_joint_home_position();
    
    void print_direct_transform (const jointVector &jointAngles);
    friend ostream &operator<<(ostream &ostream, UR5 &manipulator);
};

ostream &operator<<(ostream &ostream, UR5 &manipulator);
#endif
