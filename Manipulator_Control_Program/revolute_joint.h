#ifndef __revolute_joint_DATA__
#define __revolute_joint_DATA__

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <string>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef Eigen::Vector3f vector3d;
typedef Eigen::Matrix3f rotationalMatrix;
typedef Eigen::Matrix4f transformationMatrix;
typedef Eigen::Vector4f vector4d;
typedef Eigen::Vector2f vector2d;

class revolute_joint{
    private: 
        float joint_current_angle;
        const float joint_orientation;
        const float joint_link_a;
        const float joint_link_d;
        const float joint_min_angle;
        const float joint_max_angle;
        const bool joint_angle_limit;
    public:
        revolute_joint(const float _joint_orientation, const float _joint_link_d, const float _joint_link_a);
        revolute_joint(const float _joint_orientation, const float _joint_link_d, const float _joint_link_a,const float _joint_min_angle, const float _joint_max_angle);
        bool set_angle(const float new_angle);
        transformationMatrix get_transformation_matrix();
        float get_current_angle();
        float get_link_a();
        float get_link_d();
    friend ostream& operator << (ostream& ostream, revolute_joint& joint);
};

ostream& operator << (ostream& ostream, revolute_joint& joint) {
    ostream << "{current angle: "<< joint.joint_current_angle <<" (rad), ";
    if (joint.joint_angle_limit) {
        cout << "max angle: "<< joint.joint_max_angle << "(rad), min angle: "<< joint.joint_min_angle<<" (rad),";
    }
    ostream <<  "joint distances: [a =  "<<joint.joint_link_a<<" ,d = "<<joint.joint_link_d<< "]}";
    return ostream;
}

#endif 