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

typedef Eigen::Matrix<double,3,3> rotationalMatrix;
typedef Eigen::Matrix<double,4,4> transformationMatrix;
typedef Eigen::Matrix<double,3,1> pointVector;

class revolute_joint{
    private: 
        const string name;
        const double joint_orientation;
        const double joint_link_a;
        const double joint_link_d;
        const double joint_min_angle;
        const double joint_max_angle;
    public:
        revolute_joint(const string _name,const double _joint_orientation, const double _joint_link_d, const double _joint_link_a,const double _joint_min_angle, const double _joint_max_angle);
        transformationMatrix get_transformation_matrix(const double jointAngle);
        string get_name();
        double get_link_a();
        double get_link_d();
        double get_max_angle();
        double get_min_angle();
        double get_orientation();

    friend ostream& operator << (ostream& ostream, revolute_joint& joint);
};

ostream& operator << (ostream& ostream, revolute_joint& joint) {
    cout << joint.name << ": orientation= "<< joint.joint_orientation<< ", "<< joint.joint_link_a<< ", "<< joint.joint_link_d<< "; angle_range= "<< joint.joint_min_angle<< " to "<< joint.joint_max_angle;
    return ostream;
}

#endif 