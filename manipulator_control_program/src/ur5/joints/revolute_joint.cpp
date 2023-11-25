#include "revolute_joint.h"

transformationMatrix revolute_joint::get_transformation_matrix(const double JointAngle) {
    
    transformationMatrix r1;
    r1(0,0)=1;
    r1(0,1)=0;
    r1(0,2)=0;
    r1(0,3)=joint_link_a;

    r1(1,0)=0;
    r1(1,1)=cos(joint_orientation);
    r1(1,2)=-sin(joint_orientation);
    r1(1,3)=0;

    r1(2,0)=0;
    r1(2,1)=sin(joint_orientation);
    r1(2,2)=cos(joint_orientation);
    r1(2,3)=0;

    r1(3,0)=0;
    r1(3,1)=0;
    r1(3,2)=0;
    r1(3,3)=1;    

    transformationMatrix r2;
    r2(0,0)=cos(JointAngle);
    r2(0,1)=-sin(JointAngle);
    r2(0,2)=0;
    r2(0,3)=0;

    r2(1,0)=sin(JointAngle);
    r2(1,1)=cos(JointAngle);
    r2(1,2)=0;
    r2(1,3)=0;

    r2(2,0)=0;
    r2(2,1)=0;
    r2(2,2)=1;
    r2(2,3)=joint_link_d;

    r2(3,0)=0;
    r2(3,1)=0;
    r2(3,2)=0;
    r2(3,3)=1;
    
    /*
    transformationMatrix r1;
    r1(0,0)=cos(JointAngle);
    r1(0,1)=-sin(JointAngle);
    r1(0,2)=0;
    r1(0,3)=joint_link_a;

    r1(1,0)=cos(joint_orientation)*sin(JointAngle);
    r1(1,1)=cos(joint_orientation)*cos(JointAngle);
    r1(1,2)=-sin(joint_orientation);
    r1(1,3)=0;

    r1(2,0)=sin(JointAngle)*sin(joint_orientation);
    r1(2,1)=sin(joint_orientation)*cos(JointAngle);
    r1(2,2)=cos(joint_orientation);
    r1(2,3)=joint_link_d;

    r1(3,0)=0;
    r1(3,1)=0;
    r1(3,2)=0;
    r1(3,3)=1;    
    */

    return r1*r2;
}

revolute_joint::revolute_joint(const string _name, const double _joint_orientation, const double _joint_link_d, const double _joint_link_a,const double _joint_min_angle, const double _joint_max_angle):name(_name),joint_orientation(_joint_orientation),joint_link_a(_joint_link_a),joint_link_d(_joint_link_d),joint_min_angle(_joint_min_angle),joint_max_angle(_joint_max_angle) {
}

double revolute_joint::get_link_a() {
    return joint_link_a;
}
double revolute_joint::get_link_d() {
    return joint_link_d;
}
double revolute_joint::get_max_angle() {
    return joint_max_angle;
}
double revolute_joint::get_min_angle() {
    return joint_min_angle;
}
string revolute_joint::get_name() {
    return name;
}
double revolute_joint::get_orientation() {
    return joint_orientation;
}