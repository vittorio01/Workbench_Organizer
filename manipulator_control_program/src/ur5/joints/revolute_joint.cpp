#include "revolute_joint.h"

transformationMatrix revolute_joint::get_transformation_matrix(const double JointAngle) {
    transformationMatrix t;
    t <<    cos(JointAngle), (-sin(JointAngle)), 0, joint_link_a,
            (cos(joint_orientation)*sin(JointAngle)), (cos(joint_orientation)*cos(JointAngle)), (-sin(joint_orientation)), (-joint_link_d*sin(joint_orientation)),
            (sin(joint_orientation)*sin(JointAngle)),(sin(joint_orientation)*cos(JointAngle)),cos(joint_orientation),(joint_link_d*cos(joint_orientation)),
            0,0,0,1;

    return t;
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