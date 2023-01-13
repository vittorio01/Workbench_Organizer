#include "revolute_joint.h"

transformationMatrix revolute_joint::get_transformation_matrix() {
    transformationMatrix matrix;
    matrix(0,0)= cos(joint_current_angle);
    matrix(0,1)= -sin(joint_current_angle)*cos(joint_orientation);
    matrix(0,2)= sin(joint_current_angle)*sin(joint_orientation);
    matrix(1,0)= sin(joint_current_angle);
    matrix(1,1)= cos(joint_current_angle)*cos(joint_orientation);
    matrix(1,2)= -cos(joint_current_angle)*sin(joint_orientation);
    matrix(2,0)= 0;
    matrix(2,1)= sin(joint_orientation);
    matrix(2,2)= cos(joint_orientation);

    matrix(0,3)=joint_link_a;
    matrix(1,3)=-joint_link_d*sin(joint_orientation);
    matrix(2,3)=joint_link_d*cos(joint_orientation);

    matrix(3,0)=0;
    matrix(3,1)=0;
    matrix(3,2)=0;
    matrix(3,3)=1;
    return matrix;
}

revolute_joint::revolute_joint(const float _joint_orientation, const float _joint_link_d, const float _joint_link_a):joint_link_a(_joint_link_a),joint_link_d(_joint_link_d),joint_orientation(_joint_orientation),joint_angle_limit(false),joint_min_angle(0),joint_max_angle(0) {
    joint_current_angle=0;
}

revolute_joint::revolute_joint(const float _joint_orientation, const float _joint_link_d, const float _joint_link_a,const float _joint_min_angle, const float _joint_max_angle):joint_orientation(_joint_orientation),joint_link_a(_joint_link_a),joint_link_d(_joint_link_d),joint_angle_limit(true),joint_min_angle(_joint_min_angle),joint_max_angle(_joint_max_angle) {
    joint_current_angle=0;
}

bool revolute_joint::set_angle(const float new_angle) {
    if (joint_angle_limit && (new_angle<joint_min_angle || new_angle>joint_max_angle)) return false;
    joint_current_angle=new_angle;
    return true;
}

float revolute_joint::get_current_angle() {
    return joint_current_angle;
}

float revolute_joint::get_link_a() {return joint_link_a;}

float revolute_joint::get_link_d() {return joint_link_d;}


