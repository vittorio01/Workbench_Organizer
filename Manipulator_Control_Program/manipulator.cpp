#include "manipulator.h"


manipulator_ur5::~manipulator_ur5() {
    for (int i=0;i<JOINT_NUMBER;i++) {
        delete joint_list[i];
    }
}

manipulator_ur5::manipulator_ur5(const vector3d _robot_base_position):robot_base_position(_robot_base_position)
    {
        joint_list[0] = new revolute_joint(BASE_JOINT_ORIENTATION, BASE_JOINT_VALUES);
        joint_list[1] = new revolute_joint(SHOULDER_JOINT_ORIENTATION, SHOULDER_JOINT_VALUES);
        joint_list[2] = new revolute_joint(ELBOW_JOINT_ORIENTATION, ELBOW_JOINT_VALUES);
        joint_list[3] = new revolute_joint(WRIST1_JOINT_ORIENTATION, WRIST1_JOINT_VALUES);
        joint_list[4] = new revolute_joint(WRIST2_JOINT_ORIENTATION, WRIST2_JOINT_VALUES);
        joint_list[5] = new revolute_joint(END_EFFECTOR_JOINT_ORIENTATION, END_EFFECTOR_JOINT_VALUES);
    }

vector3d manipulator_ur5::get_end_effector_position() {

    vector4d position(robot_base_position(0),robot_base_position(1),robot_base_position(2),1);
    transformationMatrix matrix=joint_list[0]->get_transformation_matrix();
    for (int i=1;i<JOINT_NUMBER;i++) {
        matrix=matrix*(joint_list[i]->get_transformation_matrix());
    }
    position=matrix*position;
    return vector3d(position(0),position(1),position(2));
}

vector2d manipulator_ur5::get_end_effector_angles() {
    return vector2d();
}

bool manipulator_ur5::set_end_effector_position(vector3d position, float angle1, float angle2) {
    
}