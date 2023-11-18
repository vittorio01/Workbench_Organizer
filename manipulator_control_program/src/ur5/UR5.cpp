#include "UR5.h"

UR5::UR5() {
        joint_list[0] = new revolute_joint(BASE_JOINT_NAME,BASE_JOINT_ORIENTATION, BASE_JOINT_VALUES,BASE_JOINT_RANGE);
        joint_list[1] = new revolute_joint(SHOULDER_JOINT_NAME,SHOULDER_JOINT_ORIENTATION, SHOULDER_JOINT_VALUES,SHOULDER_JOINT_RANGE);
        joint_list[2] = new revolute_joint(ELBOW_JOINT_NAME,ELBOW_JOINT_ORIENTATION, ELBOW_JOINT_VALUES,ELBOW_JOINT_RANGE);
        joint_list[3] = new revolute_joint(WRIST1_JOINT_NAME,WRIST1_JOINT_ORIENTATION, WRIST1_JOINT_VALUES,WRIST1_JOINT_RANGE);
        joint_list[4] = new revolute_joint(WRIST2_JOINT_NAME,WRIST2_JOINT_ORIENTATION, WRIST2_JOINT_VALUES,WRIST2_JOINT_RANGE);
        joint_list[5] = new revolute_joint(END_EFFECTOR_JOINT_NAME,END_EFFECTOR_JOINT_ORIENTATION, END_EFFECTOR_JOINT_VALUES,END_EFFECTOR_JOINT_RANGE);
}

UR5::~UR5() {
    for (int i=0;i<JOINT_NUMBER;i++) {
        delete [] joint_list[i];
    }
}

end_effector_coordinates UR5::compute_direct_kinematics(const VectorXd &jointAngles) {
    transformationMatrix matrix;
    matrix << 1,0,0,0,
              0,1,0,0,
              0,0,1,0,
              0,0,0,1;
              
    for (int i=0;i<(JOINT_NUMBER);i++) {
        matrix=matrix*(joint_list[i]->get_transformation_matrix(jointAngles(i)));
    }
    vector4d base_position(0,0,0,1);
    base_position=matrix*base_position;
    
    vector4d pitch(1,0,0,1);
    pitch=matrix*pitch;

    vector4d roll(0,1,0,1);
    roll=matrix*roll;

    vector4d yaw(0,0,1,1);
    yaw=matrix*yaw;
    
    end_effector_coordinates coordinates;
    coordinates.position(0)=base_position(0);
    coordinates.position(1)=base_position(1);
    coordinates.position(2)=base_position(2);
    coordinates.orientation(0)=pitch(0);
    coordinates.orientation(1)=roll(1);
    coordinates.orientation(2)=yaw(2);
    return coordinates;
}



ostream &operator<<(ostream &ostream, UR5 &manipulator) {
    ostream << "Manipulator type: UR5" << endl;
    for (int i = 0; i < JOINT_NUMBER; i++)
    {
        ostream << "("<<i<<") "<<*(manipulator.joint_list[i])<<endl;
    }
    return ostream;
}