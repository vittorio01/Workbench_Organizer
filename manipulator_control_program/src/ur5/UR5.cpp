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
    vector4d base_position(0,0,0,1);

    for (int i=0;i<(JOINT_NUMBER);i++) {
        matrix=matrix*(joint_list[i]->get_transformation_matrix(jointAngles(i)));
    }
    base_position=matrix*base_position;

    end_effector_coordinates coordinates;
    coordinates.position(0)=base_position(0);
    coordinates.position(1)=base_position(1);
    coordinates.position(2)=base_position(2);
    
    if (matrix(3,1)==1 || matrix(3,1)==-1) {
        coordinates.orientation1(0)=0;
        if (matrix(3,1)==-1) {
            coordinates.orientation1(1)=M_PI_2;
            coordinates.orientation1(2)=atan2(matrix(1,2),matrix(1,3));
        } else {
            coordinates.orientation1(1)=-M_PI_2;
            coordinates.orientation1(2)=atan2(matrix(1,2),matrix(1,3));
        }
        coordinates.orientation2(0)=coordinates.orientation1(0);
        coordinates.orientation2(1)=coordinates.orientation1(1);
        coordinates.orientation2(2)=coordinates.orientation1(2);
    } else {
        coordinates.orientation1(0)=atan2(matrix(2,1),matrix(1,1));
        coordinates.orientation1(1)=atan2(-matrix(3,1),sqrt((matrix(3,2)*matrix(3,2))+(matrix(3,3)*matrix(3,3))));
        coordinates.orientation1(2)=atan2(matrix(3,2),matrix(3,3));

        coordinates.orientation2(0)=atan2(-matrix(2,1),-matrix(1,1));
        coordinates.orientation2(1)=atan2(-matrix(3,1),-sqrt((matrix(3,2)*matrix(3,2))+(matrix(3,3)*matrix(3,3))));
        coordinates.orientation2(2)=atan2(-matrix(3,2),-matrix(3,3));
    }
    

    return coordinates;
}

void UR5::print_direct_transform(const VectorXd &jointAngles) {
    transformationMatrix currentTransform;
    vector4d currentPosition;
    for (int i=0;i<(JOINT_NUMBER);i++) {
        currentTransform << 1,0,0,0,
                            0,1,0,0,
                            0,0,1,0,
                            0,0,0,1;
        currentPosition << 0,0,0,1;
        for (int j=0;j<=i;j++) {
            currentTransform=currentTransform*(joint_list[j]->get_transformation_matrix(jointAngles(j)));
        }
        currentPosition=currentTransform*currentPosition;
        cout << joint_list[i]->get_name()<<": "<<currentPosition(0)<<", "<<currentPosition(1)<<", "<<currentPosition(2)<<endl;
    }
}

ostream &operator<<(ostream &ostream, UR5 &manipulator) {
    ostream << "Manipulator type: UR5" << endl;
    for (int i = 0; i < JOINT_NUMBER; i++)
    {
        ostream << "("<<i<<") "<<*(manipulator.joint_list[i])<<endl;
    }
    return ostream;
}

