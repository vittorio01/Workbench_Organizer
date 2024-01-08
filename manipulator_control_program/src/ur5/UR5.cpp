#include "UR5.h"

UR5::UR5(const pointVector __basePosition,const Eigen::Matrix<double,3,1> __baseOrientation):basePosition(__basePosition),baseOrientation(__baseOrientation) {
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

transformationMatrix UR5::getBaseTransformationMatrix() {
    transformationMatrix matrix;
    double psi=baseOrientation(0);
    double theta=baseOrientation(1);
    double phi=baseOrientation(2);

    matrix <<   (cos(phi)*cos(theta)),((cos(phi)*sin(theta)*sin(psi))-(sin(phi)*cos(psi))),((cos(phi)*sin(theta)*cos(psi))+(sin(phi)*sin(psi))),basePosition(0),
                (sin(phi)*cos(theta)),((sin(phi)*sin(theta)*sin(psi))+(cos(phi)*cos(psi))),((sin(phi)*sin(theta)*cos(psi))-(cos(phi)*sin(psi))),basePosition(1),
                (-sin(theta)),(cos(theta)*sin(psi)),(cos(theta)*cos(psi)),basePosition(2),
                0,0,0,1;  
    return matrix;
}

transformationMatrix UR5::compute_direct_kinematics(const jointVector &jointAngles,const int end_joint_number=JOINT_NUMBER) {
    transformationMatrix matrix=getBaseTransformationMatrix();
    for (int i=0;i<end_joint_number;i++) {
        matrix=matrix*(joint_list[i]->get_transformation_matrix(jointAngles(i)));
    }
    return matrix;
}

trajectoryPointVector UR5::get_end_effector_position(const jointVector &jointAngles) {
    trajectoryPointVector position;
    transformationMatrix directTransformation=compute_direct_kinematics(jointAngles);
    position(0)=directTransformation(0,3);
    position(1)=directTransformation(1,3);
    position(2)=directTransformation(2,3);
    if (directTransformation(2,0)!= 1 && directTransformation(2,0)!=-1) {
        position(4)=-asin(directTransformation(2,0));
        position(3)=atan2((directTransformation(2,1)/(cos(position(4)))),((directTransformation(2,2)/(cos(position(4))))));
        position(5)=atan2((directTransformation(1,0)/(cos(position(4)))),((directTransformation(0,0)/(cos(position(4))))));
    } else {
        position(5)=0;
        position(3)=atan2(directTransformation(0,1),directTransformation(0,2));
        if (directTransformation(2,0)==-1) {
           position(4)=M_PI_2;
        } else {
           position(4)=-M_PI_2;
        }
    }


    return position;

}

trajectoryJointMatrix UR5::compute_trajectory(const trajectoryPointVector &endPosition, const jointVector &jointAngles,const double requiredTime, const int node_frequency, const double endEffectorCoveredRadius) {
    trajectoryEndEffectorMatrix end_effector_trajectory;
    trajectoryEndEffectorMatrix end_effector_velocities;
    trajectoryPointVector startPosition=get_end_effector_position(jointAngles);
    trajectoryJointMatrix jointTrajectory;
    if (true) {
        double homeRequiredTime=requiredTime/3;
        int homeSteps=(int)(homeRequiredTime*node_frequency);

        double destinationRequiredTime=requiredTime/3;
        int destinationSteps=(int)(destinationRequiredTime*node_frequency);

        double shoulderRequiredTime=requiredTime/3;
        int shoulderSteps=(int)(destinationRequiredTime*node_frequency);

        //home procedure
        jointVector homeJointValues;
        homeJointValues << UR5_JOINT_HOME_POSITION;
        trajectoryPointVector homePosition=get_end_effector_position(homeJointValues);

        double errorWeight=TRAJECTORY_ERROR_WEIGHT;

        double step_width=homeRequiredTime/((double)(homeSteps));
        
        trajectoryPointVector a=(homePosition-startPosition)*(-2/pow(homeRequiredTime,3));
        trajectoryPointVector b=(homePosition-startPosition)*(3/(pow(homeRequiredTime,2)));
        
        
        jointTrajectory.resize(6,homeSteps+destinationSteps+shoulderSteps);
        end_effector_trajectory.resize(6,homeSteps);
        end_effector_velocities.resize(6,homeSteps);
        for (int i=0;i<homeSteps;i++) {
            end_effector_trajectory.col(i)=(a*pow((i*step_width),3))+(b*pow((i*step_width),2))+startPosition;
            end_effector_velocities.col(i)=(a*(3*pow(i*step_width,2)))+(b*(2*(i*step_width)));
        }
        
        jointVector cumulativePos=jointAngles; 
        trajectoryPointVector error;
        pointVelocityVector velocity;
        for (int i=0;i<homeSteps;i++) {
            error=(end_effector_trajectory.col(i)-get_end_effector_position(cumulativePos))*errorWeight;
            cumulativePos=cumulativePos+(compute_joints_velocities(end_effector_velocities.col(i),cumulativePos)*step_width);
            jointTrajectory.col(i)=cumulativePos;
        }

        //base joint rotation

        jointVector jointStartPosition=jointTrajectory.col(homeSteps-1);
        jointVector jointEndPosition=jointStartPosition;
        jointEndPosition(0)=-BASE_JOINT_HOME_POSITION;
        jointVector aj=(jointEndPosition-jointStartPosition)*(-2/pow(shoulderRequiredTime,3));
        jointVector bj=(jointEndPosition-jointStartPosition)*(3/(pow(shoulderRequiredTime,2)));
        
        for (int i=homeSteps;i<(homeSteps+shoulderSteps);i++) {
            jointTrajectory.col(i)=(aj*pow(((i-homeSteps)*step_width),3))+(bj*pow(((i-homeSteps)*step_width),2))+jointStartPosition;
        }

        //destination procedure
    
        startPosition=get_end_effector_position(jointTrajectory.col((homeSteps+shoulderSteps)-1));
        errorWeight=TRAJECTORY_ERROR_WEIGHT;

        step_width=destinationRequiredTime/((double)(destinationSteps));
        
        a=(endPosition-startPosition)*(-2/pow(destinationRequiredTime,3));
        b=(endPosition-startPosition)*(3/(pow(destinationRequiredTime,2)));
        
        end_effector_trajectory.resize(6,destinationSteps);
        end_effector_velocities.resize(6,destinationSteps);
        
        for (int i=0;i<destinationSteps;i++) {
            end_effector_trajectory.col(i)=(a*pow((i*step_width),3))+(b*pow((i*step_width),2))+startPosition;
            end_effector_velocities.col(i)=(a*(3*pow(i*step_width,2)))+(b*(2*(i*step_width)));
        }
        
        cumulativePos=jointTrajectory.col(homeSteps+shoulderSteps-1); 
        for (int i=(homeSteps+shoulderSteps);i<(homeSteps+shoulderSteps+destinationSteps);i++) {
            error=(end_effector_trajectory.col(i-((homeSteps+shoulderSteps))))-get_end_effector_position(cumulativePos)*errorWeight;
            cumulativePos=cumulativePos+(compute_joints_velocities(end_effector_velocities.col(i-(homeSteps+shoulderSteps)),cumulativePos)*step_width);
            jointTrajectory.col(i)=cumulativePos;
        }
        


    } else {
        int steps=(int)(requiredTime*node_frequency);
        double errorWeight=TRAJECTORY_ERROR_WEIGHT;

        double step_width=requiredTime/((double)(steps));
        
        trajectoryPointVector a=(endPosition-startPosition)*(-2/pow(requiredTime,3));
        trajectoryPointVector b=(endPosition-startPosition)*(3/(pow(requiredTime,2)));
        
        trajectoryJointMatrix jointTrajectory;
        jointTrajectory.resize(6,steps);
        end_effector_trajectory.resize(6,steps);
        end_effector_velocities.resize(6,steps);
        for (int i=0;i<steps;i++) {
            end_effector_trajectory.col(i)=(a*pow((i*step_width),3))+(b*pow((i*step_width),2))+startPosition;
            end_effector_velocities.col(i)=(a*(3*pow(i*step_width,2)))+(b*(2*(i*step_width)));
        }

        jointVector cumulativePos=jointAngles; 
        trajectoryPointVector error;
        pointVelocityVector velocity;
        for (int i=0;i<steps;i++) {
            error=(end_effector_trajectory.col(i)-get_end_effector_position(cumulativePos))*errorWeight;
            cumulativePos=cumulativePos+(compute_joints_velocities(end_effector_velocities.col(i),cumulativePos)*step_width);
            jointTrajectory.col(i)=cumulativePos;
        }
    }
    
    int collision;
    for (int i=0;i<jointTrajectory.cols();i++) {
        collision=verifyEndEffectorCollision(jointTrajectory.col(i),endEffectorCoveredRadius);
        if (collision==1 || collision==3) {
            while (collision==1 || collision==3) {
                jointTrajectory(3,i)=jointTrajectory(3,i)-0.01;
                collision=verifyEndEffectorCollision(jointTrajectory.col(i),endEffectorCoveredRadius);
            } 
        }
    }
    return jointTrajectory;
}

jointVelocityVector UR5::compute_joints_velocities(const pointVelocityVector &end_joint_velocity,const jointVector &currentJointAngles) {
    jacobianMatrix jacobian=compute_direct_differential_kinematics(currentJointAngles);
    jointVelocityVector v=jacobian.inverse()*end_joint_velocity;
    return v;
}

jointVector UR5::get_joint_home_position() {
    jointVector home;
    home << UR5_JOINT_HOME_POSITION;
    return home;
}

jacobianMatrix UR5::compute_direct_differential_kinematics(const jointVector &jointAngles) {
    jacobianMatrix jacobian;
    transformationMatrix completeDirectKinematics=compute_direct_kinematics(jointAngles);

    transformationMatrix partialDirectKinematics;
    Eigen::Matrix<double,3,1> rotCol;
    pointVector diffKinematics;
    for (int i=0;i<JOINT_NUMBER;i++) {
        partialDirectKinematics=compute_direct_kinematics(jointAngles,(i+1));
        
        jacobian(3,i)=partialDirectKinematics(0,2);
        jacobian(4,i)=partialDirectKinematics(1,2);
        jacobian(5,i)=partialDirectKinematics(2,2);

        rotCol(0)=partialDirectKinematics(0,2);
        rotCol(1)=partialDirectKinematics(1,2);
        rotCol(2)=partialDirectKinematics(2,2);
        diffKinematics(0)=completeDirectKinematics(0,3)-partialDirectKinematics(0,3);
        diffKinematics(1)=completeDirectKinematics(1,3)-partialDirectKinematics(1,3);
        diffKinematics(2)=completeDirectKinematics(2,3)-partialDirectKinematics(2,3);
        rotCol=rotCol.cross(diffKinematics);
        jacobian(0,i)=rotCol(0);
        jacobian(1,i)=rotCol(1);
        jacobian(2,i)=rotCol(2);
    }
    jacobianMatrix angleTransformationMatrix;
    trajectoryPointVector end_effector_position=get_end_effector_position(jointAngles);
    double psi=end_effector_position(3);
    double theta=end_effector_position(4);
    double phi=end_effector_position(5);
    angleTransformationMatrix << 
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0,
        0,0,0,(cos(theta)*cos(phi)),(-sin(phi)),0,
        0,0,0,(cos(theta)*sin(phi)),cos(phi),0,
        0,0,0,(-sin(theta)),0,1;
    return angleTransformationMatrix.inverse()*jacobian;
}

void UR5::print_direct_transform(const jointVector &jointAngles) {
    transformationMatrix currentTransform;
    Eigen::Matrix<double,4,1> currentPosition;
    for (int i=0;i<(JOINT_NUMBER);i++) {
        currentTransform=compute_direct_kinematics(jointAngles,i);
        currentPosition << 0,0,0,1;
        currentPosition=currentTransform*currentPosition;
        cout << joint_list[i]->get_name()<<": "<<currentPosition(0)<<", "<<currentPosition(1)<<", "<<currentPosition(2)<<endl;
    }
}

int UR5::verifyEndEffectorCollision(const jointVector &jointAngles,const double end_effector_sphere_radius) {
    int collision=0;

    transformationMatrix temp=compute_direct_kinematics(jointAngles);
    pointVector endEffectorCenter(temp(0,3),temp(1,3),temp(2,3));

    temp=compute_direct_kinematics(jointAngles,3);
    pointVector elbow_p1(temp(0,3),temp(1,3),temp(2,3));
    temp=compute_direct_kinematics(jointAngles,4);
    pointVector elbow_p2(temp(0,3),temp(1,3),temp(2,3));

    pointVector elbow2_p1(temp(0,3),temp(1,3),temp(2,3));
    temp=compute_direct_kinematics(jointAngles,5);
    pointVector elbow2_p2(temp(0,3),temp(1,3),temp(2,3));
    
    pointVector p12=elbow_p2-elbow_p1;
    pointVector p1e=endEffectorCenter-elbow_p1;
    pointVector p2e=endEffectorCenter-elbow_p2;
    double distance;

    if (p1e.dot(p12)>0 && p2e.dot(p12)<0) {
        distance =((p12.cross(p1e)).norm())/p12.norm();
        if (distance <= (end_effector_sphere_radius+ELBOW_JOINT_RADIUS)) {
            collision=1;
        }
    }

    p12=elbow2_p2-elbow2_p1;
    p1e=endEffectorCenter-elbow2_p1;
    p2e=endEffectorCenter-elbow2_p2;

    if (p1e.dot(p12)>0 && p2e.dot(p12)<0) {
        distance =((p12.cross(p1e)).norm())/p12.norm();
        if (distance <= (end_effector_sphere_radius+WRIST1_JOINT_RADIUS)) {
            if (collision==1) {
                collision=3;
            } else {
                collision=2;
            }
        }
    }

    return collision;
}

ostream &operator<<(ostream &ostream, UR5 &manipulator) {
    ostream << "Manipulator type: UR5" << endl;
    for (int i = 0; i < JOINT_NUMBER; i++)
    {
        ostream << "("<<i<<") "<<*(manipulator.joint_list[i])<<endl;
    }
    return ostream;
}

