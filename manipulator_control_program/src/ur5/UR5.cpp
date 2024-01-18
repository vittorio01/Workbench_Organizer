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

trajectoryJointMatrix UR5::compute_trajectory(const trajectoryPointVector &endPosition, const jointVector &jointAngles,const int node_frequency) {
    trajectoryEndEffectorMatrix end_effector_trajectory;
    trajectoryEndEffectorMatrix end_effector_velocities;
    trajectoryJointMatrix jointTrajectory;

    trajectoryPointVector startPosition=get_end_effector_position(jointAngles);

    trajectoryPointVector error;
    pointVelocityVector velocity;
    trajectoryPointVector max_error;
    trajectoryPointVector a;
    trajectoryPointVector b;
    double step_width;
    double errorWeight;
    int steps;
    max_error << TRAJECTORY_MAX_ACCEPTED_ERROR;
    jointVector cumulativePos;

    bool loopEnable=true;
    int loopState=0; 
    double requiredTime;
    
    while (loopEnable && loopState < 4) {
        if (loopState==0) {
            requiredTime=TRAJECTORY_MIN_REQUIRED_TIME+(TRAJECTORY_TIME_PARAMETER*sqrt((startPosition-endPosition).norm()));
            steps=(int)(requiredTime*node_frequency);
            jointTrajectory.resize(6,steps);
            cumulativePos=jointAngles; 
        } else {
            startPosition=get_end_effector_position(jointTrajectory.col(jointTrajectory.cols()-1));
            requiredTime=TRAJECTORY_MIN_REQUIRED_TIME+(TRAJECTORY_TIME_PARAMETER*sqrt((startPosition-endPosition).norm()));
            if (requiredTime<TRAJECTORY_MIN_REQUIRED_TIME) requiredTime=TRAJECTORY_MIN_REQUIRED_TIME;
            steps=(int)(requiredTime*node_frequency);
            cumulativePos=jointTrajectory.col(jointTrajectory.cols()-1); 
            jointTrajectory.conservativeResize(Eigen::NoChange_t(),jointTrajectory.cols()+steps);
        }
        errorWeight=TRAJECTORY_ERROR_WEIGHT;
        step_width=requiredTime/((double)(steps));
        
        a=(endPosition-startPosition)*(-2/pow(requiredTime,3));
        b=(endPosition-startPosition)*(3/(pow(requiredTime,2)));

        end_effector_trajectory.resize(6,steps);
        end_effector_velocities.resize(6,steps);
        
        for (int i=0;i<steps;i++) {
            end_effector_trajectory.col(i)=(a*pow((i*step_width),3))+(b*pow((i*step_width),2))+startPosition;
            end_effector_velocities.col(i)=(a*(3*pow(i*step_width,2)))+(b*(2*(i*step_width)));
        }
        for (int i=0;i<steps;i++) {
            error=(end_effector_trajectory.col(i)-get_end_effector_position(cumulativePos))*errorWeight;
            cumulativePos=cumulativePos+(compute_joints_velocities(end_effector_velocities.col(i)+error,cumulativePos)*step_width);
            jointTrajectory.col(i+(jointTrajectory.cols()-steps))=cumulativePos;
        }
    
        error = end_effector_trajectory.col(end_effector_trajectory.cols()-1)-get_end_effector_position(jointTrajectory.col(jointTrajectory.cols()-1));
        loopEnable=false;
        for (int i=0;i<6;i++) {
            if (error(i)>max_error(i)) {
                loopEnable=true;
                break;
            }
        }
        loopState++;
        
    }
    if (jointTrajectory.cols()>1) {
        int collision;
        int firstCollisionIndex=-1;
        int lastCollisionIndex=-1;
        int borderCollisionIndex=-1;
        int borderCollisionPersistence;

        double borderCollisionJointValue;
        jointVector lastJointValues;

        bool collisionFound=false;
        bool collisionInProgress=false;
        bool direction;
        for (int i=0;i<jointTrajectory.cols();i++) {
            if (collisionInProgress) {
                collision=verifyEndEffectorCollision(jointTrajectory.col(i));
                if (collision==2 || collision==3) {
                    if (direction) {
                        lastJointValues=jointTrajectory.col(i);
                        while (collision==2 || collision==3) {
                            lastJointValues(4)=lastJointValues(4)+0.01;
                            collision=verifyEndEffectorCollision(lastJointValues);
                        }
                        if (lastJointValues(4)>borderCollisionJointValue) {
                            borderCollisionJointValue=lastJointValues(4);
                            borderCollisionIndex=i;
                            borderCollisionPersistence=1;
                        } else {
                            if (lastJointValues(4)==borderCollisionPersistence) borderCollisionPersistence++;
                        }
                    } else {
                        lastJointValues=jointTrajectory.col(i);
                        while (collision==2 || collision==3) {
                            lastJointValues(4)=lastJointValues(4)-0.01;
                            collision=collision=verifyEndEffectorCollision(lastJointValues);
                        }
                        if (lastJointValues(4)<borderCollisionJointValue) {
                            borderCollisionJointValue=lastJointValues(4);
                            borderCollisionIndex=i;
                            borderCollisionPersistence=1;
                        } else {
                            if (lastJointValues(4)==borderCollisionPersistence) borderCollisionPersistence++;
                        }
                    }
                } else {
                    lastCollisionIndex=i;
                    adjustJointTrajectory(jointTrajectory,4,node_frequency,firstCollisionIndex,borderCollisionIndex,lastCollisionIndex,borderCollisionPersistence,borderCollisionJointValue);
                    collisionInProgress=false;
                    collisionFound=false;
                }
            } else {
                collision=verifyEndEffectorCollision(jointTrajectory.col(i));
                if (collision==2 || collision==3) {
                    collisionFound=true;
                    collisionInProgress=true;
                    firstCollisionIndex=i;
                    if (i>0) {
                        direction=(jointTrajectory(3,i)-jointTrajectory(i-1))>0;
                    } else {
                        direction=(jointTrajectory(3,i+1)-jointTrajectory(i))>0;
                    }
                    borderCollisionJointValue=jointTrajectory(4,i);
                }
            }
        }
        if (collisionFound) {
            lastCollisionIndex=jointTrajectory.cols()-1;
            adjustJointTrajectory(jointTrajectory,4,node_frequency,firstCollisionIndex,borderCollisionIndex,lastCollisionIndex,borderCollisionPersistence,borderCollisionJointValue);
        }

        lastCollisionIndex=-1;
        collisionFound=false;
        collisionInProgress=false;

        
        for (int i=0;i<jointTrajectory.cols();i++) {
            if (collisionInProgress) {
                collision=verifyEndEffectorCollision(jointTrajectory.col(i));
                if (collision==1 || collision==3) {
                    lastJointValues=jointTrajectory.col(i);
                    while (collision==1 || collision==3) {
                        lastJointValues(2)=lastJointValues(2)+0.01;
                        collision=verifyEndEffectorCollision(lastJointValues);
                    }
                    if (lastJointValues(2)>borderCollisionJointValue) {
                        borderCollisionJointValue=lastJointValues(2);
                        borderCollisionIndex=i;
                        borderCollisionPersistence=1;
                    } else {
                        if (lastJointValues(2)==borderCollisionPersistence) borderCollisionPersistence++;
                    }
                } else {

                    lastCollisionIndex=i;
                    adjustJointTrajectory(jointTrajectory,2,node_frequency,firstCollisionIndex,borderCollisionIndex,lastCollisionIndex,borderCollisionPersistence,borderCollisionJointValue);
                    collisionInProgress=false;
                    collisionFound=false;
                }
            } else {
                collision=verifyEndEffectorCollision(jointTrajectory.col(i));
                if (collision==1 || collision==3) {
                    collisionFound=true;
                    collisionInProgress=true;
                    firstCollisionIndex=i;
                    borderCollisionJointValue=jointTrajectory(2,i);
                }
            }
        }
        if (collisionFound) {
            lastCollisionIndex=jointTrajectory.cols()-1;
            adjustJointTrajectory(jointTrajectory,2,node_frequency,firstCollisionIndex,borderCollisionIndex,lastCollisionIndex,borderCollisionPersistence,borderCollisionJointValue);
        }
    }
    return jointTrajectory;
}

void UR5::adjustJointTrajectory(trajectoryJointMatrix &jointTrajectory, int selectedJointIndex, int node_frequency, int firstIndex, int borderIndex, int lastIndex, int borderIndexPersistence, double borderCustomJointValue) {
    double firstJointValue=jointTrajectory(selectedJointIndex,firstIndex);
    double lastJointValue=jointTrajectory(selectedJointIndex,lastIndex);
    double dt=(double)((borderIndex-firstIndex)/((double)node_frequency));
    double aq=(borderCustomJointValue-firstJointValue)*(-2/pow(dt,3));
    double bq=(borderCustomJointValue-firstJointValue)*(3/pow(dt,2));
    double step_width=(double) (1/((double)(node_frequency)));
    for (int j=0;j<(borderIndex-firstIndex);j++) {
        jointTrajectory(selectedJointIndex,(firstIndex+j))=(aq*pow((j*step_width),3))+(bq*pow((j*step_width),2))+firstJointValue;
    }
    for (int j=0;j<borderIndexPersistence;j++) {
        jointTrajectory(selectedJointIndex,(borderIndex+j))=borderCustomJointValue;
    }
    borderIndex=borderIndex+borderIndexPersistence;
    dt=(double)((lastIndex-borderIndex)/((double)node_frequency));
    aq=(lastJointValue-borderCustomJointValue)*(-2/pow(dt,3));
    bq=(lastJointValue-borderCustomJointValue)*(3/pow(dt,2));
    for (int j=0;j<(lastIndex-borderIndex);j++) {
        jointTrajectory(selectedJointIndex,(borderIndex+j))=(aq*pow((j*step_width),3))+(bq*pow((j*step_width),2))+borderCustomJointValue;
    }
}

jointVelocityVector UR5::compute_joints_velocities(const pointVelocityVector &end_joint_velocity,const jointVector &currentJointAngles) {
    jacobianMatrix jacobian=compute_direct_differential_kinematics(currentJointAngles);
    jacobianMatrix jacobianTranspose=jacobian.transpose();
    double manipulability=sqrt((jacobian*jacobianTranspose).determinant());
    double k=0;
    if (manipulability<JACOBIAN_SINGULARITY_THRSHOLD) k=JACOBIAN_PRECISION_COEFFICIENT*pow((1-(manipulability/JACOBIAN_SINGULARITY_THRSHOLD)),2);
    return (jacobianTranspose*((jacobian*jacobianTranspose)+(pow(k,2)*jacobian.Identity())).inverse())*end_joint_velocity;  //jacobian.inverse()*end_joint_velocity;
}

transformationMatrix UR5::compute_direct_kinematics(const jointVector &jointAngles,const int end_joint_number=JOINT_NUMBER) {
    transformationMatrix matrix=getBaseTransformationMatrix();
    for (int i=0;i<end_joint_number;i++) {
        matrix=matrix*(joint_list[i]->get_transformation_matrix(jointAngles(i)));
    }
    return matrix;
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

jointVector UR5::get_joint_home_position() {
    jointVector home;
    home << UR5_JOINT_HOME_POSITION;
    return home;
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

int UR5::verifyEndEffectorCollision(const jointVector &jointAngles) {
    int collision=0;

    transformationMatrix temp=compute_direct_kinematics(jointAngles);
    pointVector endEffectorCenter(temp(0,3),temp(1,3),temp(2,3));
    
    transformationMatrix correctionMatrix;
    temp=compute_direct_kinematics(jointAngles,2);

    correctionMatrix=correctionMatrix.Identity();
    correctionMatrix(2,3)=ELBOW_JOINT_CORRECTION_MATRIX_VALUE;
    temp=temp*correctionMatrix;

    pointVector elbow_p1(temp(0,3),temp(1,3),temp(2,3));

    temp=compute_direct_kinematics(jointAngles,3);
    pointVector elbow2_p1(temp(0,3),temp(1,3),temp(2,3));
    temp=temp*correctionMatrix;
    pointVector elbow_p2(temp(0,3),temp(1,3),temp(2,3));
    
    temp=compute_direct_kinematics(jointAngles,4);
    correctionMatrix(2,3)=WRIST1_JOINT_CORRECTION_MATRIX_VALUE;
    temp=temp*correctionMatrix;
    pointVector elbow2_p2(temp(0,3),temp(1,3),temp(2,3));
    
    pointVector p12=elbow_p2-elbow_p1;
    pointVector p1e=endEffectorCenter-elbow_p1;
    pointVector p2e=endEffectorCenter-elbow_p2;
    double distance;

    if (p1e.dot(p12)>0 && p2e.dot(p12)<0) {
        distance =((p12.cross(p1e)).norm())/p12.norm();
        if (distance <= (END_EFFECTOR_COLLISION_AREA+ELBOW_JOINT_RADIUS)) {
            collision=1;
        }
    }

    p12=elbow2_p2-elbow2_p1;
    p1e=endEffectorCenter-elbow2_p1;
    p2e=endEffectorCenter-elbow2_p2;
    if (p1e.dot(p12)>0 && p2e.dot(p12)<0) {
        distance =((p12.cross(p1e)).norm())/p12.norm();
        if (distance <= (END_EFFECTOR_COLLISION_AREA+WRIST1_JOINT_RADIUS)) {
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

