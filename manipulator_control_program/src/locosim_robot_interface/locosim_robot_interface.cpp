#include "locosim_robot_interface.h"

void Locosim_robot_interface::initialize(ros::NodeHandle &node) {
    subscriber=node.subscribe("/ur5/joint_states",BUFFER_LENGHT,messageReadHandler);
    sender = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", BUFFER_LENGHT);
    for (int i=0;i<GRIPPER_NUMBER;i++) {
        lastGripperValues[i]=0;
    }
}

bool Locosim_robot_interface::getSystemStatus() {
    return online;
}


void Locosim_robot_interface::messageReadHandler(const sensor_msgs::JointState &msg) {
    if (online) {
        for (int i=0;i<JOINTS_NUMBER;i++) {
            for (int j=0;j<JOINTS_NUMBER;j++) {
                if (msg.name[i].compare(names[j])==0) {
                    positions(j)=msg.position[i];
                    velocities(j)=msg.velocity[i];
                    efforts(j)=msg.effort[i];
                }
            }
        }
    } else {
        for (int i=0;i<JOINTS_NUMBER;i++) {
            if (msg.position[i]!=0) {
                online=true;
                break;
            }
        }
        if (online) {
            for (int i=0;i<JOINTS_NUMBER;i++) {
                for (int j=0;j<JOINTS_NUMBER;j++) {
                    if (msg.name[i].compare(names[j])==0) {
                        positions(j)=msg.position[i];
                        velocities(j)=msg.velocity[i];
                        efforts(j)=msg.effort[i];
                    }
                }
            }
        }
    }
}

JointVector Locosim_robot_interface::getPositions() {
    JointVector values;
    for (int i=0;i<JOINTS_NUMBER;i++) {
        values(i)=positions(i);
    }
    return values;
}

void Locosim_robot_interface::setPosition(const JointVector& targetPosition) {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(JOINT_NUMBER+GRIPPER_NUMBER);
    for (int i=0;i<JOINT_NUMBER;i++) {
        msg.data[i]=targetPosition[i];
        lastJointPosition[i]=targetPosition[i];
    }
    for (int i=0;i<GRIPPER_NUMBER;i++) {
        msg.data[JOINT_NUMBER+i]=lastGripperValues[i];
    }
    sender.publish(msg);
}

void Locosim_robot_interface::setGripperValue(const double gripperValues[]) {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(JOINT_NUMBER+GRIPPER_NUMBER);
    for (int i=0;i<JOINT_NUMBER;i++) {
        msg.data[i]=lastJointPosition[i];
    }
    for (int i=0;i<GRIPPER_NUMBER;i++) {
        msg.data[JOINT_NUMBER+i]=gripperValues[i];
        lastGripperValues[i]=gripperValues[i];
    }
    sender.publish(msg);
}

ostream& operator<<(ostream& os,const Locosim_robot_interface& interface) {
    for (int i=0;i<(JOINTS_NUMBER);i++) {
        os << interface.names[i] << ":" << interface.positions(i) << endl;
    }
    return os;
}