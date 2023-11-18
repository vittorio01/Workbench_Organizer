#include "locosim_robot_interface.h"

void Locosim_robot_interface::initialize(ros::NodeHandle &node) {
    subscriber=node.subscribe("/ur5/joint_states",BUFFER_LENGHT,messageReadHandler);
    sender = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", BUFFER_LENGHT);
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
    msg.data.resize(6);
    for (int i=0;i<JOINT_NUMBER;i++) {
        msg.data[i]=targetPosition[i];
    }
    /*
    msg.data[SHOULDER_PAN_MSG_INDEX]=targetPosition(0);
    msg.data[SHOULDER_LIFT_MSG_INDEX]=targetPosition(1);
    msg.data[ELBOW_MSG_INDEX ]=targetPosition(2);
    msg.data[WRIST_1_MSG_INDEX]=targetPosition(3);
    msg.data[WRIST_2_MSG_INDEX]=targetPosition(4);
    msg.data[WRIST_3_MSG_INDEX]=targetPosition(5);
    */
    sender.publish(msg);
}

ostream& operator<<(ostream& os,const Locosim_robot_interface& interface) {
    for (int i=0;i<(JOINTS_NUMBER);i++) {
        os << interface.names[i] << ":" << interface.positions(i) << endl;
    }
    return os;
}