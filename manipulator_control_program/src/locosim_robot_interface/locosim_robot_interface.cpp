#include "locosim_robot_interface.h"

void Locosim_robot_interface::initialize(ros::NodeHandle &node) {
    subscriber=node.subscribe("/ur5/joint_states",BUFFER_LENGHT,messageReadHandler);
}

void Locosim_robot_interface::messageReadHandler(const sensor_msgs::JointState &msg) {
    for (int i=0;i<JOINTS_NUMBER;i++) {
        names[i]=msg.name[i];
        positions[i]=msg.position[i];
        velocities[i]=msg.velocity[i];
        efforts[i]=msg.effort[i];
    }
}

float Locosim_robot_interface::getVelocity(const int jointNumber) {
    if (jointNumber<JOINTS_NUMBER) {
        return velocities[jointNumber];
    }
    return 0;
}

string Locosim_robot_interface::getName(const int jointNumber) {
    if (jointNumber<JOINTS_NUMBER) {
        return names[jointNumber];
    }
    return 0;
}
float Locosim_robot_interface::getEffort(const int jointNumber) {
    if (jointNumber<JOINTS_NUMBER) {
        return efforts[jointNumber];
    }
    return 0;
}
float Locosim_robot_interface::getPosition(const int jointNumber) {
    if (jointNumber<JOINTS_NUMBER) {
        return positions[jointNumber];
    }
    return 0.;
}