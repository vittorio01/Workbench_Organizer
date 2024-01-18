#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>

#include "ur5/UR5.h"
#include "ur5/UR5.cpp"
#include "ur5/gripper/soft_gripper.h"
#include "ur5/gripper/soft_gripper.cpp"
#include "locosim_robot_interface/locosim_robot_interface.h"
#include "locosim_robot_interface/locosim_robot_interface.cpp"

#include "manipulator_control_program/mcp_command.h"
#include "manipulator_control_program/mcp_status.h"

#define NODE_FREQUENCY 200

#define UR5_SPAWN_POSITION      0.5,0.35,1.8
#define UR5_SPAWN_ORIENTATION   0,0,0

using namespace std;

typedef enum{initializing,idle,moving,planning,gripper_adjusting,homing_planning} stateType;
typedef enum{none,ef_move,gripper_adjust,homing_procedure} commandType;

stateType internalState;
commandType assignedCommand;
trajectoryPointVector destinationPose;
double gripperTargetValue;
trajectoryJointMatrix jointMotionMatrix;

void commandHandler (const manipulator_control_program::mcp_command &message) {
    if (message.command.compare("move")==0 && internalState==idle) {
        assignedCommand=ef_move;
        destinationPose(0)=message.x;
        destinationPose(1)=message.y;
        destinationPose(2)=message.z;
        destinationPose(3)=message.phi;
        destinationPose(4)=message.theta;
        destinationPose(5)=message.psi;
        return;
    } 
    if (message.command.compare("gripper_adjust")==0 && internalState==idle) {
        assignedCommand=gripper_adjust;
        gripperTargetValue=message.gripperDistance;
        return;
    }
    if (message.command.compare("homing_procedure")==0 && internalState==idle) {
        assignedCommand=homing_procedure;
        return;
    }
}

int main(int argc, char** argv) {
    ROS_INFO("started manipulator_control_program ROS node" );
    ros::init(argc,argv,"data_listener");
    ros::NodeHandle node;
    
    Locosim_robot_interface interface;
    interface.initialize(node);
    
    
    internalState=initializing;
    assignedCommand=none;
    UR5 manipulator(pointVector(UR5_SPAWN_POSITION),Eigen::Matrix<double,3,1>(UR5_SPAWN_ORIENTATION));
    
    ros::Publisher mcpStatusPublisher = node.advertise<manipulator_control_program::mcp_status>("mcp_status", 10);
    ros::Subscriber mcpCommandSubscriber = node.subscribe("mcp_command",10,commandHandler);

    manipulator_control_program::mcp_status msg;
    trajectoryPointVector end_effector_position;
    ros::Rate loop_rate(NODE_FREQUENCY);

    int currentTrajectoryIndex;

    while (ros::ok()) {
        switch(internalState) {
            case idle:
                msg.status="Idle";
                switch (assignedCommand) {
                    case ef_move:
                        ROS_INFO("New end effector destination received." );
                        internalState=planning;
                        break;
                    case gripper_adjust:
                        ROS_INFO("New gripper distance received." );
                        internalState=gripper_adjusting;
                        break;
                    case homing_procedure:
                        ROS_INFO("Homing procedure command received.");
                        internalState=homing_planning;
                        break;
                    case none:
                        break;
                    default:
                        ROS_INFO("Unknown command. Skipping..." );
                        break;
                }
                break;
            case homing_planning:
                destinationPose=manipulator.get_end_effector_position(manipulator.get_joint_home_position());
                internalState=planning;
                break;
            case planning:
                msg.status="Planning";
                ROS_INFO("Starting motion planning..." );
                jointMotionMatrix=manipulator.compute_trajectory(destinationPose,interface.getPositions(),NODE_FREQUENCY);
                if (jointMotionMatrix.cols()>0) {
                    internalState=moving;
                    currentTrajectoryIndex=0;
                    ROS_INFO("Motion planning Complete!");
                    ROS_INFO("New destination: {%f, %f, %f, %f, %f, %f}\n",destinationPose(0),destinationPose(1),destinationPose(2),destinationPose(3),destinationPose(4),destinationPose(5));
                } else {
                    internalState=idle;
                    assignedCommand=none;
                    ROS_INFO("Nothing to do. Waiting for other commands");
                }
               
            case moving:
                msg.status="Moving";
                if (currentTrajectoryIndex<jointMotionMatrix.cols()) {
                    ROS_INFO("Changing Joint Positions: [%f, %f, %f, %f, %f, %f]",jointMotionMatrix(0,currentTrajectoryIndex),jointMotionMatrix(1,currentTrajectoryIndex),jointMotionMatrix(2,currentTrajectoryIndex),jointMotionMatrix(3,currentTrajectoryIndex),jointMotionMatrix(4,currentTrajectoryIndex),jointMotionMatrix(5,currentTrajectoryIndex));
                    interface.setPosition(jointMotionMatrix.col(currentTrajectoryIndex));
                    currentTrajectoryIndex++;
                } else {
                    ROS_INFO("Moving procedure complete! Waiting for other commands");
                    internalState=idle;
                    assignedCommand=none;
                }
                break;
            case gripper_adjusting:
                msg.status="Adjusting Gripper Distance";
                interface.setGripperValue(softGripper::translateGripperValue(gripperTargetValue));
                internalState=idle;
                assignedCommand=none;
                break;
            case initializing:
                msg.status="Initializing";
                if (interface.getSystemStatus()) {
                    internalState=homing_planning;
                    assignedCommand=none;
                    ROS_INFO("Interface connected! Starting homing procedure" );
                }
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            
            default:
                internalState=idle;
                assignedCommand=none;
                break;
        }
        end_effector_position=manipulator.get_end_effector_position(interface.getPositions());
        msg.x=end_effector_position(0);
        msg.y=end_effector_position(1);
        msg.z=end_effector_position(2);
        msg.phi=end_effector_position(3);
        msg.theta=end_effector_position(4);
        msg.psi=end_effector_position(5);
        mcpStatusPublisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

