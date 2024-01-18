#include "soft_gripper.h"

double* softGripper::translateGripperValue(const double fingerDistance) {
    double verifiedFingerDistance=fingerDistance;
    if (fingerDistance>MAX_GRIPPER_VALUE) verifiedFingerDistance=MAX_GRIPPER_VALUE;
    if (fingerDistance<MIN_GRIPPER_VALUE) verifiedFingerDistance=MIN_GRIPPER_VALUE;
    static double gripperValues[GRIPPER_FINGERS];
    gripperValues[0]=((verifiedFingerDistance*0.45)/0.04)-0.45;
    gripperValues[1]=gripperValues[0];
    return gripperValues;
}

double softGripper::getMaxFingerDistance() {
    return MAX_GRIPPER_VALUE;
}

double softGripper::getMinFingerDistance() {
    return MIN_GRIPPER_VALUE;
}