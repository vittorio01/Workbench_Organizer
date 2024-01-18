#ifndef __SOFT_GRIPPER_H__
#define __SOFT_GRIPPER_H__ 

#define MAX_GRIPPER_VALUE 0.065
#define MIN_GRIPPER_VALUE 0
#define GRIPPER_FINGERS   2

class softGripper {
    public:
        static double getMaxFingerDistance();
        static double getMinFingerDistance();
        static double* translateGripperValue(const double fingerDistance); 
};

#endif 