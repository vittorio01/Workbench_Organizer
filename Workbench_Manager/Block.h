#ifndef ROBOTICS_BLOCK_H
#define ROBOTICS_BLOCK_H
#include "Block_Type.h"

using namespace std;

struct coordinates{
    float x;
    float y;
    float z;
};


class Block {
private:
    Block_Type block_type;
    coordinates position;
    float angle;

public:
    Block();
    Block(Block_Type _block_type, coordinates _position, float _angle);
    ~Block();
    void ins_block_type(Block_Type _block_type);
    void ins_position(coordinates _position);
    void ins_angle(float _angle);
    void move(coordinates _position, float _angle);
    friend ostream& operator <<(ostream& os, Block& b);
};

ostream& operator <<(ostream& os, Block& b);

#endif
