#ifndef ROBOTICS_BLOCK_TYPE_H
#define ROBOTICS_BLOCK_TYPE_H
#include "Block_Dimension.h"
#include <string>

using namespace std;

class Block_Type {
private:
    Block_Dimension block_dimension;
    string colour;

public:
    Block_Type();
    Block_Type(Block_Dimension _block_dimension, string _colour);
    ~Block_Type();
    void ins_block_dimension(Block_Dimension _block_dimension);
    void ins_colour(string _colour);
    friend ostream& operator << (ostream& os, Block_Type& bt);
};

ostream& operator << (ostream& os, Block_Type& bt);

#endif
