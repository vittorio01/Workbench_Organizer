#include "Block_Type.h"
#include "Block_Dimension.h"

using namespace std;

Block_Type::Block_Type() {
    Block_Dimension bd;
    block_dimension = bd;
    colour = "";
}

Block_Type::Block_Type(Block_Dimension _block_dimension, string _colour) {
    block_dimension = _block_dimension;
    colour = _colour;
}

Block_Type::~Block_Type() {}

void Block_Type::ins_block_dimension(Block_Dimension _block_dimension) {
    block_dimension = _block_dimension;
}

void Block_Type::ins_colour(string _colour) {
    colour = _colour;
}

ostream& operator << (ostream& os, Block_Type& bt){
    return os << "\nBlock Type:\ncolour: " << bt.colour << bt.block_dimension;
}
