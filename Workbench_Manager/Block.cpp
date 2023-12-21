#include "Block.h"
#include "Block_Type.h"

using namespace std;

Block::Block() {
    Block_Type bt;
    block_type = bt;
    position.x = 0;
    position.y = 0;
    position.z = 0;
}

Block::Block(Block_Type _block_type, coordinates _position, float _angle) {
    block_type = _block_type;
    position = _position;
    angle = _angle;
}

Block::~Block() {}

void Block::ins_block_type(Block_Type _block_type) {
    block_type = _block_type;
}

void Block::ins_position(coordinates _position) {
    position = _position;
}

void Block::ins_angle(float _angle) {
    angle = _angle;
}

void Block::move(coordinates _position, float _angle) {
    //call MCP function

    position = _position;
    angle = _angle;
}

ostream& operator <<(ostream& os, Block& b){
    return os << "Block coordinates:\n x = " << b.position.x << "   y = " << b.position.y << "  z = " << b.position.z << "\n angle: " << b.angle << b.block_type;
}
