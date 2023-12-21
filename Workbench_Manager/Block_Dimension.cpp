#include "Block_Dimension.h"

using namespace std;

Block_Dimension::Block_Dimension() {
    base = 0;
    altezza = 0;
}

Block_Dimension::Block_Dimension(float _base, float _altezza) {
    base = _base;
    altezza = _altezza;
}

Block_Dimension::~Block_Dimension() {}

void Block_Dimension::ins_altezza(float _altezza) {
    altezza = _altezza;
}

void Block_Dimension::ins_base(float _base) {
    base = _base;
}

ostream& operator<<(ostream& os, Block_Dimension& bd){
    return os << "\nBlock Dimensions:\nBasis: " << bd.base <<"\nHeight: " << bd.altezza;
}

