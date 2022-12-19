#ifndef ROBOTICS_BLOCK_DIMENSION_H
#define ROBOTICS_BLOCK_DIMENSION_H
#include <string>
#include <iostream>

using namespace std;

class Block_Dimension {
private:
    float base, altezza;

public:
    Block_Dimension();
    Block_Dimension(float _base, float _altezza);
    ~Block_Dimension();
    void ins_base(float _base);
    void ins_altezza(float _altezza);
    friend ostream& operator << (ostream& os, Block_Dimension& bd);
};

ostream& operator << (ostream& os, Block_Dimension& bd);
#endif
