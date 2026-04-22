#include "Matrix.h"
#ifndef TRANSFORM
#define TRANSFORM
class Transform {
public:

    //Identity transform
    Transform();
    // create a transformation "A from B" using a rotation matrix and a position
    // vector
    Transform(Matrix R_AB, Matrix p_AB);
    // create a transformation "A from B" using an intermediary by combining
    // F_AC*F_CB
    Transform(Transform AC, Transform CB);
    Matrix operator*(Matrix u_B);
    Transform inverse();
    Matrix R_AB;
    Matrix p_AB;
    Transform operator*(Transform T_BC);
    void print();
};

#endif