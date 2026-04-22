#include "Transform.h"
#include "Matrix.h"
#include "helperFunctions.h"
#include <iostream>
#include <cassert>
#ifndef TRANSFORMTEST
#define TRANSFORMTEST

using std::cout;
using std::endl;

/**
 * @brief test the transform class multiply operation
 * 
 */
void testTransformMultiply() {
    Transform A = generateRandomTransform();
    Transform B = generateRandomTransform();
    Transform AB = A*B;
    assert(AB.R_AB == A.R_AB*B.R_AB);
    assert(AB.p_AB == A.R_AB*B.p_AB+A.p_AB);
    cout << "Transform multiply success" << endl;
}

/**
 * @brief test the transform class inverse operation
 * 
 */
void testTransformInverse() {
    Transform A = generateRandomTransform();
    assert((A*A.inverse()).R_AB == I);
    assert((A*A.inverse()).p_AB == origin);
    Matrix vec = generateRandomPoint();
    Matrix newVec = A*vec;
    Matrix oldVec = A.inverse()*newVec;
    assert(vec == oldVec);
    cout << "Transform inverse success" << endl;
}

/**
 * @brief test the transform class multiply by vector operation
 * 
 */
void testTransformMultiplyByVector() {
    Transform F_AB = generateRandomTransform();
    Matrix vec_B = generateRandomPoint(); 
    assert(F_AB*vec_B == F_AB.R_AB*vec_B+F_AB.p_AB);
    cout << "Transform multiply by vector success" << endl;
}

/**
 * @brief test the transform class
 * 
 */
void testTransformClass() {
    testTransformMultiply();
    testTransformInverse();
    testTransformMultiplyByVector();
}

#endif