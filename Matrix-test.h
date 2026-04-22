#include <assert.h>

#include <iostream>
#include <cmath>
#include "Matrix.h"
#include <cassert>
using std::cout;
using std::endl;
using std::sqrt;


#ifndef MATRIXTEST
#define MATRIXTEST

/**
 * @brief test creating a matrix out of columns
 * 
 */
void testColumnConstructor() {
    vector<Matrix> columns;
    columns.push_back(Matrix(4, 1, {1, 2, 3, 4}));
    columns.push_back(Matrix(4, 1, {5, 6, 7, 8}));
    columns.push_back(Matrix(4, 1, {9, 10, 11, 12}));
    Matrix a(columns);
    assert(a == Matrix(4, 3, {1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12}));
    cout << "Matrix column constructor success" << endl;
}

/**
 * @brief test adding matrices together
 * 
 */
void testPlus() {
    Matrix A(3, 1, {0, 2, 3});
    Matrix B(3, 1, {2, 4, 5});
    Matrix C = A + B;
    Matrix CTest(3, 1, {2, 6, 8});
    assert(C.matrixArray == CTest.matrixArray);
    A = Matrix(3, 3, {0, 1, 2, 3, 4, 5, 6, 7, 8});
    B = Matrix(3, 3, {8, 7, 6, 5, 4, 3, 2, 1, 0});
    C = A + B;
    CTest = Matrix(3, 3, {8, 8, 8, 8, 8, 8, 8, 8, 8});
    assert(C.matrixArray == CTest.matrixArray);
    try {
        A = Matrix(2, 2, {0, 0, 0, 0});
        B = Matrix();
        A + B;
        assert(1 == 0);
    } catch(const std::invalid_argument& e) {
    }
    cout << "Matrix plus success" << endl;
}

/**
 * @brief test multiplying a matrix by a scalar
 * 
 */
void testMultiplyByScalar() {
    Matrix A(2, 2, {1, 2, 3, 4});
    assert((2 * A).matrixArray == vector<double>({2, 4, 6, 8}));
    Matrix B(2, 3, {1, 2, 3, 4, 5, 6});
    assert((B * 3).matrixArray == vector<double>({3, 6, 9, 12, 15, 18}));
    cout << "Matrix scalar multiply success" << endl;
}

/**
 * @brief test muliplying matrices
 * 
 */
void testMultiplyByMatrix() {
    Matrix A(2, 2, {1, 2, 3, 4});
    Matrix B(2, 2, {1, 0, 0, 1});
    assert((A * B).matrixArray == A.matrixArray);
    A = Matrix(2, 3, {1, 2, 3, 4, 5, 6});
    B = Matrix(3, 2, {10, 11, 20, 21, 30, 31});
    assert((A * B).matrixArray == vector<double>({140, 146, 320, 335}));
    cout << "Matrix multiply success" << endl;
}


/**
 * @brief test taking the determinant of a matrix
 * 
 */
void testDeterminant() {
    Matrix A(2, 2, {1, 2, 3, 4});
    assert(A.det() == -2);
    Matrix B(3, 3, {1, -2, 3, 2, 0, 3, 1, 5, 4});
    assert(B.det() == 25);
    Matrix C(1, 2, {1, 2});
    try {
        C.det();
        assert(1 == 0);
    } catch(const std::invalid_argument& e) {
    }
    Matrix D(1, 1, {1});
    try {
        D.det();
        assert(1 == 0);
    } catch(const std::domain_error& e) {
    }
    cout << "Matrix determinant success" << endl;
}

/**
 * @brief test taking the adjuct of a matrix
 * 
 */
void testAdjunct() {
    Matrix A(3, 3, {2, 1, 3, 0, 2, 4, 1, 1, 2});
    assert(A.adjunct().matrixArray ==
           vector<double>({0, 1, -2, 4, 1, -8, -2, -1, 4}));
    Matrix C(3, 3, {2, 3, 4, -3, -3, -2, -2, 1, -1});
    assert(C.adjunct().matrixArray ==
           vector<double>({5, 7, 6, 1, 6, -8, -9, -8, 3}));
    Matrix B(3, 2, {1, 0, 0, 0, 0, 0});
    try {
        B.adjunct();
        assert(1 == 0);
    } catch(const std::invalid_argument& e) {
    }
    cout << "Matrix adjunct success" << endl;
}

/**
 * @brief test inverting various matrices
 * 
 */
void testInverse() {
    Matrix M(6,6,{
        2,5,6,7,9,0,
        0,0,6,5,4,99,
        9,6,7,8,0,0,
        0,0,0,100,12,1,
        90,65,67,6,0,9,
        8,0,4,6,6,0
        });
    assert(M.inverse()*M == Matrix(6,6,{
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1}));
    Matrix A(2, 2, {1, 2, 2, 3});
    assert(A.inverse().matrixArray == vector<double>({-3, 2, 2, -1}));
    Matrix C(3, 3, {2, 1, 3, 0, 2, 4, 1, 1, 2});
    assert(C.inverse().matrixArray ==
           vector<double>({0, -.5, 1, -2, -.5, 4, 1, .5, -2}));
    try {
        Matrix B(2, 2, {1, 1, 1, 1});
        B.inverse();
        assert(1 == 0);
    } catch(const std::invalid_argument& e) {
    }
    try {
        Matrix B(1, 2, {1, 1});
        B.inverse();
        assert(1 == 0);
    } catch(const std::invalid_argument& e) {
    }
    try {
        Matrix B(4, 4, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
        B.inverse();
        assert(1 == 0);
    } catch(const std::invalid_argument& e) {
    }

    cout << "Matrix inverse success" << endl;
}

/**
 * @brief test transposing a matrix
 * 
 */
void testTranspose() {
    Matrix A(3, 4, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12});
    assert(A.transpose().matrixArray ==
           vector<double>({1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12}));
    cout << "Matrix transpose success" << endl;
}

/**
 * @brief test taking a magnitude of a vector
 * 
 */
void testMatrixMagnitude() {
    Matrix B(3, 1, {2, 3, 4});
    assert(isCloseTo(B.magnitude(), std::sqrt(4 + 9 + 16)));
    cout << "Matrix magnitude success" << endl;
}

/**
 * @brief test getting the eignevalues and vectors of a matrix
 * 
 */
void testEigens() {
    Matrix A(3, 3, {-2, 4, 2, -2, 1, 2, 4, 2, 5});
    A = A.transpose()*A;
    vector<double> values;
    vector<Matrix> vectors;
    std::tie(values, vectors) = A.eigenValuesAndVectors();
    for(int i = 0; i < 3; i++) {
        assert(A*vectors[i] == values[i]*vectors[i]);
    }
    cout << "Matrix test eigens success" << endl;
}

/**
 * @brief run all matrix tests
 * 
 */
void testMatrixClass() {
    testInverse();
    testPlus();
    testMultiplyByMatrix();
    testMultiplyByScalar();
    testDeterminant();
    testAdjunct();
    testTranspose();
    testColumnConstructor();
    testMatrixMagnitude();
    testEigens();
}

#endif