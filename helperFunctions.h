#include <cmath>
#include <stdlib.h>
#include <time.h>

#include "Matrix.h"
#include "Transform.h"
#include <cassert>
#include <tuple>
#include <map>

#ifndef HELPER
#define HELPER

//this is a map of choose values in order to save compute time
const std::map<std::tuple<int, int>, int> chooseMap = {
    {std::tuple(5, 0), 1},
    {std::tuple(5, 1), 5},
    {std::tuple(5, 2), 10},
    {std::tuple(5, 3), 10},
    {std::tuple(5, 4), 5},
    {std::tuple(5, 5), 1},
};

static Matrix I(3, 3, {1, 0, 0, 0, 1, 0, 0, 0, 1});
static Matrix origin(3, 1, {0, 0, 0});

/**
 * @brief generate a random double from 0-1
 * 
 * @return double 
 */
static double randomdouble() {
    return static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
}

/**
 * @brief generate a random rotation matrix
 * 
 * @return Matrix 
 */
static Matrix generateRandomRotation() {

    double rotX = randomdouble() * 2 * M_PI;
    double rotY = randomdouble() * 2 * M_PI;
    double rotZ = randomdouble() * 2 * M_PI;

    // generate a rotation matrix using random angles
    Matrix Rx(
        3, 3,
        {1, 0, 0, 0, (double)cos(rotX), -sinf(rotX), 0, sinf(rotX), cosf(rotX)});
    Matrix Ry(3, 3,
              {cosf(rotY), 0, sinf(rotY), 0, 1, 0, -sinf(rotY), 0, cosf(rotY)});
    Matrix Rz(3, 3,
              {cosf(rotZ), -sinf(rotZ), 0, sinf(rotZ), cosf(rotZ), 0, 0, 0, 1});
    Matrix R = Rx * Ry * Rz;
    return R;
}



/**
 * @brief Choose function. 
 * WARNING: This is a mapping as the numbers are very small and there are only a few possible inputs in this program
 * 
 * @param N 
 * @param R 
 * @return int 
 */
static int choose(int N, int R) {
    return chooseMap.at(std::tuple(N, R));
}

/**
 * @brief Generates an identity matrix of a given size
 * 
 * @param size 
 * @return Matrix 
 */
static Matrix generate_identity(int size) {
    vector<double> arr;
    for(int i = 0; i < size; i++) {
        for(int j = 0; j < size; j++) {\
            arr.push_back(i == j ? 1 : 0);
        }
    }
    return Matrix(size, size, arr);
}
/**
 * @brief generate a random 3D point
 * 
 * @return Matrix 
 */
static Matrix generateRandomPoint() {
    // generate a random vector with values from -10 to +10
    Matrix p(3, 1,
             {randomdouble() * 20 - 10, randomdouble() * 20 - 10,
              randomdouble() * 20 - 10});
    return p;
}

/**
 * @brief generate a random transform
 * 
 * @return Transform 
 */
static Transform generateRandomTransform() {
    // create the transform
    return Transform(generateRandomRotation(), generateRandomPoint());
}

/**
 * @brief generate pivot frames for a pivot calibration
 * 
 * @param p_tip 
 * @param p_post 
 * @return vector<Transform> 
 */
static vector<Transform> generatePivotFrames(Matrix p_tip, Matrix p_post) {
    vector<Transform> pivots;
    // generate 15 frames of pivots
    for(int i = 0; i < 15; i++) {
        // first come up with a rotation, then work to get the displacement
        // based on the tip and post
        Matrix R_i = generateRandomRotation();
        // given p_post = R_i*p_tip+p_i, p_i = p_post - R_i*p_tip
        Matrix p_i = p_post + -1 * R_i * p_tip;
        pivots.push_back(Transform(R_i, p_i));
        assert(p_post == pivots[i] * p_tip);
    }
    return pivots;
}


#endif