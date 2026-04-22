#include "helperFunctions.h"
#include "Pivot.h"
#include <cassert>
#include <iostream>
using std::cout;
using std::endl;

#ifndef PIVOT_TEST
#define PIVOT_TEST


/**
 * @brief test the pivot calibration class by generating a 
 * random tip, post, and random pivot frames and testing the pivot calibration's performance
 * 
 */
void testPivot() {
    Matrix p_tip = generateRandomPoint();
    Matrix p_post = generateRandomPoint();
    vector<Transform> transformList;
    transformList = generatePivotFrames(p_tip, p_post);
    Pivot PivotCalibrater(transformList);
    Matrix p_t_Computed = PivotCalibrater.p_t;
    Matrix p_post_Computer = PivotCalibrater.p_post;
    assert(p_tip == p_t_Computed);
    cout << "pivot test success" << endl;
}

#endif