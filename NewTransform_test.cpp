/**
 * @file Transform_test.cpp
 * @author Tyler Lehrfeld
 * @brief test the transform class
 * @version 0.1
 * @date 2025-03-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #include "Point.h"
 #include "NewTransform.h"
 #include <cassert>
 #include <cmath>
 #include <math.h>
 
 void test_multiply() {
     NewTransform t1(0,0,0,10,10,10);
     NewTransform t2(M_PI/2,0,0,0,0,0);
     Point p1 = {.x = 0, .y = 1, .z = 0};
     Point expected = {.x= 10,.y= 10,.z= 11};
     Point result = (t1*t2*p1);
     assert((result).x == expected.x);
 }
 
 
 int main() {
     test_multiply();
     return 0;
 }