#include <iostream>
#include <string>
#include <cmath>
#include "Matrix-test.h"
#include "PointCloudTest.h"
#include "Transform-test.h"
#include "Pivot-test.h"

using std::cout;
using std::endl;
using std::string;

/**
 * @brief runs all of tests in which the appropriate args are given
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char *argv[]) {
    for(int i = 0; i < argc; i++) {
        string argument = argv[i];
        if(argument == "matrix") {
            testMatrixClass();
        }
        if(argument == "transform") {
            testTransformClass();
        }
        if(argument == "pivot") {
            testPivot();
        }
    }    
}