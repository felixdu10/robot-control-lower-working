#include "Pivot.h"
#include <vector>
#include "helperFunctions.h"
#include <iostream>

/**
 * @brief Create a pivot object with p_tip and p_post
 * 
 * @param FrameTransformationList 
 */
Pivot::Pivot(vector<Transform> FrameTransformationList) {
    // given k frames, R_k*p_tip-p_post=-p_k.
    // We want the form Ax=b to set up a least squares problem where A and b are
    // known. then we can put it in the form x=(A^T*A)^-1*A^T*b That means we
    // want x = p_tip | p_post, so we reform the equation to be R_k*p_tip -
    // p_post = - p_k.
    //  = (R_k |- I)(p_tip | -p_post) = -p_k
    // We can expand this to be one big matrix and vector by stacking the Rs and
    // calculating p_post-p_k for each k.
    int k = FrameTransformationList.size();
    Matrix A(3 * k, 6, vector<double>(18 * k, 0));
    // populate A
    for(int i = 0; i < k; i++) {
        bool Switch = false;
        int Rind = 0;
        int Iind = 0;
        for(int j = 0; j < 18; j++) {
            if(j % 3 == 0) {
                Switch = !Switch;
            }
            if(Switch) {
                A.matrixArray[18 * i + j] =
                    FrameTransformationList[i].R_AB.matrixArray[Rind];
                Rind++;
            } else {
                A.matrixArray[18 * i + j] =
                    (-1 * I).matrixArray[Iind];
                Iind++;
            }
        }
    }
    Matrix b(3 * k, 1, vector<double>(3 * k, 0));
    // populate b
    for(int i = 0; i < k; i++) {
        for(int j = 0; j < 3; j++) {
            b.matrixArray[3 * i + j] =
                -1 * FrameTransformationList[i].p_AB.matrixArray[j];
        }
    }
    // get least squares solution to  p_tip | p_post= x
    Matrix x = (A.transpose() * A).inverse() * A.transpose() * b;
    p_t = Matrix(3, 1, {x.matrixArray[0], x.matrixArray[1], x.matrixArray[2]});
    p_post = Matrix(3,1,{x.matrixArray[3], x.matrixArray[4], x.matrixArray[5]});
}
