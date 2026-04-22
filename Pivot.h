#include "Matrix.h"
#include "Transform.h"

#ifndef PIVOT_1
#define PIVOT_1
/**
 * @brief pivot class with p_tip and p_post 
 * 
 */
class Pivot {
    public:
    Pivot(std::vector<Transform> FrameTransformationList);
    Matrix p_t;
    Matrix p_post;
};

#endif