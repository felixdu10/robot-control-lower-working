#include "Point.h"
#include "Matrix.h"
#include "Transform.h"
#include <cassert>
struct plane {
    Point normal;
    double D;
};


Matrix get_p_post(std::vector<Transform> F_OM1, std::vector<Transform> F_OM2, Transform F_M2N) {
    assert(F_OM1.size() == F_OM2.size());
    int num_transforms = F_OM1.size();
    vector<Matrix> xs;
    double tot_z = 0;
    for(int i = 0; i < num_transforms; i++) {
        xs.push_back(F_OM1[i].inverse()*F_OM2[i]*F_M2N*Matrix(3,1,{0,0,0}));
        tot_z += xs[i].matrixArray[2];
        xs[i].matrixArray[2] = 1;
    }
    Matrix A(xs);
    A = A.transpose();
    A.print();
    Matrix b(num_transforms,1,vector<double>(num_transforms, 0));
    for(int i = 0; i < num_transforms; i++) {
        b.matrixArray[i]= xs[i].matrixArray[0] * xs[i].matrixArray[0] + xs[i].matrixArray[1] * xs[i].matrixArray[1];
    }
    Matrix x = (A.transpose() * A).inverse() * A.transpose() * b;
    Matrix p_post(3,1,{x.matrixArray[0]/2, x.matrixArray[1]/2, tot_z/num_transforms});
    return p_post;
    
    
    

}