#include "pivot_needle.h"

NewTransform get_average_transform(vector<NewTransform> transforms) {
    Quaternion q_sums;
    Matrix t_sums(3,1,{0,0,0});
    for(int i = 0; i <transforms.size(); i++) {
        Quaternion q_i = transforms[i].to_quaternion();
        q_sums.w += q_i.w;
        q_sums.x += q_i.x;
        q_sums.y += q_i.y;
        q_sums.z +=q_i.z;
        t_sums = t_sums + transforms[i].to_transform().p_AB;
    }
    q_sums.w = q_sums.w / transforms.size();
    q_sums.x = q_sums.x / transforms.size();
    q_sums.y = q_sums.y / transforms.size();
    q_sums.z = q_sums.z / transforms.size();
    NewTransform T;
    T.from_quaternion(q_sums);
    T.matrix[0][3] = t_sums.matrixArray[0] / transforms.size();
    T.matrix[1][3] = t_sums.matrixArray[1] / transforms.size();
    T.matrix[2][3] = t_sums.matrixArray[2] / transforms.size();
    return T;
};


int main() {
    Matrix p_bottom_linkage_home_in_optical_coordinates(3,1,{0,0,0});
    vector<NewTransform> Ts;
    for(int i = 0; i < 5; i++) {
        NewTransform T = get_needle_pivot_transform(p_bottom_linkage_home_in_optical_coordinates);
        T.print();
        Ts.push_back(T);
    
    }
    get_average_transform(Ts).print();

    //T = get_needle_pivot_transform_and_minimize_x(p_bottom_linkage_home_in_optical_coordinates);
    //T.print();
}
