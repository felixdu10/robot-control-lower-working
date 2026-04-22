#include <fstream>
#include "NewTransform.h"
#include "Matrix.h"
#include "kinematic_structs.h"


NewTransform get_transform(std::ifstream& file, int num_strs) {
    std::string str = "";
    double num;
    int count = 0;
    while(str == "" || str == " " || count < num_strs) {
        file >> str;
        if(!(str == "" || str == " "))
            count++;
    }
    NewTransform T;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            file >> num;
            T.matrix[i][j] = num;
        }
        
    }
    return T;
}

Point get_point(std::ifstream& file, int num_strs) {
    Point p;
    std::string str = "";
    double num;
    int count = 0;
    while((str == "" || str == " ") && count < num_strs) {
        file >> str;
        if(!(str == "" || str == " "))
            count++;
    }
    file >> p.x;
    file >> p.y;
    file >> p.z;
    return p;
    
}

struct NewTransforms {
    NewTransform F_M2N;
    NewTransform F_OM_home;
};

NewTransform parse_beginning(std::ifstream& file) {
    return get_transform(file,1);
 
}

int main() {

    vector<double> loop({0,5,10,15,10,5,0,-5,-10,-15,-10,-5});
    std::ifstream results("angular_results.txt");
    int num_results = 100;
    std::string line;
    NewTransform F_M2N = parse_beginning(results);
    Matrix y(3,1,{0,1,0});
    NewTransform prev_F_RN_measured;
    NewTransform prev_F_RN_expected;
    NewTransform prev_F_OM2;
    Point prev_expected_needle;
    Point prev_measured_needle;
    double prev_y = 0;
    double prev_x = 0;
    Point RCM_PT;
    Point zero = {0,0,0};
    for(int i = 0; i < num_results; i++) {
        NewTransform F_OM1 = get_transform(results, 1);
        NewTransform F_OM2 = get_transform(results, 1);
        Point measured_needle = get_point(results, 0);
        if(i == 0) {
            RCM_PT = measured_needle;   
        }
        std::cout <<loop[i % loop.size()] <<"," <<(measured_needle - RCM_PT).magnitude() << std::endl;
        
        
    }
}