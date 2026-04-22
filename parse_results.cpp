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
    while(str == "" || str == " " || count < num_strs) {
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

NewTransforms parse_beginning(std::ifstream& file) {
 NewTransform T1 = get_transform(file,1);
 NewTransform T2 = get_transform(file, 2);
 get_transform(file,1);
 return {T1, T2};
 
}

int main() {
    std::ifstream results("resultsy2.txt");
    std::ifstream grid_file("gridy.txt");
    vector<double> xs({0,0,5,10,20,30,20,10,5,0,-5,-10,-20,-30,-20,-10,-5,0,5,10,20,30,20,10,5,0,-5,-10,-20,-30,-20,-10,-5,0,5,10,20,30,20,10,5,0,-5,-10,-20,-30,-20,-10,-5,0,5,10,20,30,20,10,5,0,-5,-10,-20,-30,-20,-10,-5,0});
    if(!grid_file.is_open()) {
        std::cout << "grid file not open";
        return -1;
    }
    int num_results = 67;
    std::string line;
    NewTransforms Ts = parse_beginning(results);
    NewTransform F_OM2_home = Ts.F_OM_home;
    NewTransform F_M2N = Ts.F_M2N;
    Matrix y(3,1,{0,1,0});
    Matrix forward_vec = F_OM2_home.to_transform().R_AB * F_M2N.to_transform().R_AB * y;
    Point forward = {forward_vec.matrixArray[0], forward_vec.matrixArray[1], forward_vec.matrixArray[2]};
    Matrix z(3,1,{0,0,1});
    Matrix up = (F_OM2_home.to_transform().R_AB * z);
    NewTransform prev_F_RN_measured;
    NewTransform prev_F_RN_expected;
    NewTransform prev_F_OM2;
    Point prev_expected_needle;
    Point prev_measured_needle;
    double prev_y = 0;
    double prev_x = 0;
    Point zero = {0,0,0};
    for(int i = 0; i < num_results; i++) {
        std::string line;
        grid_file >> line;
        if(line == "") {
            break;
        }
        std::string x = line.substr(1,line.find(",")-1);
        std::string y = line.substr(line.find(",")+1, line.length() - line.find(",") -2);
        //std::cout << x << ", " << y << std::endl;
        double cur_y = std::stod(y);
        double cur_x = xs[i];
        results >> line;
        results >> line;
        NewTransform F_OM1 = get_transform(results, 1);
        NewTransform F_OM2 = get_transform(results, 1);
        NewTransform F_RN_measured = get_transform(results, 2);
        NewTransform F_RN_expected = get_transform(results, 2);
        Point measured_base = F_RN_measured * zero;
        //Point expected_base = F_RN_expected * zero;
        //F_RN_measured.to_transform().p_AB.print_desmos();
        //F_RN_expected.to_transform().p_AB.print_desmos();
        Point expected_needle = get_point(results, 2);
        Point measured_needle = get_point(results, 2);
        Point diff = get_point(results, 2);
        results >> line;
        results >> line;
        double diff_mag;
        
        results >> diff_mag;
        //measured_needle.print_desmos();
        //expected_needle.print_desmos();
        Point needle = LOWER_END_EFFECTOR_TO_NEEDLEPOINT;
        needle.z = -115;
        //Point sec = {102.446, 23.6629, 811.533};
        //Point sec = {59.2545, -28.4828, 805.778};
        //Point sec = {82.8181, 15.7081, 838.96};
        //Point sec = {204.218, -18.0973, 677.676};
        //Point sec = {207.606, -23.5237, 685.488};
        Point sec = {0,0,0};
        Point vec = {36,22.25+11.6/2, -11.764-5};
        //(F_OM2 *F_M2N* zero).print();
        if(i != -1) {
            //std::cout << prev_y - cur_y << std::endl;
            //(F_OM2 * F_M2N * zero).print();
            //Point p_0 = (prev_F_OM2 * zero);
            //Point expected = (forward * (cur_y- 410));
            Point p = (F_OM2 *F_M2N* zero) - sec;
            //p.print_desmos();
            //expected.print();
            //std::cout << (p-expected).magnitude() << std::endl;
            //p.print_desmos();
            //(p - p_0).print_desmos();
            if(prev_x - cur_x < 0) {
                //std::cout << "(" << cur_y << ","<< abs((p - p_0).magnitude() - abs(prev_y - cur_y))<<")" <<std::endl;
                //std::cout << cur_y -410 << ", " << p.magnitude() << ";" << std::endl;
                //std::cout << cur_y -410 <<  ", " <<(F_OM1 * zero - sec).magnitude() << ";"<< std::endl;
                
                std::cout << cur_x <<  ", " <<(F_OM2 *F_M2N * zero - sec).magnitude() << ";"<< std::endl;
        
            }
            //std::cout << (p - p_0).magnitude() << std::endl;
            //acos(((F_OM2.to_transform().R_AB * z).transpose() * up).magnitude());

        }
        //std::cout << acos(((F_OM2.to_transform().R_AB * z).transpose() * up).magnitude()) << std::endl;
        prev_F_OM2 = F_OM2;
        prev_x = cur_x;
        prev_y = cur_y;
        //prev_F_RN_measured = F_RN_measured;
        //prev_F_RN_expected = F_RN_expected;
        //prev_expected_needle = expected_needle;
        //prev_measured_needle = measured_needle;
    }
}