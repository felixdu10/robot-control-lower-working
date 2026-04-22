#include <string>
#include <iostream>
#include <fstream>
#include "NewTransform.h"
#include "PointCloudTransform.h"
#include "galil_control_calls.h"
#include <string>
#include "./scripts/pivot_needle.h"
#include "./scripts/pivot_robot.h"
#include "inverse_kinematics.h"
#include "forward_kinematics.h"

slider_positions home_positions = {
    BASE_TO_SLIDER_MAX - 60   ,//- HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 26.79,// - HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 23.48,//- HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 60   ,//- HALF_SLIDER_WIDTH,
    0
};

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
    std::ofstream results_file("home_results.txt");
    
    
    std::string command = "";
    int num_measurements_needed = 10;
    int num_measurements_taken = 0;
    vector<NewTransform> F_OM1_list;
    vector<NewTransform> F_OM2_list;
    NewTransform previous_F_OM1;
    NewTransform previous_F_OM2;
    std::cout <<"collect home positions" << std::endl;
    //std::cin >> command;
    NewTransform F_OM1 = get_average_transform(F_OM1_list);
    NewTransform F_M1R(0,0,M_PI,0,0,0);
    //F_M1R.matrix[0][3] = -37.5;
    //F_M1R.matrix[1][3] = -5;
    //F_M1R.matrix[2][3] = -71.99;
    //F_M1R.matrix[0][0] = -1;
    //F_M1R.matrix[0][1] = 0;
    //F_M1R.matrix[0][2] = 0;
    //F_M1R.matrix[1][0] = 0;
    //F_M1R.matrix[1][1] = -1;
    //F_M1R.matrix[1][2] = 0;
    //F_M1R.matrix[2][0] = 0;
    //F_M1R.matrix[2][1] = 0;
    //F_M1R.matrix[2][2] = 1;
    results_file << "F_M1R" <<std::endl <<F_M1R.to_string();
    int num_trials = 0;
    int tot_trials = 21;
    Point expected = {0,0,0};
    init_galil(1);
    while(num_trials < tot_trials) {
        std::cout << "move robot and take readings?" << std::endl;
        //delay_ms(500);
        //std::cin >> command;
        if(command == "q") {
            break;
        }
        
        HomeUpBlocking(1,1);
        vector<NewTransform> F_OM1s;
        vector<NewTransform> F_OM2s;
        NewTransform F_OM1(0,0,0,0,0,0);
        NewTransform F_OM2(0,0,0,0,0,0);
        while(F_OM1s.size() < 10) {
            NewTransform newF1(0,0,0,0,0,0);
            read_transform("./out.txt", newF1, true);
            NewTransform newF2(0,0,0,0,0,0);
            read_transform("./out.txt", newF2, false); 
            if(!(newF1 == F_OM1 || newF2 == F_OM2)) {
                F_OM1s.push_back(newF1);
                F_OM1 = newF1;
                F_OM2s.push_back(newF2);
                F_OM2 = newF2;
            }
            delay_ms(100);
        }
        F_OM1 = get_average_transform(F_OM1s);
        F_OM2 = get_average_transform(F_OM2s);
        results_file << "F_OM1" <<std::endl << F_OM1.to_string();
        Point tip = {0,0,0};
        Point actual = F_OM1 * F_M1R * tip; 
        if(num_trials != 0) {
            results_file << "difference vector: " << std::endl <<
            (actual - expected).to_string(false);
            results_file << "difference magnitude" <<std::endl << (actual - expected).magnitude() << std::endl;
        } else {
            expected = actual;
            results_file << "first home: " << std::endl << expected.to_string(false);
        }
        num_trials++;
    }
    stop_galil();
    return 0;
}