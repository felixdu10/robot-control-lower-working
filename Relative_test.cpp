#include <string>
#include <iostream>
#include <fstream>
#include "NewTransform.h"
#include "PointCloudTransform.h"
#include "galil_control_calls.h"
#include <string>
#include "./scripts/pivot_needle.h"
#include "inverse_kinematics.h"
#include "forward_kinematics.h"

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



Robot inverse_robot;

int main() {
    std::ofstream results_file("angular_results.txt");
    
    NewTransform F_M2N(0,0,M_PI, .00558085, 28.3789, -5.49509);
    results_file << "F_M2N" <<std::endl << F_M2N.to_string();
    std::string command;
    //delay_ms(5000);
    vector<double> loop({0,5,10,15,10,5,0,-5,-10,-15,-10,-5});
    
    Point target = {0, 425, -90};
    int num_loops = 3;
    init_galil(3);
    for(int i = 0; i < num_loops; i++) {
        for(int j = 0; j < loop.size(); j++) {
            NewTransform T_RP(0,0,0,0,0,0);
            approach_definition approach = {
                target,
                M_PI/2,
                loop[j]*M_PI/180.0
            };
            slider_positions sliders = inverse_kinematics(approach, T_RP, inverse_robot);
            inverse_robot.bottom_linkage.extended_end_effector.print_desmos();
            move_robot_with_slider_positions(sliders);
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
            results_file << "F_OM2" <<std::endl << F_OM2.to_string();
            Point needle_tip = LOWER_END_EFFECTOR_TO_NEEDLEPOINT;
            needle_tip.z -= sliders.needle_extension;
            results_file << (F_OM2 * F_M2N * needle_tip).to_string(false);
        }
    }
    stop_galil();
    
    
    
    
    
    
}