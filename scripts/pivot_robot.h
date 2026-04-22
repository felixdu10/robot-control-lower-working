#include "../Pivot.h"
#include "../Matrix.h"
#include <string>
#include <fstream>
#include <chrono>
#include <thread>
#include "../NewTransform.h"
#include "../kinematic_structs.h"
#include "../restricted_plane_pivot.h"
#include "./pivot_needle.h"

#ifndef ROBOT_PIVOT
#define ROBOT_PIVOT



/**
 * @brief Get the transform from the robot marker to the robot. We do this by pivoting around the lower base
 * 
 * @param F_M2N The transform from the Needle frame to the marker frame.
 * @return NewTransform 
 */
NewTransform get_robot_pivot_transform(NewTransform F_M2N) {
    int num_measurements_needed = 50;
    int num_measurements_taken = 0;
    vector<Transform> F_OM1_list;
    vector<Transform> F_OM2_list;
    NewTransform previous_F_OM1;
    NewTransform previous_F_OM2;
    Point lower_base = LOWER_BASE;
    //colect data
    delay_ms(5000);
    while(num_measurements_taken < num_measurements_needed) {
        
        NewTransform F_OM2;
        NewTransform F_OM1;

        std::string file_path = "./out.txt";
        read_transform(file_path, F_OM1, true);
        read_transform(file_path, F_OM2, false);
        if(!(F_OM1 == previous_F_OM1 || F_OM2 == previous_F_OM2)) {
            F_OM1_list.push_back(F_OM1.to_transform());
            F_OM2_list.push_back(F_OM2.to_transform());
            num_measurements_taken++;
            std::cout << "measurement " <<num_measurements_taken << " taken" << std::endl;
            previous_F_OM2 = F_OM2;
            previous_F_OM1 = F_OM1;
        }
        delay_ms(200);
    }
    Matrix p_post_marker = get_p_post(F_OM1_list, F_OM2_list, F_M2N.to_transform());
    Matrix translation = p_post_marker + -1 * lower_base.to_matrix();
    return NewTransform(0,0,0, translation.matrixArray[0], translation.matrixArray[1], translation.matrixArray[2]);
    
    //This code was used for the first unconstrained pivot problem it does not constrain the z axis because we do not twist around it.
    /*

    Matrix A(num_measurements_taken * 3,6,vector<double>(num_measurements_taken*3*6, 0));
    for(int i = 0; i < num_measurements_taken; i++) {
        bool Switch = false;
        int R_M2ind = 0;
        int R_M1ind = 0;
        for(int j = 0; j < 18; j++) {
            if(j % 3 == 0) {
                Switch = !Switch;
            }
            if(Switch) {
                A.matrixArray[18 * i + j] =
                    F_OM2_list[i].R_AB.matrixArray[R_M2ind];
                R_M2ind++;
            } else {
                A.matrixArray[18 * i + j] =
                    -1*F_OM1_list[i].R_AB.matrixArray[R_M1ind];
                R_M1ind++;
            }
        }
    }

    Matrix b(3 * num_measurements_taken, 1, vector<double>(3 * num_measurements_taken, 0));
    // populate b
    for(int i = 0; i < num_measurements_taken; i++) {
        Matrix b_i = F_OM1_list[i].R_AB*upper_base.to_matrix() + F_OM1_list[i].p_AB+ -1*F_OM2_list[i].p_AB;
        for(int j = 0; j < 3; j++) {
            b.matrixArray[3 * i + j] = b_i.matrixArray[j];
        }
    }

    Matrix x = (A.transpose() * A).inverse() * A.transpose() * b;
    Matrix marker_to_upper_base = Matrix(3, 1, {x.matrixArray[0], x.matrixArray[1], x.matrixArray[2]});
    Matrix marker_to_robot_translation = Matrix(3,1,{x.matrixArray[3], x.matrixArray[4], x.matrixArray[5]});
    return NewTransform(0,0,0, marker_to_robot_translation.matrixArray[0], marker_to_robot_translation.matrixArray[1], marker_to_robot_translation.matrixArray[2]);
    */
    
}

#endif