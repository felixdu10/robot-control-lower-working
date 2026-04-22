#include "../Pivot.h"
#include <string>
#include <fstream>
#include <chrono>
#include <thread>
#include "../NewTransform.h"
#include "ini_helpers.h"
#ifndef NEEDLE_PIVOT
#define NEEDLE_PIVOT


/**
 * @brief Read a transform from a file (usually out.txt).
 * 
 * @param filename The file to read from
 * @param T The transform to update
 * @param first whether to read the first or second transform in the file
 * @return double registration error. If read fails, we return -1
 */
double read_transform(std::string filename, NewTransform& T, bool first) {
    std::ifstream file(filename);
    if(!file.is_open()) {
        std::cout << filename<<": file not open." << std::endl;
        return -1;
    }
    std::string transform_name;
    file >> transform_name;
    if(transform_name == "") {
        return -1;
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::string doubleStr;
            file >> doubleStr; 
            try {
                T.matrix[i][j] = stod(doubleStr);
            } catch (std::runtime_error e) {
                std::cout << "transform not read: could not read number" << std::endl;
                return -1;
            }
            
        }
    }
    double registration_error;
    file >> registration_error;
    if(first) {
        return registration_error;
    } else {

        file >> transform_name;
        if(transform_name == "") {
            return -1;
        }
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                std::string doubleStr;
                file >> doubleStr; 
                try {
                    T.matrix[i][j] = stod(doubleStr);
                } catch (std::runtime_error e) {
                    std::cout << "transform not read: could not read number" << std::endl;
                    return -1;
                }

            }
        }
    
    }
    file.close();
    return registration_error;
}


/**
 * @brief delay x milliseconds
 * 
 * @param milliseconds 
 */
void delay_ms(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}



/**
 * @brief Get the transform from the needle marker frame to the needle pivot frame.
 * 
 * @return NewTransform 
 */
NewTransform get_needle_pivot_transform(Matrix& p_bottom_linkage_in_optical_frame) {
    int num_measurements = 20;
    std::string file_path = "../out.txt";
    
    std::vector<Transform> transform_list;
    NewTransform old_transform;
    delay_ms(5000);
    while(transform_list.size() < num_measurements) {
        std::cout << transform_list.size() << std::endl;
        NewTransform T;
        read_transform(file_path, T, false);
        if(!(T == old_transform)) {
            transform_list.push_back(T.to_transform());
            T.print();
        }
        old_transform = T;
        delay_ms(200);
    }
    Pivot pivot_calibrator(transform_list);
    p_bottom_linkage_in_optical_frame = pivot_calibrator.p_post;
    NewTransform Needle(0,0,M_PI, pivot_calibrator.p_t.matrixArray[0], pivot_calibrator.p_t.matrixArray[1], pivot_calibrator.p_t.matrixArray[2]);
    return Needle;
}

/**
 * @brief Get the transform from the needle marker frame to the needle pivot frame.
 * 
 * @return NewTransform 
 */
NewTransform get_needle_pivot_transform_and_minimize_x(Matrix& p_bottom_linkage_in_optical_frame) {
    int num_measurements = 50;
    std::string file_path = "../out.txt";
    
    std::vector<Transform> transform_list;
    NewTransform old_transform;
    delay_ms(5000);
    while(transform_list.size() < num_measurements) {
        std::cout << transform_list.size() << std::endl;
        NewTransform T;
        read_transform(file_path, T, false);
        if(!(T == old_transform)) {
            transform_list.push_back(T.to_transform());
            T.print();
        }
        old_transform = T;
        delay_ms(200);
    }
    double lowest_x = 100;
    int lowest_degree = 0;
    NewTransform Needle(0,0,0,0,0,0);
    for(int i = 0; i < 360; i++) {
        Pivot pivot_calibrator(transform_list);
        Matrix p_tip = pivot_calibrator.p_t;
        if(abs(p_tip.matrixArray[0]) < lowest_x) {
            lowest_degree = i;
            lowest_x = p_tip.matrixArray[0];
            Needle = NewTransform(0,0,M_PI + static_cast<double>(i) * M_PI / 180.0, pivot_calibrator.p_t.matrixArray[0], pivot_calibrator.p_t.matrixArray[1], pivot_calibrator.p_t.matrixArray[2]);
        }
        for(int j = 0; j < transform_list.size(); j++) {
            transform_list[j] = Transform(rotation_from_axis_and_angle({0,0,1}, M_PI / 180.0) * transform_list[j].R_AB, transform_list[j].p_AB);
        }

    }
    std::cout << "degree_shift: "  << lowest_degree << std::endl;
    return Needle;
}

#endif
