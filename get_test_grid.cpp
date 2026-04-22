const double HORIZONTAL_RANGE = 200;
const double VERTICAL_RANGE = 1000;


#include "inverse_kinematics.h"
#include "kinematic_structs.h"
#include "Robot.h"
#include <fstream>

int main() {
    std::ofstream point_file;
    point_file.open("grid.txt");
    std::ofstream sliders_file;
    sliders_file.open("sliders.txt");
    double no_extension_z = 0;
    double lowest_needle_extension = 1000;
    slider_positions sliders;
    for(double z = -70; z <= -60; z+=.1) {
        approach_definition approach = {{0,350, z}, 0, 0};
        Robot inverse_robot;
        sliders = inverse_kinematics(approach, NewTransform(0,0,0,0,0,0), inverse_robot);
        if(abs(inverse_robot.sliders.needle_extension) < abs(lowest_needle_extension)) {
            lowest_needle_extension = inverse_robot.sliders.needle_extension;
            std::cout << z << ": " << lowest_needle_extension  << " " << std::endl;
            
            no_extension_z = z;
        }
    }
    for(double x = -HORIZONTAL_RANGE/2; x <= HORIZONTAL_RANGE/2; x += 10) {
        for(double y = 350; y <= VERTICAL_RANGE/2; y += 10) {
            std::cout << x << " " << y << std::endl;
            try {
                if(x == 0 && y == 224) {
                    std::cout << "here" << std::endl;
                }
                Robot inverse_robot;
                approach_definition approach = {{x,y, no_extension_z}, 0, 0};
                sliders = inverse_kinematics(approach, NewTransform(0,0,0,0,0,0), inverse_robot);
                std::string error_string;
                if(inverse_robot.is_valid(error_string)) {
                    point_file << "(" <<x <<"," <<y<<")" << std::endl;
                    sliders_file << sliders.get_slider_string(true);
                }
            } catch (const std::runtime_error e) {
                std::cout << "caught" << std::endl;
            }
        }
    }
}