#include "forward_kinematics.h"
#include "kinematic_structs.h"
#include <fstream>
#include <iostream>

int main() {
  std::ifstream grid_file("grid.txt");

  if (!grid_file.is_open()) {
    std::cout << "grid file not open";
    return -1;
  }
  std::ifstream slider_file("sliders.txt");
  if (!slider_file.is_open()) {
    std::cout << "slider file not open";
    return -1;
  }
  std::ofstream img_file("./scripts/img.csv");
  img_file << "x,y,val" <<std::endl;
  std::string next_word;
  while (grid_file >> next_word) {
    double left_slider, left_middle_slider, right_middle_slider, right_slider,
        needle_extension;
    slider_file >> left_slider >> left_middle_slider >> right_middle_slider >>
        right_slider >> needle_extension;
    //std::cout << next_word << std::endl;
    slider_positions sliders = {left_slider, left_middle_slider,
                                right_middle_slider, right_slider,  -115-LOWER_END_EFFECTOR_TO_NEEDLEPOINT.z};
    Robot forward_robot;
    Point ee = get_end_effector(sliders, forward_robot);
    double max_error = 0;
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          for (int I = -1; I <= 1; I++) {
            slider_positions new_pos = {sliders.left_slider_y + 0.03 * static_cast<double>(i),
                                        sliders.left_middle_slider_y + 0.03 * static_cast<double>(j),
                                        sliders.right_middle_slider_y + 0.03 * static_cast<double>(k),
                                        sliders.right_slider_y + .03 * static_cast<double>(I),
                                        sliders.needle_extension};
            Point ee_adjusted = get_end_effector(new_pos, forward_robot);
            double mag = (ee_adjusted - ee).magnitude();
            if (mag > max_error) {
              max_error = mag;
            }
          }
        }
      }
    }
    std::string x = next_word.substr(1,next_word.find(",")-1);
        std::string y = next_word.substr(next_word.find(",")+1, next_word.length() - next_word.find(",") -2);
        img_file << x << ", " << y << ", " << max_error << std::endl;
  }
  return 0;
}