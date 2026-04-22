#include "../inverse_kinematics.h"
#include "../kinematics.h"
#include "templated_classes/Templated_Point.h"
#include "templated_classes/Templated_Transform.h"
#include <cassert>
int main() {
  for (int i = 375; i < 445; i += 5) {
    External::Robot inverse_robot;
    External::target_and_injection_point_approach approach = {{0, double(i), -63},
                                                              {0, double(i), -62}};
    External::slider_positions guess_pos =
        External::inverse_kinematics(approach, inverse_robot);

    Parameters<double> p = get_default_parameters<double>();
    p = adjust_params(p);
    Thetas<double> thetas = {.theta_1 = guess_pos.right_slider_y,
                             .theta_2 = guess_pos.right_slider_y,
                             .theta_3 = guess_pos.left_middle_slider_y,
                             .theta_4 = guess_pos.right_middle_slider_y,
                             .theta_5 = guess_pos.needle_extension};

    Transform<double> target = get_end_effector(thetas, p);
    Thetas<double> thetas_guess = get_thetas(target, p);
    Transform<double> actual_target = get_end_effector(thetas_guess, p);

    Point<double> target_z = {actual_target.R.matrix[0][2],
                              actual_target.R.matrix[1][2],
                              actual_target.R.matrix[2][2]};
    Point<double> actual_z = {target.R.matrix[0][2], target.R.matrix[1][2],
                              target.R.matrix[2][2]};
    std::cout << "targetting error: "
              << (actual_target.p - target.p).magnitude() << std::endl;
    std::cout << "angular error: " << std::acos(target_z * actual_z)
              << std::endl;
  }
}
