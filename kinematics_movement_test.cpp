#include "forward_kinematics.cpp"
#include "galil_control_calls.h"

int main() {
    double left_slider_y, left_middle_slider_y, right_middle_slider_y, right_slider_y;
    std::cin >> left_slider_y;
    std::cin >> left_middle_slider_y;
    std::cin >> right_middle_slider_y;
    std::cin >> right_slider_y;
    Point end_effector = get_end_effector(
        {-63, left_slider_y, 0}, {-21, left_middle_slider_y, 0},
        {21.07, right_middle_slider_y, 0}, {63, right_slider_y, 0},
        {0, 186.4, 0}, {0, 223.62, 0}, 0);
    end_effector.print();
    //move_robot_with_slider_positions({left_slider_y, left_middle_slider_y, right_middle_slider_y, right_slider_y});
}