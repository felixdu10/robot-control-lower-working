#include "robot_controller.h"
#include "forward_kinematics.h"
#include "galil_control_calls.h"
#include "inverse_kinematics.h"
#include "kinematic_structs.h"
void home_protocol(slider_positions home) {
    bool left_behind, right_behind, left_middle_behind, right_middle_behind;
    left_behind = right_behind = left_middle_behind = right_middle_behind =
        false;
    std::string input_str;
    std::cout << "is the left slider behind the flag?" << std::endl;
    std::cin >> input_str;
    if (input_str == "y" || input_str == "yes") {
        left_behind = true;
    }
    std::cout << "is the right slider behind the flag?" << std::endl;
    std::cin >> input_str;

    if (input_str == "y" || input_str == "yes") {
        right_behind = true;
    }
    std::cout << "is the left middle slider behind the flag?" << std::endl;
    std::cin >> input_str;

    if (input_str == "y" || input_str == "yes") {
        left_middle_behind = true;
    }
    std::cout << "is the right middle slider behind the flag?" << std::endl;
    std::cin >> input_str;

    if (input_str == "y" || input_str == "yes") {
        right_middle_behind = true;
    }
    init_galil(0);
    std::cout << "Ready to home Low?" << std::endl;
    std::cin >> input_str;
    HomeLowBlocking(right_middle_behind, left_middle_behind);
    std::cout << "Ready to home Up?" << std::endl;
    std::cin >> input_str;
    HomeUpBlocking(right_behind, left_behind);
    std::cout << "Done Home" << std::endl;
}

void home_behind(slider_positions home) {
    bool left_behind, right_behind, left_middle_behind, right_middle_behind;
    left_behind = right_behind = left_middle_behind = right_middle_behind =
        true;
    init_galil(init_types::home);
    HomeLowBlocking(right_middle_behind, left_middle_behind);
    HomeUpBlocking(right_behind, left_behind);
}


void robot_controller::move_no_compensation(slider_positions sliders) {
	GoToBothBlocking(sliders);
	cur_positions = sliders;

}


void robot_controller::move(slider_positions positions) {
    init_galil(init_types::up_or_down);
    slider_positions stage_1_positions = positions;
    bool do_stage_two = false;
    if (stage_1_positions.left_slider_y < cur_positions.left_slider_y) {
        stage_1_positions.left_slider_y -= 10;
        do_stage_two = true;
    }
    if (stage_1_positions.left_middle_slider_y <
        cur_positions.left_middle_slider_y) {
        stage_1_positions.left_middle_slider_y -= 10;
        do_stage_two = true;
    }
    if (stage_1_positions.right_middle_slider_y <
        cur_positions.right_middle_slider_y) {
        stage_1_positions.right_middle_slider_y -= 10;
        do_stage_two = true;
    }
    if (stage_1_positions.right_slider_y < cur_positions.right_slider_y) {
        stage_1_positions.right_slider_y -= 10;
        do_stage_two = true;
    }
    GoToBothBlocking(stage_1_positions);
    if (do_stage_two) {
        GoToBothBlocking(positions);
    }
    cur_positions = positions;
}

void robot_controller::move(approach_definition def) {
    slider_positions new_positions =
        inverse_kinematics(def, NewTransform(0, 0, 0, 0, 0, 0), robot);
    move(new_positions);
}

void robot_controller::move(target_and_injection_point_approach approach) {
    slider_positions new_positions = inverse_kinematics(approach, robot);
    move(new_positions);
}

robot_controller::robot_controller(slider_positions home) {
    get_end_effector(home, robot);
    home_protocol(home);
    cur_positions = robot.sliders;
    cur_positions.left_middle_slider_y -= 10;
    cur_positions.right_middle_slider_y -= 10;
    cur_positions.right_slider_y -= 10;
    cur_positions.left_slider_y -= 10;
    init_galil(init_types::up_or_down);
    GoToLowBlocking(cur_positions.right_middle_slider_y,
                    cur_positions.left_middle_slider_y);
    GoToUpBlocking(cur_positions.right_slider_y, cur_positions.left_slider_y);
    home_behind(home);
    cur_positions = robot.sliders;
}
