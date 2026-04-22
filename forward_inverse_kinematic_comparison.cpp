#include "Point.h"
#include "Robot.h"
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "kinematic_structs.h"
#include <iostream>

//slider_positions home_positions = {
//    BASE_TO_SLIDER_MAX - 53.92 - HALF_SLIDER_WIDTH,
//    BASE_TO_SLIDER_MAX - 19.94923 - HALF_SLIDER_WIDTH,
//    BASE_TO_SLIDER_MAX - 16.63853 - HALF_SLIDER_WIDTH,
//    BASE_TO_SLIDER_MAX - 53.92 - HALF_SLIDER_WIDTH,
//    20
//};

slider_positions home_positions = {
    BASE_TO_SLIDER_MAX - 60   ,//- HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 26.79,// - HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 23.48,//- HALF_SLIDER_WIDTH,
    BASE_TO_SLIDER_MAX - 60   ,//- HALF_SLIDER_WIDTH,
    0
};

void update_robot_forward(slider_positions sliders, Robot& robot) {
    robot.sliders = sliders;
    get_end_effector({sliderXs[0], sliders.left_slider_y, 0}, {sliderXs[1],sliders.left_middle_slider_y, 0},{sliderXs[2], sliders.right_middle_slider_y, 0}, {sliderXs[3], sliders.right_slider_y, 0}, UPPER_BASE, LOWER_BASE, sliders.needle_extension, robot);
}

void update_robot_backward(approach_definition approach_definition, Robot& robot) {
  inverse_kinematics(approach_definition, NewTransform(0, 0, 0, 0, 0, 0), robot);
}

bool compare_point(Point a, Point b) {
    return isclose((a - b).magnitude(), 0);
}

bool compare_linkages(linkage_array links1, linkage_array links2) {
    bool equal = true;
    if(!compare_point(links1.left_midpoint, links2.left_midpoint)) {
        std::cout << "left midpoint not equal: " << std::endl;
        links1.left_midpoint.print();
        links2.left_midpoint.print();
        equal = false;
    }
    if(!compare_point(links1.right_midpoint, links2.right_midpoint)) {
        std::cout << "right midpoint not equal: " << std::endl;
        links1.right_midpoint.print();
        links2.right_midpoint.print();
        equal = false;
    }
    if(!compare_point(links1.base, links2.base)) {
        std::cout << "base not equal: " << std::endl;
        links1.base.print();
        links2.base.print();
        equal = false;
    }
    if(!compare_point(links1.extended_end_effector, links2.extended_end_effector)) {
        std::cout << "extended end effector not equal: " << std::endl;
        links1.extended_end_effector.print();
        links2.extended_end_effector.print();
        equal = false;
    }
    if(!compare_point(links1.left_joint, links2.left_joint)) {
        std::cout << "left joint not equal: " << std::endl;
        links1.left_joint.print();
        links2.left_joint.print();
        equal = false;
    }
    if(!compare_point(links1.right_joint, links2.right_joint)) {
        std::cout << "right joint not equal: " << std::endl;
        links1.right_joint.print();
        links2.right_joint.print();
        equal = false;
    }
    if(!compare_point(links1.linkage_end_effector, links2.linkage_end_effector)) {
        std::cout << "linkage end effector not equal: " << std::endl;
        links1.linkage_end_effector.print();
        links2.linkage_end_effector.print();
        equal = false;
    }
    return equal;
}

bool compare_sliders(slider_positions sliders1, slider_positions sliders2) {
    std::string msg = "";
    if(!isclose(sliders1.left_slider_y, sliders2.left_slider_y)) {
        msg += "Discrepency in left slider. ";
    }
    if(!isclose(sliders1.left_middle_slider_y, sliders2.left_middle_slider_y)) {
        msg += "Discrepency in left middle slider. ";
    }
    if(!isclose(sliders1.right_middle_slider_y, sliders2.right_middle_slider_y)) {
        msg += "Discrepency in right middle slider. ";
    }
    if(!isclose(sliders1.right_slider_y, sliders2.right_slider_y)) {
        msg += "Discrepency in right slider. ";
    }
    if(!isclose(sliders1.needle_extension, sliders2.needle_extension)) {
        msg += "Discrepency in needle extension. ";
    }
    std::cout << msg << std::endl;
    return msg == "";
}

void compare_robots(Robot robot1, Robot robot2) {
    if(!compare_point(robot1.origin, robot2.origin)) {
        std::cout << "origin not equal: " << std::endl;
        robot1.origin.print();
        robot2.origin.print();
    }
    if(!compare_point(robot1.x_prime, robot2.x_prime)) {
        std::cout << "xprime not equal: " << std::endl;
        robot1.x_prime.print();
        robot2.x_prime.print();
    }
    if(!compare_point(robot1.y_prime, robot2.y_prime)) {
        std::cout << "yprime not equal: " << std::endl;
        robot1.y_prime.print();
        robot2.y_prime.print();
    }
    if(!compare_point(robot1.z_prime, robot2.z_prime)) {
        std::cout << "zprime not equal: " << std::endl;
        robot1.z_prime.print();
        robot2.z_prime.print();
    }
    if(!compare_linkages(robot1.top_linkage, robot2.top_linkage)) {
        std::cout << "top linkage not equal: " << std::endl;
    }
    if(!compare_linkages(robot1.bottom_linkage, robot2.bottom_linkage)) {
        std::cout << "bottom linkage not equal: " << std::endl;
    }
    if(!compare_sliders(robot1.sliders, robot2.sliders)) {
        std::cout << "sliders not equal: " << std::endl;
        robot1.sliders.print(true);
        robot2.sliders.print(true);
    }    
}

int main() {
    Robot forward_robot;
    Robot inverse_robot;
    
    
//
    //approach_definition inverse_approach = {{-60, 410, 64.9},
    //                         0,
    //                         0};
//
    //Robot inverse_robot;
    //update_robot_backward(inverse_approach, inverse_robot);
    ////inverse_robot.sliders.print();
    //slider_positions positions = inverse_robot.sliders;
    //double right = BASE_TO_SLIDER_MAX - positions.right_slider_y - HALF_SLIDER_WIDTH;
    //double left = BASE_TO_SLIDER_MAX - positions.left_slider_y - HALF_SLIDER_WIDTH;
    //double right_middle = BASE_TO_SLIDER_MAX - positions.right_middle_slider_y - HALF_SLIDER_WIDTH;
    //double left_middle = BASE_TO_SLIDER_MAX - positions.left_middle_slider_y - HALF_SLIDER_WIDTH;
    //std::cout << left << " " << left_middle << " " << right_middle << " " << right << std::endl;
    //update_robot_forward(inverse_robot.sliders, forward_robot);
    //compare_robots(inverse_robot, forward_robot);
    forward_robot.sliders = home_positions;
    Point target_p = get_end_effector(home_positions, forward_robot);
    target_p.print_desmos();
    forward_robot.bottom_linkage.extended_end_effector.print_desmos();
    forward_robot.z_prime.print_desmos();
    
    Point injection_p = target_p + forward_robot.z_prime;
    inverse_kinematics({target_p, injection_p}, inverse_robot);
    compare_robots(forward_robot, inverse_robot);

}