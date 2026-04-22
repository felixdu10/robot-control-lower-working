#ifndef ROBOT_CONTROLLER
#define ROBOT_CONTROLLER

#include "Robot.h"
#include "kinematic_structs.h"

class robot_controller {
  public:
    Robot robot;
    slider_positions cur_positions;
    robot_controller(slider_positions home);
    void move(slider_positions positions);
    void move_no_compensation(slider_positions positions);
    void move(approach_definition approach);
    void move(target_and_injection_point_approach approach);
};
#endif // ROBOT_CONTROLLER
