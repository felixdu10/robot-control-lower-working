#include "kinematic_structs.h"
#include "robot_controller.h"

static std::string input;
int main() {
    // setup
    slider_positions home_positions = {
        BASE_TO_SLIDER_MAX - 60,    //- HALF_SLIDER_WIDTH,
        BASE_TO_SLIDER_MAX - 26.79, // - HALF_SLIDER_WIDTH,
        BASE_TO_SLIDER_MAX - 23.48, //- HALF_SLIDER_WIDTH,
        BASE_TO_SLIDER_MAX - 60,    //- HALF_SLIDER_WIDTH,
        0};

    robot_controller rc(home_positions);
    // Test 10 rounds of axis A non-backlash movement
    slider_positions new_positions = home_positions;
    new_positions.right_slider_y -= 2.5;
    for (int i = 0; i < 10; i++) {
        rc.move_no_compensation(new_positions);
        std::cout << "collect data" << std::endl;
        std::cin >> input;
        rc.move(home_positions);
    }
    // Test 10 rounds of axis A backlash movement
    // Test 10 rounds of axis A backlash compensated movement
    // Test 10 rounds of axis A non-backlash integrated movement
}
