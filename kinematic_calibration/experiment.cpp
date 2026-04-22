#include "atracsys_functions.h"
#include "kinematics.h"
#include "templated_classes/Templated_Transform.h"
#include <vector>
namespace External {
#include "../inverse_kinematics.cpp"
#include "../robot_controller.h"
} // namespace External

const int num_measurements_per_pose = 10;

int get_positions(std::vector<External::slider_positions> &positions) {
  External::Robot robot;
  int X_WIDTH = 100;
  int Y_HEIGHT = 500;
  int Y_MIN = 300;
  for (float x = -X_WIDTH; x <= X_WIDTH; x += 10) {
    for (float y = Y_MIN; y < Y_HEIGHT; y += 10) {
      for (int i = 0; i < 5; i++) {
        float theta;
        float phi = 0;
        if (i == 0) {
          theta = 0;
        } else {
          theta = M_PI / 4;
          phi += M_PI / 2;
        }
        External::approach_definition def = {{x, y, -100}, theta, phi};
        try {
          External::slider_positions sliders = External::inverse_kinematics(
              def, External::NewTransform(0, 0, 0, 0, 0, 0), robot);
          positions.push_back(sliders);
        } catch (std::runtime_error e) {
        }
      }
    }
  }
  return positions.size();
}

static const External::slider_positions home_positions = {
    External::BASE_TO_SLIDER_MAX - 60,    //- HALF_SLIDER_WIDTH,
    External::BASE_TO_SLIDER_MAX - 26.79, // - HALF_SLIDER_WIDTH,
    External::BASE_TO_SLIDER_MAX - 23.48, //- HALF_SLIDER_WIDTH,
    External::BASE_TO_SLIDER_MAX - 60,    //- HALF_SLIDER_WIDTH,
    0};
static const Transform<double> F_MR(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
static const Transform<double> F_MN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
int main() {
  std::vector<External::slider_positions> positions;
  int num_positions = get_positions(positions);
  External::robot_controller rc(home_positions);
  std::vector<Measurement<double>> measurement_array;
  for (External::slider_positions position : positions) {
    // go to position
    rc.move(position);
    // read values
	std::vector<Measurement<double>> averaging_vec;
    for (int measurement_idx = 0; measurement_idx < num_measurements_per_pose;
         measurement_idx++) {
       averaging_vec.push_back(get_atracsys_measurement(BOTH));
			
    }
    // write values to file
  }

  get_best_params(measurement_array, positions, F_MR, F_MN);
}
