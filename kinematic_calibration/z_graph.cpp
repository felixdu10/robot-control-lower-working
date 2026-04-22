#include "jacobian.h"
#include "helpers.h"
#include "atracsys_functions.h"
#include "inverse_kinematics.h"
#include "kinematics.h"
#include "robot_controller_ros.h"
#include "templated_classes/Templated_Point.h"
#include "templated_classes/Templated_Transform.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

using std::vector;
int main() {
  // we want to go to many positions in the workspace and record the z
  // coordinates wrt x and y coordinates. this code will go to xy positions,
  // keep the end effector upright, and determine the x y and z positions at
  // that spot.

  constexpr Parameters<double> params = get_default_parameters<double>();
  // first, determine the target locations
  float sagittal_range = 40;
  float axial_range = 30;
  vector<Thetas<double>> default_thetas;
  vector<Thetas<double>> adjusted_thetas;

  for (int i = -sagittal_range; i <= sagittal_range; i += 5) {
    for (int j = -axial_range; j <= axial_range; j += 5) {
      Transform<double> target_transform(0, 0, 0, i, 405 + j, -115);
      try {
        Thetas<double> thetas = get_thetas(target_transform);
        default_thetas.push_back(thetas);
      } catch (const std::runtime_error &e) {
      }
    }
  }

  default_thetas = sort_min_travel(default_thetas);
  AtracsysTracker<double> atracsys("geometry100000.ini", "geometry999.ini");
  RobotControllerROS robot(true, get_default_parameters<double>());
  Measurement<double> atracsys_measurement;
  // move to each of those positions, may need to write a new move function
  for (Thetas<double> position : default_thetas) {

    robot.move(position);
    while (atracsys.getMeasurement(BOTH, atracsys_measurement) < 0) {
      std::this_thread::sleep_for(750ms);
    };
    Transform<double> actual_F_RN =
        F_M1R<double>().inverse() * atracsys_measurement.F_OM1.inverse() *
        atracsys_measurement.F_OM2 * F_M2N<double>(0, params);
    // record xy and z
    appendPointToCSV(actual_F_RN.p, "points");
  }
}
