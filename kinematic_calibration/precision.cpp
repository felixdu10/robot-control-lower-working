#include <cmath>
#include <fstream>
#include "helpers.h"
#include "kinematics.h"
#include "inverse_kinematics.h"
#include "atracsys_functions.h"
#include "robot_controller.h"

const bool backlash_compensation = true;

int main() {
  Parameters<double> params = get_default_parameters<double>();
  std::ofstream logfile("precision_new_marker_comparison_continued2.csv");
  open_log_file(logfile);
  //AtracsysTracker<double> atracsys("geometry100000.ini", "geometry999.ini");
  Measurement<double> atracsys_measurement;
  RobotController robot(backlash_compensation);

  std::vector<Thetas<double>> positions;
  Transform<double> target_transform;
  Transform<double> home_transform;
  Transform<double> expected_F_EE;
  frame_errors *errors_1 = new frame_errors;
  frame_errors *errors_2 = new frame_errors;
  home_transform = Transform<double>(0, 0, 0, 0, 405, -115);
  std::vector<Transform<double>> targets;
  targets.push_back(home_transform);
  targets.push_back(Transform<double>(0, 0, 0, 40, 405, -115));
  targets.push_back(Transform<double>(0, 0, 0, -40, 405, -115));
  targets.push_back(Transform<double>(0, 0, 0, 0, 405 + 30, -115));
  targets.push_back(Transform<double>(0, 0, 0, 5, 405 - 30, -115));
  targets.push_back(Transform<double>(0, 0, 0, 0, 425, -115));
  targets.push_back(Transform<double>(25.0 * M_PI / 180.0, 0, 0, 0, 420, -10));
  targets.push_back(
      Transform<double>(25.0 * M_PI / 180.0, 0, M_PI / 2.0, 0, 420, -10));
  targets.push_back(
      Transform<double>(25.0 * M_PI / 180.0, 0, M_PI, 0, 420, -10));
  targets.push_back(
      Transform<double>(25.0 * M_PI / 180.0, 0, 3.0 * M_PI / 2.0, 0, 420, -10));
  int pos = 1;
  for (Transform<double> transform : targets) {
    std::cout << "position: " << pos++ << std::endl;
    get_thetas(transform);
  }
  // robot.home();
  for (int i = 0; i < 10; i++) {
    for (Transform<double> transform : targets) {
      //        robot.home();
      expected_F_EE = get_end_effector(home_positions);
      //        while (atracsys.getMeasurement(BOTH, atracsys_measurement,
      //        errors_1,
      //                                       errors_2) < 0) {
      //          std::this_thread::sleep_for(750ms);
      //        };
      //        get_errors(atracsys_measurement, home_positions, params,
      //        expected_F_EE, i,
      //                   logfile);
      //
      Thetas<double> thetas = get_thetas(transform);
      robot.move(thetas);
      std::cout << "at cycle " << i << std::endl;
      targets[i].print();
      expected_F_EE = get_end_effector(thetas);
      //while (atracsys.getMeasurement(BOTH, atracsys_measurement, errors_1,
      //                               errors_2) < 0) {
      //  std::this_thread::sleep_for(750ms);
      //};
      get_errors(atracsys_measurement, thetas, params, expected_F_EE, i,
                 logfile);
      std::cout << "marker 1 translation error: " << errors_1->translation
                << ", angular error: " << errors_1->angular << std::endl;
      std::cout << "marker 2 translation error: " << errors_2->translation
                << ", angular error: " << errors_2->angular << std::endl;
    }
  }
  delete errors_1;
  delete errors_2;
}
