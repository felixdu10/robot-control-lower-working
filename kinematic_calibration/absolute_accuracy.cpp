#include "jacobian.h"
#include "helpers.h"
#include "atracsys_functions.h"
#include "inverse_kinematics.h"
#include "kinematics.h"
#include "robot_controller.h"
#include "templated_classes/Templated_Point.h"
#include "templated_classes/Templated_Transform.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

static bool backlash_compensation_on = true;
using std::vector;
int main() {

  constexpr Parameters<double> params = get_default_parameters<double>();
  RobotController robot(backlash_compensation_on);
  AtracsysTracker<double> atracsys("geometry100000.ini", "geometry999.ini");
  vector<Transform<double>> targets;
  vector<Thetas<double>> default_thetas;
  vector<Thetas<double>> adjusted_thetas;
  int k = 4;
  for (int j = -30; j <= 30; j += 60) {
    for (int i = -40; i <= 40; i += 80) {
      for (int k = 0; k < 4; k++) {
        double theta = 0;
        double phi = 0;
        switch (k) {
        case 2:
          break;
        default:
          theta = M_PI / 2 * k;
          phi = 25 * M_PI / 180.0;
          break;
        }
        Transform<double> target_transform(phi, 0, theta, i, 405 + j, -115);
        try {
          Thetas<double> thetas = get_thetas(target_transform);
          default_thetas.push_back(thetas);
        } catch (const std::runtime_error &e) {
        }
      }
    }
  }
  Transform<double> home_transform(0, 0, 0, 0, 405, -115);
  // default_thetas = sort_min_travel(default_thetas);
  std::vector<Thetas<double>> new_default_thetas;
  // robot.home();
  int I = 0;

  std::ofstream logfile("absolute_accuracy.csv");
  for (Thetas<double> default_theta : default_thetas) {
    try {
      robot.move(default_theta, atracsys);
      targets.push_back(get_end_effector(default_theta));
      adjusted_thetas.push_back(robot.cur_thetas);
      new_default_thetas.push_back(default_thetas[I]);
      adjusted_thetas[I].print();

      Transform<double> expected_F_EE = get_end_effector(new_default_thetas[I]);
      Measurement<double> atracsys_measurement;
      while (atracsys.getMeasurement(BOTH, atracsys_measurement) < 0) {
        std::this_thread::sleep_for(750ms);
      };
      Transform<double> actual_F_EE =
          F_M1R<double>().inverse() * atracsys_measurement.F_OM1.inverse() *
          atracsys_measurement.F_OM2 * F_M2N<double>(0, params) *
          F_NEE(new_default_thetas[I], params);
      double targeting_error = (actual_F_EE.p - expected_F_EE.p).magnitude();
      if (targeting_error > 50) {
        atracsys_measurement.F_OM1 =
            atracsys_measurement.F_OM1 * Transform<double>(0, 0, M_PI, 0, 0, 0);
        actual_F_EE = F_M1R<double>().inverse() *
                      atracsys_measurement.F_OM1.inverse() *
                      atracsys_measurement.F_OM2 * F_M2N<double>(0, params) *
                      F_NEE(new_default_thetas[I], params);
        targeting_error = (actual_F_EE.p - expected_F_EE.p).magnitude();
      }
      Point<double> expected_z = {expected_F_EE.R.matrix[0][2],
                                  expected_F_EE.R.matrix[1][2],
                                  expected_F_EE.R.matrix[2][2]};
      Point<double> actual_z = {actual_F_EE.R.matrix[0][2],
                                actual_F_EE.R.matrix[1][2],
                                actual_F_EE.R.matrix[2][2]};
      double angular_error = 180 / M_PI * std::acos(expected_z * actual_z);

      // write all to file
      write_to_log_file(logfile, atracsys_measurement.F_OM1,
                        atracsys_measurement.F_OM2, expected_F_EE, actual_F_EE,
                        targeting_error, angular_error, I);

      robot.move(get_thetas(home_transform));
      robot.move(new_default_thetas[I], atracsys);
      robot.cur_thetas.print();
      std::cout << "position " << I + 1 << "/" << default_thetas.size()
                << std::endl;
    } catch (std::runtime_error &e) {
    }
    I++;
  }
  int index = 0;
  for (int N = 0; N < 20; N++) {
    for (int i = 0; i < new_default_thetas.size(); i++) {
      robot.move(adjusted_thetas[i]);
      robot.cur_thetas.print();
      std::string s;
      std::cin >> s;
      Transform<double> expected_F_EE = get_end_effector(new_default_thetas[i]);
      Measurement<double> atracsys_measurement;
      while (atracsys.getMeasurement(BOTH, atracsys_measurement) < 0) {
        std::this_thread::sleep_for(750ms);
      };
      // record errors

      Transform<double> actual_F_EE =
          F_M1R<double>().inverse() * atracsys_measurement.F_OM1.inverse() *
          atracsys_measurement.F_OM2 * F_M2N<double>(0, params) *
          F_NEE(new_default_thetas[i], params);
      double targeting_error = (actual_F_EE.p - expected_F_EE.p).magnitude();
      if (targeting_error > 50) {
        atracsys_measurement.F_OM1 =
            atracsys_measurement.F_OM1 * Transform<double>(0, 0, M_PI, 0, 0, 0);
        actual_F_EE = F_M1R<double>().inverse() *
                      atracsys_measurement.F_OM1.inverse() *
                      atracsys_measurement.F_OM2 * F_M2N<double>(0, params) *
                      F_NEE(new_default_thetas[i], params);
        targeting_error = (actual_F_EE.p - expected_F_EE.p).magnitude();
      }
      Point<double> expected_z = {expected_F_EE.R.matrix[0][2],
                                  expected_F_EE.R.matrix[1][2],
                                  expected_F_EE.R.matrix[2][2]};
      Point<double> actual_z = {actual_F_EE.R.matrix[0][2],
                                actual_F_EE.R.matrix[1][2],
                                actual_F_EE.R.matrix[2][2]};
      double angular_error = 180 / M_PI * std::acos(expected_z * actual_z);

      // write all to file
      write_to_log_file(logfile, atracsys_measurement.F_OM1,
                        atracsys_measurement.F_OM2, expected_F_EE, actual_F_EE,
                        targeting_error, angular_error, index);
      index++;
      // end record errors
    }
  }
}
