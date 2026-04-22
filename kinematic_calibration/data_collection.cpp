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

template <typename T>
double theta_distance(const Thetas<T> &a, const Thetas<T> &b) {
  return std::sqrt(
      std::pow(a.theta_1 - b.theta_1, 2) + std::pow(a.theta_2 - b.theta_2, 2) +
      std::pow(a.theta_3 - b.theta_3, 2) + std::pow(a.theta_4 - b.theta_4, 2));
}

template <typename T>
std::vector<Thetas<T>>
sort_min_travel(const std::vector<Thetas<T>> &positions) {
  if (positions.empty())
    return {};

  std::vector<Thetas<T>> sorted;
  sorted.reserve(positions.size());

  std::vector<bool> visited(positions.size(), false);
  int current = 0; // start arbitrarily at index 0
  sorted.push_back(positions[current]);
  visited[current] = true;

  for (size_t step = 1; step < positions.size(); ++step) {
    double best_dist = std::numeric_limits<double>::max();
    int best_idx = -1;

    for (size_t i = 0; i < positions.size(); ++i) {
      if (visited[i])
        continue;
      double d = theta_distance(positions[current], positions[i]);
      if (d < best_dist) {
        best_dist = d;
        best_idx = i;
      }
    }

    if (best_idx == -1)
      break;
    sorted.push_back(positions[best_idx]);
    visited[best_idx] = true;
    current = best_idx;
  }

  return sorted;
}
void compare_params(Parameters<double> actual, Parameters<double> optimized,
                    const int ct) {
  double *arr_1 = new double[ct];
  int count = Parameters_to_array<double>(actual, arr_1);
  assert(count == ct);
  double *arr_2 = new double[ct];
  Parameters_to_array<double>(optimized, arr_2);
  double mse_tot = 0;
  for (int i = 0; i < count; i++) {
    double diff = arr_1[i] - arr_2[i];
    std::cout << "difference in parameter " << i << " = " << diff << std::endl;
    mse_tot += diff * diff;
  }
  mse_tot /= count;
  std::cout << "Mean squared error = " << mse_tot << std::endl;
  delete[] arr_2;
  delete[] arr_1;
}
// Transform<double> F_M1R(0, 0, 0, 0, -5.5, -(67 + 43.5));
// Transform<double> F_M2N(0, 0, M_PI, .00558085, 28.3789, -5.49509);
Transform<double> F_NEE(const Thetas<double> &thetas,
                        const Parameters<double> &params) {
  Point P1 = get_upper_linkage_P(thetas, params);
  Point P2 = get_lower_linkage_P(thetas, params);
  Point<double> z = (P1 - P2).normalize();
  Point<double> x =
      cross(get_upper_linkage_n_vec(thetas, params), z).normalize();
  Point<double> y = cross(z, x);
  Transform<double> N_Frame(x, y, z, P2);
  Transform<double> EE_frame = get_end_effector(thetas, params);
  return N_Frame.inverse() * EE_frame;
}
Transform<double> F_NEE(const Thetas<double> &thetas) {
  return F_NEE(thetas, get_default_parameters<double>());
};

template <typename T>
double total_theta_distance(const std::vector<Thetas<T>> &path) {
  if (path.size() < 2)
    return 0.0;
  double total = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    total += std::sqrt(std::pow(path[i].theta_1 - path[i + 1].theta_1, 2) +
                       std::pow(path[i].theta_2 - path[i + 1].theta_2, 2) +
                       std::pow(path[i].theta_3 - path[i + 1].theta_3, 2) +
                       std::pow(path[i].theta_4 - path[i + 1].theta_4, 2));
  }
  return total;
}

int main() {
  constexpr Parameters<double> params = get_default_parameters<double>();
  constexpr int ct = get_num_tunable_params(params);
  std::ofstream logfile("targetting_final.csv");
  open_log_file(logfile);
  // get F_M1R
  // get F_M2EE
  // backlash flag
  bool backlash_compensation = true;
  // targeting positions in vector
  std::vector<Thetas<double>> positions;
  for (int k = 0; k < 5; k++) {
    for (int j = -30; j <= 30; j += 30) {
      for (int i = -40; i <= 40; i += 20) {
        // polar coordinate representation
        double theta, phi = 0;
        switch (k) {
        case 4:
          theta = 0;
          phi = 0;
          break;
        default:
          theta = M_PI / 2 * k;
          phi = 25 * M_PI / 180.0;
          break;
        }
        try {
          Transform<double> target_transform(phi, 0, theta, i, 405 + j, -115);
          Thetas<double> opt = get_thetas(target_transform);
          positions.push_back(opt);
          std::cout << "thetas i j k: " << i << " " << j << " " << k
                    << std::endl;
        } catch (const std::runtime_error &e) {
        }
      }
    }
  }

  std::cout << total_theta_distance(positions) << std::endl;
  positions = sort_min_travel(positions);
  std::cout << total_theta_distance(positions) << std::endl;
  AtracsysTracker<double> atracsys("geometry100000.ini", "geometry999.ini");
  RobotController robot(backlash_compensation);
  //  robot.home();
  // enter for loop:
  int index = 0;
  std::vector<Measurement<double>> validation_measurements;
  std::vector<Thetas<double>> validation_positions;
  std::vector<Measurement<double>> measurements;
  std::vector<Measurement<double>> record_measurements;
  std::vector<Thetas<double>> true_positions;
  std::vector<Thetas<double>> adjustments;
  Point<double> prev = {0, 0, 0};
  Point<double> zero = {(-0.0308035), (28.3232666667), (-9.07299)};
  double Dist = 0;
  std::vector<double> dists;
  for (Thetas<double> thetas : positions) {
    thetas.print();
    // go to targetted position based on inverse kinematics
    // get encoder position error to account for it
    std::cout << "starting motion" << std::endl;
    Thetas<double> errs = robot.move(thetas, atracsys);
    // encoder_error_struct errs = robot.move(thetas);
    adjustments.push_back(errs);
    std::cout << "ending motion" << std::endl;
    // calculate expected F_EE
    Thetas<double> encoder_adjusted_thetas = thetas;
    encoder_adjusted_thetas.theta_1 += errs.theta_1;
    encoder_adjusted_thetas.theta_2 += errs.theta_2;
    encoder_adjusted_thetas.theta_3 += errs.theta_3;
    encoder_adjusted_thetas.theta_4 += errs.theta_4;

    // encoder_adjusted_thetas.theta_1 += errs.mmErrLeft;
    // encoder_adjusted_thetas.theta_2 += errs.mmErrRight;
    // encoder_adjusted_thetas.theta_3 += errs.mmErrLeftMiddle;
    // encoder_adjusted_thetas.theta_4 += errs.mmErrRightMiddle;
    Transform<double> expected_F_EE = get_end_effector(thetas);
    // Transform<double> expected_F_EE =
    // get_end_effector(encoder_adjusted_thetas);
    //   get measurement
    Measurement<double> atracsys_measurement;
    Point<double> cur;
    while (atracsys.getMeasurement(BOTH, atracsys_measurement) < 0) {
      std::this_thread::sleep_for(750ms);
    };
    cur = atracsys_measurement.F_OM2 * zero;
    prev = cur;
    std::mt19937_64 rng;
    uint64_t timeSeed =
        std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
    rng.seed(ss);
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    double randomNumber = dist(rng);
    if (randomNumber < .25) {
      validation_measurements.push_back(atracsys_measurement);
      validation_positions.push_back(encoder_adjusted_thetas);
      record_measurements.push_back(atracsys_measurement);
    } else {
      measurements.push_back(atracsys_measurement);

      // true_positions.push_back(thetas);
      true_positions.push_back(encoder_adjusted_thetas);
      record_measurements.push_back(atracsys_measurement);
    }
    // get positional error of measurement
    // get angular error of needle and positional error of needle
    Transform<double> actual_F_EE =
        F_M1R<double>().inverse() * atracsys_measurement.F_OM1.inverse() *
        atracsys_measurement.F_OM2 * F_M2N<double>(0, params) *
        F_NEE(thetas, params);
    double targeting_error = (actual_F_EE.p - expected_F_EE.p).magnitude();
    if (targeting_error > 50) {
      atracsys_measurement.F_OM1 =
          atracsys_measurement.F_OM1 * Transform<double>(0, 0, M_PI, 0, 0, 0);
      actual_F_EE = F_M1R<double>().inverse() *
                    atracsys_measurement.F_OM1.inverse() *
                    atracsys_measurement.F_OM2 * F_M2N<double>(0, params) *
                    F_NEE(thetas, params);
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
    write_to_log_file(logfile, record_measurements[index].F_OM1,
                      record_measurements[index].F_OM2, expected_F_EE,
                      actual_F_EE, targeting_error, angular_error, index);
    index++;
    std::cout << "position " << index << "/" << positions.size() << std::endl;
  } // exit loop
  // std::cout << "begginning calibration: " << measurements.size() << " "
  //           << true_positions.size() << std::endl;

  // Parameters<double> new_params = ceres_solve_with_validation<ct>(
  //     measurements, true_positions, validation_measurements,
  //     validation_positions, params);

  // compare_params(params, new_params, ct);
  // for (int i = 0; i < 4; i++) {
  //   std::cout
  //       << new_params.tunable_params.loop_parameters[i].y_slider_offset.value
  //       << std::endl;
  // }
  // std::cout << new_params.tunable_params.upper_base_z_offset.value << " "
  //           << new_params.tunable_params.lower_base_z_offset.value <<
  //           std::endl;
  for (int i = 0; i < adjustments.size(); i++) {
    adjustments[i].print();
  }
  for (double dist : dists) {
    std::cout << dist << std::endl;
  }
}
