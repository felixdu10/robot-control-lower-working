#ifndef JACOBIAN
#define JACOBIAN
#include "ceres/ceres.h"
#include "kinematics.h"
#include "templated_classes/Templated_Transform.h"
#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/types.h>
#include <strings.h>
#include <vector>

/**
 * @brief Get the F_M1R base transform
 *
 * @tparam T
 * @return The transform of the baseplate to the robot frame
 */
template <typename T> Transform<T> F_M1R() {
  // return Transform<T>(T(0), T(0), T(0), T(0), T(0), T(0));
  return Transform<T>(T(0), T(0), T(0), T(0), T(-5.5), T(-(67 + 43.5)));
};

/**
 * @brief Get the F_M2N transform, which returns the needle to marker transform,
 * allowing pose measurement
 *
 * @tparam T
 * @param theta_5
 * @param parameters
 * @return
 */
template <typename T> Transform<T> F_M2N(T theta_5, Parameters<T> parameters) {
  // smaller holder transform
  Transform<T> pivot(T(0), T(0), T(M_PI),
                     T(parameters.tunable_params.pivot_x.value),
                     T(parameters.tunable_params.pivot_y.value),
                     T(parameters.tunable_params.pivot_z.value));
  // this is big holder transform
  // Transform<T> pivot(T(0), T(0), T(M_PI), T(-0.0308035), T(28.3232666667),
  //                    T(-9.07299));
  //  Replace 0s pivot calibration
  //  Transform<T> pivot;
  //  return pivot *
  //        Transform<T>(
  //            T(0), T(0), T(0), T(0),
  //            parameters.tunable_params.bottom_needle_to_holder_distance.value,
  //            parameters.tunable_params.needle_offset.value + theta_5);
  return pivot;
};

template <typename T> Transform<T> F_M2N() {
  // smaller holder transform
  Transform<T> pivot(T(0), T(0), T(M_PI), T(.00558085), T(28.3789),
                     T(-5.49509));
  return pivot;
};
/**
 * @brief Get the loop closure error of a position based on a measurement.
 *
 * @tparam T
 * @param thetas The position of the robot's sliders
 * @param parameters The kinematic parameters of the robot
 * @param measurement The measurement of the position of the markers
 * @param loop the kinematic loop we use to determine error
 * @return The error magnitude
 */
template <typename T>
T error(Thetas<T> thetas, Parameters<T> parameters, Measurement<T> measurement,
        linkage_values loop) {
  Point<T> zero = {T(0), T(0), T(0)};
  F_M1R<T>();
  F_M2N(thetas.theta_5, parameters);
  //  Point<T> N_obs = F_M1R<T>().inverse() * measurement.F_OM1.inverse() *
  //                  measurement.F_OM2 * F_M2N(thetas.theta_5, parameters) *
  //                  zero;
  //  Point<T> N_obs = measurement.F_OM2.p;
  // Point<T> N_calc = get_end_effector(thetas, parameters).p;
  Point<T> C_i = get_linkage_C(thetas, parameters, loop);
  Point<T> B_i = get_linkage_B(thetas, parameters, loop);
  Transform<T> f_M2N = F_M2N<T>(T(0), parameters);
  Transform<T> F_RM2 = (get_end_effector(thetas, parameters) *
                        F_NEE(thetas, parameters).inverse() * f_M2N.inverse());
  // Point<T> P_calc = get_lower_linkage_P(thetas, parameters);
  Transform<T> f_RM2_obs =
      (F_M1R<T>().inverse() * measurement.F_OM1.inverse() * measurement.F_OM2);
  Point<T> Needle = {
      T(0), parameters.tunable_params.bottom_needle_to_holder_distance.value,
      T(-115)};
  T c_i =
      parameters.tunable_params.loop_parameters[loop].distal_link_length.value;
  // return (N_obs - N_calc + C_i - B_i).magnitude() - c_i;
  return (f_RM2_obs * Needle - F_RM2 * Needle + C_i - B_i).magnitude() - c_i;
}

/**
 * @brief Turn an atracsys measurement into a templated class
 *
 * @tparam T
 * @param m the measurement
 * @return the templated measurement
 */
template <typename T> Measurement<T> measurement_to_T(Measurement<double> &m) {
  Measurement<T> ret;
  ret.F_OM1 = T(m.F_OM1.p);
}

/**
 * @struct Error_Residual
 * @brief This error residual struct is used to calculate residuals in order to
 * conduct the kinematic calibration.
 *
 */
struct Error_Residual {
  Measurement<double> measurement;
  Thetas<double> thetas;
  linkage_values val;
  int index;
  Error_Residual(Measurement<double> m, Thetas<double> t, linkage_values v,
                 int i)
      : measurement(m), thetas(t), val(v), index(i) {}

  template <typename T>
  bool operator()(const T *const params, T *residual) const {
    Parameters<T> paramaters = array_to_Parameters<T>(params);
    Measurement<T> measurement_T = measurement.template convert<T>();
    Thetas<T> thetas_T = thetas.template convert<T>();
    T e = error(thetas_T, paramaters, measurement_T, val);
    residual[0] = e;
    return true;
  }
};

template <int ct>
Parameters<double> ceres_solve(std::vector<Measurement<double>> measurements,
                               std::vector<Thetas<double>> thetas,
                               Parameters<double> guess) {
  ceres::Problem problem;
  double *param_arr = new double[ct];
  int CT = Parameters_to_array(guess, param_arr);
  assert(CT == ct);
  for (int i = 0; i < measurements.size(); i++) {
    for (int j = 0; j < num_loops; j++) {
      // minimize tunable parameters wrt error for each loop for each
      ceres::CostFunction *cf =
          new ceres::AutoDiffCostFunction<Error_Residual, 1, ct>(
              new Error_Residual(measurements[i], thetas[i],
                                 static_cast<linkage_values>(j), i));
      problem.AddResidualBlock(cf, nullptr, param_arr);
    }
  }
  for (int k = 0; k < ct; ++k) {
    problem.SetParameterLowerBound(param_arr, k, param_arr[k] - 5.0);
    problem.SetParameterUpperBound(param_arr, k, param_arr[k] + 5.0);
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 20;
  options.max_num_iterations = 500;
  options.dense_linear_algebra_library_type = ceres::CUDA;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  std::cout << "Ceres Initial cost: " << summary.initial_cost
            << ", Final cost: " << summary.final_cost << "\n";
  Parameters<double> optimized = array_to_Parameters(param_arr);
  delete[] param_arr;
  return optimized;
}
double get_error_cost(const std::vector<Measurement<double>> &measurements,
                      const std::vector<Thetas<double>> &thetas,
                      const Parameters<double> guess) {
  double mse = 0;
  for (int i = 0; i < measurements.size(); ++i) {
    for (int j = 0; j < num_loops; ++j) {
      double err = error<double>(thetas[i], guess, measurements[i],
                                 static_cast<linkage_values>(j));
      mse += err * err;
    }
  }
  return mse / 2;
}
template <int ct>
Parameters<double> ceres_solve_with_validation(
    const std::vector<Measurement<double>> &measurements,
    const std::vector<Thetas<double>> &thetas,
    const std::vector<Measurement<double>> &validation_measurements,
    const std::vector<Thetas<double>> &validation_thetas,
    const Parameters<double> &guess) {

  ceres::Problem problem;
  double *param_arr = new double[ct];
  int CT = Parameters_to_array(guess, param_arr);
  assert(CT == ct);

  // Add residuals
  for (int i = 0; i < (int)measurements.size(); ++i) {
    for (int j = 0; j < num_loops; ++j) {
      ceres::CostFunction *cf =
          new ceres::AutoDiffCostFunction<Error_Residual, 1, ct>(
              new Error_Residual(measurements[i], thetas[i],
                                 static_cast<linkage_values>(j), i));
      problem.AddResidualBlock(cf, nullptr, param_arr);
    }
  }

  // Set bounds
  for (int k = 0; k < ct; ++k) {
    problem.SetParameterLowerBound(param_arr, k, param_arr[k] - 5.0);
    problem.SetParameterUpperBound(param_arr, k, param_arr[k] + 5.0);
  }

  // Solver options
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.dense_linear_algebra_library_type = ceres::CUDA;
  options.num_threads = 20;
  options.max_num_iterations = 1; // only 1 per Solve() call
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;

  // --- Early stopping parameters ---
  const int max_total_iters = 500;
  const int patience = 15; // stop if no improvement for 15 iterations
  const double tol = 1e-6; // minimal improvement threshold
  int no_improve = 0;
  double best_val_error = std::numeric_limits<double>::infinity();
  Parameters<double> best_params = guess;

  for (int iter = 0; iter < max_total_iters; ++iter) {
    ceres::Solve(options, &problem, &summary);

    // Compute validation MSE
    Parameters<double> current_params = array_to_Parameters(param_arr);
    double val_mse = get_error_cost(validation_measurements, validation_thetas,
                                    current_params);

    std::cout << "Iter " << iter << " | Train cost: " << summary.final_cost
              << " | Validation MSE: " << val_mse << std::endl;

    // Check for improvement
    if (val_mse + tol < best_val_error) {
      best_val_error = val_mse;
      best_params = current_params;
      no_improve = 0;
    } else if (++no_improve >= patience) {
      std::cout << "Early stopping triggered after " << iter
                << " iterations. Best validation MSE = " << best_val_error
                << "\n";
      break;
    }
  }

  std::cout << "Ceres Initial cost: " << summary.initial_cost
            << ", Final cost: " << summary.final_cost << "\n";

  delete[] param_arr;
  return best_params;
}

#endif // JACOBIAN
