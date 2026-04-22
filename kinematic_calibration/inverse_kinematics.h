#include "auto.h"
#include "kinematics.h"
#include "templated_classes/Templated_Point.h"
#include <stdexcept>
namespace External {
#include "../inverse_kinematics.cpp"
}

template <typename T>
Thetas<Auto<5, T>> to_thetas(External::slider_positions s) {

  Thetas<Auto<5, T>> t;
  t.theta_1 = Auto<5, T>(s.left_slider_y);
  t.theta_1.epsilon[0] = 1;
  t.theta_2 = Auto<5, T>(s.right_slider_y);
  t.theta_2.epsilon[1] = 1;
  t.theta_3 = Auto<5, T>(s.left_middle_slider_y);
  t.theta_3.epsilon[2] = 1;
  t.theta_4 = Auto<5, T>(s.right_middle_slider_y);
  t.theta_4.epsilon[3] = 1;
  t.theta_5 = Auto<5, T>(s.needle_extension);
  t.theta_5.epsilon[4] = 1;
  return t;
}
template <typename T>
Thetas<T> get_thetas(Transform<T> EE_transform, Parameters<T> parameters) {

  Transform<Auto<5, T>> differentiable_EE_transform;
  differentiable_EE_transform.p = EE_transform.p.template convert<Auto<5, T>>();
  differentiable_EE_transform.R = EE_transform.R.template convert<Auto<5, T>>();
  int num_params = get_num_tunable_params(parameters);
  T *array = new T[num_params];
  Parameters_to_array(parameters, array);
  Parameters<Auto<5, T>> differentiable_params =
      get_default_parameters<Auto<5, T>>();
  Auto<5, T> *differentiable_arr = new Auto<5, T>[num_params];
  Parameters_to_array<Auto<5, T>>(differentiable_params, differentiable_arr);
  for (int i = 0; i < num_params; ++i) {
    differentiable_arr[i] = Auto<5, T>(array[i]);
  }
  differentiable_params = array_to_Parameters(differentiable_arr);
  delete[] array;
  delete[] differentiable_arr;
  External::Robot inverse_robot;
  External::target_and_injection_point_approach approach = {
      {EE_transform.p.x, EE_transform.p.y, EE_transform.p.z},
      {EE_transform.p.x + EE_transform.R.matrix[0][2],
       EE_transform.p.y + EE_transform.R.matrix[1][2],
       EE_transform.p.z + EE_transform.R.matrix[2][2]}};
  External::slider_positions guess_pos =
      External::inverse_kinematics(approach, inverse_robot);
  Thetas<Auto<5, T>> guess = to_thetas<T>(guess_pos);
  guess.theta_1 -= differentiable_params.tunable_params.loop_parameters[0]
                       .y_slider_offset.value;
  guess.theta_2 -= differentiable_params.tunable_params.loop_parameters[1]
                       .y_slider_offset.value;
  guess.theta_3 -= differentiable_params.tunable_params.loop_parameters[2]
                       .y_slider_offset.value;
  guess.theta_4 -= differentiable_params.tunable_params.loop_parameters[3]
                       .y_slider_offset.value;
  Transform<Auto<5, T>> cur_T;
  Auto<5, T> loss(1);
  int iter = 0;

  while (loss > .0001) {
    cur_T = get_end_effector<Auto<5, T>>(guess, differentiable_params);
    Point<Auto<5, T>> diff_Z = {differentiable_EE_transform.R.matrix[0][2],
                                differentiable_EE_transform.R.matrix[1][2],
                                differentiable_EE_transform.R.matrix[2][2]};
    Point<Auto<5, T>> cur_Z = {cur_T.R.matrix[0][2], cur_T.R.matrix[1][2],
                               cur_T.R.matrix[2][2]};
    Point<Auto<5, T>> translational_diff =
        (cur_T.p - differentiable_EE_transform.p);
    Point<Auto<5, T>> rot_diff = (diff_Z - cur_Z);
    Auto<5, T> cos_sim = acos(diff_Z * cur_Z);
    loss = translational_diff.x * translational_diff.x +
           translational_diff.y * translational_diff.y +
           translational_diff.z * translational_diff.z + cos_sim;
    if (iter == 0) {
      // std::cout << "loss on inverse_kinematics: " << loss.val
      //           << " on iter: " << iter << std::endl;
    }
    T grad_magnitude = sqrt(
        loss.epsilon[0] * loss.epsilon[0] + loss.epsilon[1] * loss.epsilon[1] +
        loss.epsilon[2] * loss.epsilon[2] + loss.epsilon[3] * loss.epsilon[3] +
        loss.epsilon[4] * loss.epsilon[4]);
    T learning_rate = 0.01 / (1.0 + 0.01 * iter);
    if (loss.val >= .0001 && grad_magnitude > 1e-10) {
      guess.theta_1.val -= learning_rate * loss.epsilon[0] / grad_magnitude;
      guess.theta_2.val -= learning_rate * loss.epsilon[1] / grad_magnitude;
      guess.theta_3.val -= learning_rate * loss.epsilon[2] / grad_magnitude;
      guess.theta_4.val -= learning_rate * loss.epsilon[3] / grad_magnitude;
      guess.theta_5.val -= learning_rate * loss.epsilon[4] / grad_magnitude;
    }
    //   std::cout << "loss epsilon: " << loss.epsilon[0] << " " <<
    //   loss.epsilon[1]
    //             << " " << loss.epsilon[2] << " " << loss.epsilon[3] << " "
    //             << loss.epsilon[4] << " " << std::endl;
    iter++;
    if (iter == 5000) {

      // std::cout << "loss on inverse_kinematics: " << loss.val
      //           << " on iter: " << iter << std::endl;
      loss = 0;
    }
  }
  Thetas<T> final_guess;
  final_guess.theta_1 = guess.theta_1.val;
  final_guess.theta_2 = guess.theta_2.val;
  final_guess.theta_3 = guess.theta_3.val;
  final_guess.theta_4 = guess.theta_4.val;
  final_guess.theta_5 = guess.theta_5.val;
  if (final_guess.theta_1 < 87 || final_guess.theta_2 < 87 ||
      final_guess.theta_3 < 95 || final_guess.theta_4 < 95) {
    throw std::runtime_error("thetas are too close to robot");
  }
  //std::cout << "loss on inverse_kinematics: " << loss.val
  //          << " on iter: " << iter << std::endl;
  return final_guess;
}

template <typename T> Thetas<T> get_thetas(Transform<T> EE_transform) {
  Parameters<T> parameters = get_default_parameters<T>();
  return get_thetas(EE_transform, parameters);
}
