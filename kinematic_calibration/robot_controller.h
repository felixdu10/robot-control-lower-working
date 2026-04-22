#ifndef ROBOT_CONTROLLER_CALIBRATION
#define ROBOT_CONTROLLER_CALIBRATION

#include "../galil_control_calls.h"
#include "jacobian.h"
#include "kinematics.h"
#include "atracsys_functions.h"
#include "auto.h"
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <vector>

const static double theta_1_backlash = 0.213;
const static double theta_2_backlash = 0.225;
// TODO: unverified numbers: we are currently guessing
const static double theta_3_backlash = 0.172;
const static double theta_4_backlash = 0.213;

struct lin_eq_ans {
  double x;
  double y;
};

/*
 * Solve equation a*x1+b*y1=x, a*x2+b*y2=y
 *
 */
lin_eq_ans solve_linear_equation(double x1, double x2, double y1, double y2,
                                 double x, double y) {
  double denom = 1 / (x1 * y2 - x2 * y1);
  return {(x * y2 - x2 * y) * denom, (-x * y1 + x1 * y) * denom};
}

template <typename T> Thetas<Auto<5, T>> to_thetas(Thetas<double> s) {

  Thetas<Auto<5, T>> t;
  t.theta_1 = Auto<5, T>(s.theta_1);
  t.theta_1.epsilon[0] = 1;
  t.theta_2 = Auto<5, T>(s.theta_2);
  t.theta_2.epsilon[1] = 1;
  t.theta_3 = Auto<5, T>(s.theta_3);
  t.theta_3.epsilon[2] = 1;
  t.theta_4 = Auto<5, T>(s.theta_4);
  t.theta_4.epsilon[3] = 1;
  t.theta_5 = Auto<5, T>(s.theta_5);
  t.theta_5.epsilon[4] = 1;
  return t;
}

struct position_error {
  Point<double> error_vec;
  double angular_error;
};

struct linkage_ee_positions {
  Point<double> upper_ee;
  Point<double> lower_ee;
};

class RobotController {
public:
  static Thetas<double> live_thetas;
  bool compensating;
  Parameters<double> parameters;
  bool left_forward = true;
  bool right_forward = true;
  bool left_middle_forward = true;
  bool right_middle_forward = true;
  Thetas<double> cur_thetas = home_positions;
  RobotController(bool compensation, Parameters<double> parameters)
      : compensating(compensation), parameters(parameters) {
    left_forward = true;
    right_middle_forward = true;
    right_forward = true;
    left_middle_forward = true;
  }

  RobotController(bool compensation) : compensating(compensation) {
    left_forward = true;
    right_middle_forward = true;
    right_forward = true;
    left_middle_forward = true;
    parameters = get_default_parameters<double>();
  }

  void home() {
    bool left_behind, right_behind, left_middle_behind, right_middle_behind;
    left_behind = right_behind = left_middle_behind = right_middle_behind =
        false;
    std::string input_str;
    std::cout << "is the left slider behind the flag?" << std::endl;
    std::cin >> input_str;
    if (input_str == "y" || input_str == "yes") {
      left_behind = true;
    }
    std::cout << "is the right slider behind the flag?" << std::endl;
    std::cin >> input_str;

    if (input_str == "y" || input_str == "yes") {
      right_behind = true;
    }
    std::cout << "is the left middle slider behind the flag?" << std::endl;
    std::cin >> input_str;
    if (input_str == "y" || input_str == "yes") {
      left_middle_behind = true;
    }
    std::cout << "is the right middle slider behind the flag?" << std::endl;
    std::cin >> input_str;

    if (input_str == "y" || input_str == "yes") {
      right_middle_behind = true;
    }
    init_galil(1);
    std::cout << "Ready to home Low?" << std::endl;
    std::cin >> input_str;
    HomeLowBlocking(right_middle_behind, left_middle_behind);
    std::cout << "Ready to home Up?" << std::endl;
    std::cin >> input_str;
    HomeUpBlocking(right_behind, left_behind);
    std::cout << "Done Home" << std::endl;
  }

  Thetas<double> move2(Thetas<double> desired_pos,
                       AtracsysTracker<double> &atracsys) {
    move(desired_pos);
    Thetas<double> un_adjusted_thetas = cur_thetas;
    int MAX_ITERS = 50;
    double box_size = 1;
    double MAX_TARGETING_ERROR = .15;
    double MAX_ANGULAR_ERROR = .2 * M_PI / 180;
    double SHRINK_RATE = .8;
    double angular_error;
    double targeting_error;
    Measurement<double> cur_measure;
    Parameters<double> params = get_default_parameters<double>();
    Point<double> lower_end_effector_pos =
        get_lower_linkage_P(desired_pos, params);
    Point<double> z_vec =
        (get_upper_linkage_P(desired_pos, params) - lower_end_effector_pos)
            .normalize();
    Point<double> diff_lower;
    for (int i = 0; i < MAX_ITERS; i++) {
      while (atracsys.getMeasurement(BOTH, cur_measure) < 0) {
        std::this_thread::sleep_for(250ms);
      }

      Transform<double> cur_frame =
          F_M1R<double>().inverse() * cur_measure.F_OM1.inverse() *
          cur_measure.F_OM2 * F_M2N<double>(0, params);
      Point<double> cur_lower_end_effector = cur_frame.p;
      Point<double> cur_z = {cur_frame.R.matrix[0][2], cur_frame.R.matrix[1][2],
                             cur_frame.R.matrix[2][2]};
      angular_error = acos(cur_z * z_vec);
      diff_lower = lower_end_effector_pos - cur_lower_end_effector;
      targeting_error = diff_lower.magnitude() + cur_lower_end_effector.z;
      // flip back plate measure if error is too big
      if (targeting_error > 50) {
        cur_measure.F_OM1 =
            cur_measure.F_OM1 * Transform<double>(0, 0, M_PI, 0, 0, 0);
        cur_frame = F_M1R<double>().inverse() * cur_measure.F_OM1.inverse() *
                    cur_measure.F_OM2 * F_M2N<double>(0, params);
        cur_lower_end_effector = cur_frame.p;
        cur_z = {cur_frame.R.matrix[0][2], cur_frame.R.matrix[1][2],
                 cur_frame.R.matrix[2][2]};
        angular_error = acos(cur_z * z_vec);
        //        diff_lower = (lower_end_effector_pos +
        //                      (z_vec * (cur_lower_end_effector.z / z_vec.z)))
        //                      -
        //                     cur_lower_end_effector;
        //
        diff_lower = lower_end_effector_pos - cur_lower_end_effector;
        targeting_error = diff_lower.magnitude();
      }
      std::cout << "iter: " << i << " targeting_error: " << targeting_error
                << " angular_error: " << angular_error << std::endl;
      if (targeting_error < MAX_TARGETING_ERROR &&
          angular_error < MAX_ANGULAR_ERROR) {
        return {
            cur_thetas.theta_1 - un_adjusted_thetas.theta_1,
            cur_thetas.theta_2 - un_adjusted_thetas.theta_2,
            cur_thetas.theta_3 - un_adjusted_thetas.theta_3,
            cur_thetas.theta_4 - un_adjusted_thetas.theta_4,
            cur_thetas.theta_5,
        };
      }
      double dist = (params.upper_base.z.value +
                     params.tunable_params.upper_base_z_offset.value) -
                    (params.lower_base.z.value +
                     params.tunable_params.lower_base_z_offset.value);

      Point<double> cur_upper_end_effector =
          (cur_lower_end_effector +
           cur_z * ((dist - cur_lower_end_effector.z) / cur_z.z));
      Point<double> diff_upper =
          (lower_end_effector_pos + z_vec * (dist / z_vec.z)) -
          cur_upper_end_effector;
      Point<double> vec_1;
      Point<double> vec_2;
      Point<double> vec_3;
      Point<double> vec_4;

      Thetas<double> before_thetas = cur_thetas;
      for (int T = 0; T < 4; T++) {
        // move one motor at a time there and back
        if (T == 0)
          before_thetas.theta_1 += box_size;
        if (T == 1)
          before_thetas.theta_2 += box_size;
        if (T == 2)
          before_thetas.theta_3 += box_size;
        if (T == 3)
          before_thetas.theta_4 += box_size;
        move(before_thetas);

        // take a measurement at each spot
        while (atracsys.getMeasurement(BOTH, cur_measure) < 0) {
          std::this_thread::sleep_for(250ms);
        }
        // get upper and lower
        cur_frame = F_M1R<double>().inverse() * cur_measure.F_OM1.inverse() *
                    cur_measure.F_OM2 * F_M2N<double>(0, params);

        Point<double> new_lower_end_effector = cur_frame.p;
        cur_z = {cur_frame.R.matrix[0][2], cur_frame.R.matrix[1][2],
                 cur_frame.R.matrix[2][2]};
        // flip back plate measure if error is too big
        if (targeting_error > 50) {
          cur_measure.F_OM1 =
              cur_measure.F_OM1 * Transform<double>(0, 0, M_PI, 0, 0, 0);
          cur_frame = F_M1R<double>().inverse() * cur_measure.F_OM1.inverse() *
                      cur_measure.F_OM2 * F_M2N<double>(0, params);
          new_lower_end_effector = cur_frame.p;
          cur_z = {cur_frame.R.matrix[0][2], cur_frame.R.matrix[1][2],
                   cur_frame.R.matrix[2][2]};
        }
        dist = (params.upper_base.z.value +
                params.tunable_params.upper_base_z_offset.value) -
               (params.lower_base.z.value +
                params.tunable_params.lower_base_z_offset.value);

        Point<double> new_upper_end_effector =
            (new_lower_end_effector +
             cur_z * ((dist - new_lower_end_effector.z) / cur_z.z));

        // get the vector for each motion
        if (T == 0) {
          vec_1 = new_upper_end_effector - cur_upper_end_effector;
        }
        if (T == 1) {
          vec_2 = new_upper_end_effector - cur_upper_end_effector;
        }
        if (T == 2) {
          vec_3 = new_lower_end_effector - cur_lower_end_effector;
        }
        if (T == 3)
          vec_4 = new_lower_end_effector - cur_lower_end_effector;
        if (T == 0)
          before_thetas.theta_1 -= box_size;
        if (T == 1)
          before_thetas.theta_2 -= box_size;
        if (T == 2)
          before_thetas.theta_3 -= box_size;
        if (T == 3)
          before_thetas.theta_4 -= box_size;
        if (T == 0 && vec_1.x < 0 || vec_1.y < 0) {
          T--;
        }
        if (T == 1 && vec_2.x > 0 || vec_2.y < 0) {
          T--;
        }

        if (T == 2 && vec_3.x < 0 || vec_3.y < 0) {
          T--;
        }
        if (T == 3 && vec_4.x > 0 || vec_4.y < 0) {
          T--;
        }
        move(before_thetas);
      }
      // solve linear equation for next motion
      lin_eq_ans upper_ans = solve_linear_equation(
          vec_1.x, vec_2.x, vec_1.y, vec_2.y, diff_upper.x, diff_upper.y);
      lin_eq_ans lower_ans = solve_linear_equation(
          vec_3.x, vec_4.x, vec_3.y, vec_4.y, diff_lower.x, diff_lower.y);
      Thetas<double> adjusted_thetas = {
          cur_thetas.theta_1 + upper_ans.x * box_size,
          cur_thetas.theta_2 + upper_ans.y * box_size,
          cur_thetas.theta_3 + lower_ans.x * box_size,
          cur_thetas.theta_4 + lower_ans.y * box_size, cur_thetas.theta_5};
      // go
      move(adjusted_thetas);
      // decrease box size
      box_size *= .75;
    }
  }

  Thetas<double> move(Thetas<double> desired_pos,
                      AtracsysTracker<double> &atracsys) {
    double targeting_error = 100;
    double angular_error = 20;
    int iters = 0;
    double MAX_ITERS = 50;
    double MAX_TARGETING_ERROR = .1;
    double MAX_ANGULAR_ERROR = .4 * M_PI / 180;
    // go x% of the distance towards goal on each iteration.
    double UPDATE_RATE_1 = 1;
    double UPDATE_RATE_2 = 1;
    double UPDATE_RATE_3 = 1;
    double UPDATE_RATE_4 = 1;
    double DECAY_RATE = .95;
    Thetas<double> start = desired_pos;
    encoder_error_struct errs = move(desired_pos);
    std::cout << "begining adjustments" << std::endl;
    Parameters<double> params = get_default_parameters<double>();
    Point<double> lower_end_effector_pos =
        get_lower_linkage_P(desired_pos, params);
    Point<double> z_vec =
        (get_upper_linkage_P(desired_pos, params) - lower_end_effector_pos)
            .normalize();
    Thetas<double> un_adjusted_thetas = cur_thetas;
    std::vector<Measurement<double>> measurements;
    Measurement<double> cur_measure;
    Measurement<double> best_measure;
    Thetas<double> desired_thetas;
    Point<double> diff_no_z;
    Point<double> diff_lower;
    double best_angular;
    double best_targeting;
    lin_eq_ans prev_upper_diffs = {0, 0};
    lin_eq_ans prev_lower_diffs = {0, 0};
    do {
      iters++;
      while (atracsys.getMeasurement(BOTH, cur_measure) < 0) {
        std::this_thread::sleep_for(250ms);
      }
      Transform<double> cur_frame =
          F_M1R<double>().inverse() * cur_measure.F_OM1.inverse() *
          cur_measure.F_OM2 * F_M2N<double>(0, params);
      Point<double> cur_lower_end_effector = cur_frame.p;
      Point<double> cur_z = {cur_frame.R.matrix[0][2], cur_frame.R.matrix[1][2],
                             cur_frame.R.matrix[2][2]};
      angular_error = acos(cur_z * z_vec);
      // diff_lower = (lower_end_effector_pos +
      //               (cur_z * (cur_lower_end_effector.z / cur_z.z))) -
      //              cur_lower_end_effector;

      diff_lower = lower_end_effector_pos - cur_lower_end_effector;

      diff_no_z = {diff_lower.x, diff_lower.y, 0};
      targeting_error = diff_no_z.magnitude();
      // flip back plate measure if error is too big
      if (targeting_error > 50) {
        cur_measure.F_OM1 =
            cur_measure.F_OM1 * Transform<double>(0, 0, M_PI, 0, 0, 0);
        cur_frame = F_M1R<double>().inverse() * cur_measure.F_OM1.inverse() *
                    cur_measure.F_OM2 * F_M2N<double>(0, params);
        cur_lower_end_effector = cur_frame.p;
        cur_z = {cur_frame.R.matrix[0][2], cur_frame.R.matrix[1][2],
                 cur_frame.R.matrix[2][2]};
        angular_error = acos(cur_z * z_vec);
        diff_lower = lower_end_effector_pos - cur_lower_end_effector;
        targeting_error = diff_no_z.magnitude();
      }

      std::cout << "on iter " << iters
                << " targetting_error: " << targeting_error << std::endl
                << "angular_error (rad): " << angular_error << std::endl;
      double dist = (params.upper_base.z.value +
                     params.tunable_params.upper_base_z_offset.value) -
                    (params.lower_base.z.value +
                     params.tunable_params.lower_base_z_offset.value);
      Point<double> cur_upper_end_effector =
          (cur_lower_end_effector +
           cur_z * ((dist - cur_lower_end_effector.z) / cur_z.z));
      // Point<double> diff_upper =
      //     (lower_end_effector_pos + z_vec * (dist / z_vec.z)) -
      //     cur_upper_end_effector;
      Point<double> diff_upper = (z_vec - cur_z) * dist;
      Parameters<Auto<5, double>> templated_params =
          get_default_parameters<Auto<5, double>>();
      Point<Auto<5, double>> estimated_lower_jacobian =
          get_lower_linkage_P<Auto<5, double>>(to_thetas<double>(cur_thetas),
                                               templated_params);
      Point<Auto<5, double>> estimated_upper_jacobian =
          get_upper_linkage_P<Auto<5, double>>(to_thetas<double>(cur_thetas),
                                               templated_params);
      lin_eq_ans lower_diff_thetas = solve_linear_equation(
          estimated_lower_jacobian.x.epsilon[2],
          estimated_lower_jacobian.x.epsilon[3],
          estimated_lower_jacobian.y.epsilon[2],
          estimated_lower_jacobian.y.epsilon[3], diff_lower.x, diff_lower.y);
      lin_eq_ans upper_dif_thetas = solve_linear_equation(
          estimated_upper_jacobian.x.epsilon[0],
          estimated_upper_jacobian.x.epsilon[1],
          estimated_upper_jacobian.y.epsilon[0],
          estimated_upper_jacobian.y.epsilon[1], diff_upper.x, diff_upper.y);
      if (upper_dif_thetas.x * prev_upper_diffs.x < 1) {
        UPDATE_RATE_1 *= DECAY_RATE;
      } else {
        // UPDATE_RATE_1 /= DECAY_RATE;
      }
      if (upper_dif_thetas.y * prev_upper_diffs.y < 1) {
        UPDATE_RATE_2 *= DECAY_RATE;
      } else {
        // UPDATE_RATE_2 /= DECAY_RATE;
      }

      if (lower_diff_thetas.x * prev_lower_diffs.x < 1) {
        UPDATE_RATE_3 *= DECAY_RATE;
      } else {
        // UPDATE_RATE_3 /= DECAY_RATE;
      }

      if (lower_diff_thetas.x * prev_lower_diffs.x < 1) {
        UPDATE_RATE_4 *= DECAY_RATE;
      } else {
        // UPDATE_RATE_4 /= DECAY_RATE;
      }
      // if (targeting_error < MAX_TARGETING_ERROR) {
      //   UPDATE_RATE_3 = 0;
      //   UPDATE_RATE_4 = 0;
      // }
      // if (angular_error < .03 && angular_error > MAX_ANGULAR_ERROR) {
      //   UPDATE_RATE_1 = std::max(UPDATE_RATE_1, .01);
      //   UPDATE_RATE_2 = std::max(UPDATE_RATE_2, .01);
      // }
      prev_upper_diffs = upper_dif_thetas;
      prev_lower_diffs = lower_diff_thetas;
      desired_thetas = {
          cur_thetas.theta_1 + UPDATE_RATE_1 * upper_dif_thetas.x,
          cur_thetas.theta_2 + UPDATE_RATE_2 * upper_dif_thetas.y,
          cur_thetas.theta_3 + UPDATE_RATE_3 * lower_diff_thetas.x,
          cur_thetas.theta_4 + UPDATE_RATE_4 * lower_diff_thetas.y,
          cur_thetas.theta_5};
      if (diff_lower.magnitude() > MAX_TARGETING_ERROR ||
          angular_error > MAX_ANGULAR_ERROR) {
        errs = move(desired_thetas);
      }
    } while (iters < MAX_ITERS &&
             (diff_no_z.magnitude() > MAX_TARGETING_ERROR ||
              angular_error > MAX_ANGULAR_ERROR));

    std::cout << "on iter " << iters << " targetting_error: " << targeting_error
              << std::endl
              << "angular_error (rad): " << angular_error << std::endl;
    std::cout << "z contribution to position error : " << diff_lower.z
              << std::endl;
    if (iters == MAX_ITERS) {
      throw std::runtime_error("took too long to adjust");
    }
    return {
        cur_thetas.theta_1 - un_adjusted_thetas.theta_1,
        cur_thetas.theta_2 - un_adjusted_thetas.theta_2,
        cur_thetas.theta_3 - un_adjusted_thetas.theta_3,
        cur_thetas.theta_4 - un_adjusted_thetas.theta_4,
        cur_thetas.theta_5,
    };
  }

  encoder_error_struct move(Thetas<double> new_thetas) {

    if (big_motion(new_thetas, cur_thetas)) {
      Thetas<double> first_thetas;
      first_thetas.theta_1 = (new_thetas.theta_1 + cur_thetas.theta_1) / 2;
      first_thetas.theta_2 = (new_thetas.theta_2 + cur_thetas.theta_2) / 2;
      first_thetas.theta_3 = (new_thetas.theta_3 + cur_thetas.theta_3) / 2;
      first_thetas.theta_4 = (new_thetas.theta_4 + cur_thetas.theta_4) / 2;
      first_thetas.theta_5 = cur_thetas.theta_5;
      move(first_thetas);
      return move(new_thetas);
    }
    init_galil(5);
    Thetas<double> targeting_thetas = new_thetas;
    if (compensating) {
      // left slider
      if (left_forward) {
        if (new_thetas.theta_1 < cur_thetas.theta_1 &&
            abs(new_thetas.theta_1 - cur_thetas.theta_1) > .01) {
          targeting_thetas.theta_1 -= theta_1_backlash;
          left_forward = false;
        }
      } else {
        if (new_thetas.theta_1 > cur_thetas.theta_1 &&
            abs(new_thetas.theta_1 - cur_thetas.theta_1) > .01) {
          targeting_thetas.theta_1 += theta_1_backlash;
          left_forward = true;
        }
      } // left slider end
      // right slider
      if (right_forward) {
        if (new_thetas.theta_2 < cur_thetas.theta_2 &&
            abs(new_thetas.theta_2 - cur_thetas.theta_2) > .01) {
          targeting_thetas.theta_2 -= theta_2_backlash;
          right_forward = false;
        }
      } else {
        if (new_thetas.theta_2 > cur_thetas.theta_2 &&
            abs(new_thetas.theta_2 - cur_thetas.theta_2) > .01) {
          targeting_thetas.theta_2 += theta_2_backlash;
          right_forward = true;
        }
      } // right slider end
      // left middle slider
      if (left_middle_forward) {
        if (new_thetas.theta_3 < cur_thetas.theta_3 &&
            abs(new_thetas.theta_3 - cur_thetas.theta_3) > .01) {
          targeting_thetas.theta_3 -= theta_3_backlash;
          left_middle_forward = false;
        }
      } else {
        if (new_thetas.theta_3 > cur_thetas.theta_3 &&
            abs(new_thetas.theta_3 - cur_thetas.theta_3) > .01) {
          targeting_thetas.theta_3 += theta_3_backlash;
          left_middle_forward = true;
        }
      } // left middle slider end
        // right middle slider
      if (right_middle_forward) {
        if (new_thetas.theta_4 < cur_thetas.theta_4 &&
            abs(new_thetas.theta_4 - cur_thetas.theta_4) > .01) {
          targeting_thetas.theta_4 -= theta_4_backlash;
          right_middle_forward = false;
        }
      } else {
        if (new_thetas.theta_4 > cur_thetas.theta_4 &&
            abs(new_thetas.theta_4 - cur_thetas.theta_4) > .01) {
          targeting_thetas.theta_4 += theta_4_backlash;
          right_middle_forward = true;
        }
      } // right middle slider end
    }
    encoder_error_struct errs =
        move_robot_with_slider_positions(targeting_thetas);
    cur_thetas = new_thetas;
    cur_thetas.theta_1 += errs.mmErrLeft;
    cur_thetas.theta_2 += errs.mmErrRight;
    cur_thetas.theta_3 += errs.mmErrLeftMiddle;
    cur_thetas.theta_4 += errs.mmErrRightMiddle;
    return errs;
  }

private:
  bool big_motion(Thetas<double> new_thetas, Thetas<double> old_thetas) {
    if (abs(new_thetas.theta_1 - old_thetas.theta_1) < 7 &&
        abs(new_thetas.theta_2 - old_thetas.theta_2) < 7 &&
        abs(new_thetas.theta_3 - old_thetas.theta_3) < 7 &&
        abs(new_thetas.theta_4 - old_thetas.theta_4) < 7) {
      return false;
    }
    return true;
    Transform<double> t_new = get_end_effector(new_thetas);
    Transform<double> t_old = get_end_effector(old_thetas);
    Transform<double> t_diff = t_old * t_new.inverse();
    Point<double> z = {t_diff.R.matrix[0][2], t_diff.R.matrix[1][2],
                       t_diff.R.matrix[2][2]};
    Point<double> Z = {0, 0, 1};
    return (t_diff.p.magnitude() > 15 || acos(z * Z) * 180.0 / M_PI > 10);
  }
};

#endif // !define ROBOT_CONTROLLER_CALIBRATION
