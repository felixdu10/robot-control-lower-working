#ifndef ROBOT_CONTROLLER_CALIBRATION_ROS
#define ROBOT_CONTROLLER_CALIBRATION_ROS

#include "jacobian.h"
#include "kinematics.h"
#include "atracsys_functions.h"
#include "auto.h"
#include "../galil_control_calls.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <my_robot_interfaces/action/actuate_slides.hpp>

#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <vector>
#include <future>

using ActuateSlides = my_robot_interfaces::action::ActuateSlides;
using ActuateSlidesGAH = rclcpp_action::ClientGoalHandle<ActuateSlides>;

const static double theta_1_backlash = 0.0;
const static double theta_2_backlash = 0.0;
const static double theta_3_backlash = 0.0;
const static double theta_4_backlash = 0.0;

struct lin_eq_ans {
  double x;
  double y;
};

lin_eq_ans solve_linear_equation(double x1, double x2, double y1, double y2,
                                 double x, double y) {
  double denom = 1.0 / (x1 * y2 - x2 * y1);
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

class RobotControllerROS {
public:
  static Thetas<double> live_thetas;

  bool compensating;
  Parameters<double> parameters;

  bool left_forward = true;
  bool right_forward = true;
  bool left_middle_forward = true;
  bool right_middle_forward = true;

  Thetas<double> cur_thetas = home_positions;

  // ------------------------------------------------------------------
  // Construction
  // ------------------------------------------------------------------

  /// Full constructor – pass in an already-spun rclcpp::Node and a
  /// pre-created action client so the controller is testable without
  /// depending on a global executor.
  RobotControllerROS(bool compensation, Parameters<double> params,
                  rclcpp::Node::SharedPtr node,
                  rclcpp_action::Client<ActuateSlides>::SharedPtr action_client)
      : compensating(compensation), parameters(params), node_(node),
        action_client_(action_client) {
    reset_direction_flags();
  }

  /// Convenience constructor – creates its own node and action client.
  RobotControllerROS(bool compensation, Parameters<double> params)
      : compensating(compensation), parameters(params) {
    reset_direction_flags();
    init_ros();
  }

  /// Minimal constructor – uses default parameters.
  explicit RobotControllerROS(bool compensation) : compensating(compensation) {
    reset_direction_flags();
    parameters = get_default_parameters<double>();
    init_ros();
  }

  // ------------------------------------------------------------------
  // Home  (sends a HOME command via the action server)
  // ------------------------------------------------------------------
  void home() {
    RCLCPP_INFO(node_->get_logger(), "Sending HOME command");

    auto goal = ActuateSlides::Goal();
    goal.command = "HOME";
    goal.target_positions = {0.0f, 0.0f, 0.0f, 0.0f};
    goal.speed = 1.0f;

    send_goal_and_wait(goal);

    cur_thetas.theta_1 = 0.0;
    cur_thetas.theta_2 = 0.0;
    cur_thetas.theta_3 = 0.0;
    cur_thetas.theta_4 = 0.0;

    RCLCPP_INFO(node_->get_logger(), "Homing complete");
  }

  // ------------------------------------------------------------------
  // Closed-loop move with Atracsys (move2 variant – coarse Jacobian)
  // ------------------------------------------------------------------
  Thetas<double> move2(Thetas<double> desired_pos,
                       AtracsysTracker<double> &atracsys) {
    move(desired_pos);
    Thetas<double> un_adjusted_thetas = cur_thetas;

    const int MAX_ITERS = 50;
    double box_size = 1.0;
    const double MAX_TARGETING_ERROR = 0.15;
    const double MAX_ANGULAR_ERROR = 0.2 * M_PI / 180.0;

    double angular_error = 0.0;
    double targeting_error = 0.0;

    Measurement<double> cur_measure;
    Parameters<double> params = get_default_parameters<double>();

    Point<double> lower_end_effector_pos =
        get_lower_linkage_P(desired_pos, params);
    Point<double> z_vec =
        (get_upper_linkage_P(desired_pos, params) - lower_end_effector_pos)
            .normalize();

    Point<double> diff_lower;

    for (int i = 0; i < MAX_ITERS; i++) {
      while (atracsys.getMeasurement(BOTH, cur_measure) < 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

      Transform<double> cur_frame =
          F_M1R<double>().inverse() * cur_measure.F_OM1.inverse() *
          cur_measure.F_OM2 * F_M2N<double>(0, params);

      Point<double> cur_lower_end_effector = cur_frame.p;
      Point<double> cur_z = {cur_frame.R.matrix[0][2], cur_frame.R.matrix[1][2],
                             cur_frame.R.matrix[2][2]};

      angular_error = acos(cur_z * z_vec);
      diff_lower = lower_end_effector_pos - cur_lower_end_effector;
      targeting_error = diff_lower.magnitude() + cur_lower_end_effector.z;

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
        targeting_error = diff_lower.magnitude();
      }

      RCLCPP_INFO(node_->get_logger(),
                  "iter %d  targeting_error=%.4f  angular_error=%.6f", i,
                  targeting_error, angular_error);

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
          cur_lower_end_effector +
          cur_z * ((dist - cur_lower_end_effector.z) / cur_z.z);

      Point<double> diff_upper =
          (lower_end_effector_pos + z_vec * (dist / z_vec.z)) -
          cur_upper_end_effector;

      Point<double> vec_1, vec_2, vec_3, vec_4;
      Thetas<double> before_thetas = cur_thetas;

      for (int T = 0; T < 4; T++) {
        if (T == 0)
          before_thetas.theta_1 += box_size;
        if (T == 1)
          before_thetas.theta_2 += box_size;
        if (T == 2)
          before_thetas.theta_3 += box_size;
        if (T == 3)
          before_thetas.theta_4 += box_size;

        move(before_thetas);

        while (atracsys.getMeasurement(BOTH, cur_measure) < 0)
          std::this_thread::sleep_for(std::chrono::milliseconds(250));

        cur_frame = F_M1R<double>().inverse() * cur_measure.F_OM1.inverse() *
                    cur_measure.F_OM2 * F_M2N<double>(0, params);

        Point<double> new_lower_end_effector = cur_frame.p;
        cur_z = {cur_frame.R.matrix[0][2], cur_frame.R.matrix[1][2],
                 cur_frame.R.matrix[2][2]};

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
            new_lower_end_effector +
            cur_z * ((dist - new_lower_end_effector.z) / cur_z.z);

        if (T == 0)
          vec_1 = new_upper_end_effector - cur_upper_end_effector;
        if (T == 1)
          vec_2 = new_upper_end_effector - cur_upper_end_effector;
        if (T == 2)
          vec_3 = new_lower_end_effector - cur_lower_end_effector;
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

        // Original direction-sanity checks preserved verbatim
        if (T == 0 && vec_1.x < 0 || vec_1.y < 0)
          T--;
        if (T == 1 && vec_2.x > 0 || vec_2.y < 0)
          T--;
        if (T == 2 && vec_3.x < 0 || vec_3.y < 0)
          T--;
        if (T == 3 && vec_4.x > 0 || vec_4.y < 0)
          T--;

        move(before_thetas);
      }

      lin_eq_ans upper_ans = solve_linear_equation(
          vec_1.x, vec_2.x, vec_1.y, vec_2.y, diff_upper.x, diff_upper.y);
      lin_eq_ans lower_ans = solve_linear_equation(
          vec_3.x, vec_4.x, vec_3.y, vec_4.y, diff_lower.x, diff_lower.y);

      Thetas<double> adjusted_thetas = {
          cur_thetas.theta_1 + upper_ans.x * box_size,
          cur_thetas.theta_2 + upper_ans.y * box_size,
          cur_thetas.theta_3 + lower_ans.x * box_size,
          cur_thetas.theta_4 + lower_ans.y * box_size, cur_thetas.theta_5};

      move(adjusted_thetas);
      box_size *= 0.75;
    }

    // Should not be reached if MAX_ITERS is sufficient
    return {
        cur_thetas.theta_1 - un_adjusted_thetas.theta_1,
        cur_thetas.theta_2 - un_adjusted_thetas.theta_2,
        cur_thetas.theta_3 - un_adjusted_thetas.theta_3,
        cur_thetas.theta_4 - un_adjusted_thetas.theta_4,
        cur_thetas.theta_5,
    };
  }

  // ------------------------------------------------------------------
  // Closed-loop move with Atracsys (Jacobian / iterative variant)
  // ------------------------------------------------------------------
  Thetas<double> move(Thetas<double> desired_pos,
                      AtracsysTracker<double> &atracsys) {
    double targeting_error = 100.0;
    double angular_error = 20.0;
    int iters = 0;

    const int MAX_ITERS = 50;
    const double MAX_TARGETING_ERROR = 0.1;
    const double MAX_ANGULAR_ERROR = 0.4 * M_PI / 180.0;

    double UPDATE_RATE_1 = 1.0, UPDATE_RATE_2 = 1.0;
    double UPDATE_RATE_3 = 1.0, UPDATE_RATE_4 = 1.0;
    const double DECAY_RATE = 0.95;

    move(desired_pos);

    RCLCPP_INFO(node_->get_logger(), "Beginning adjustments");

    Parameters<double> params = get_default_parameters<double>();

    Point<double> lower_end_effector_pos =
        get_lower_linkage_P(desired_pos, params);
    Point<double> z_vec =
        (get_upper_linkage_P(desired_pos, params) - lower_end_effector_pos)
            .normalize();

    Thetas<double> un_adjusted_thetas = cur_thetas;

    Measurement<double> cur_measure;
    Thetas<double> desired_thetas;

    Point<double> diff_no_z;
    Point<double> diff_lower;

    lin_eq_ans prev_upper_diffs = {0.0, 0.0};
    lin_eq_ans prev_lower_diffs = {0.0, 0.0};

    do {
      ++iters;

      while (atracsys.getMeasurement(BOTH, cur_measure) < 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

      Transform<double> cur_frame =
          F_M1R<double>().inverse() * cur_measure.F_OM1.inverse() *
          cur_measure.F_OM2 * F_M2N<double>(0, params);

      Point<double> cur_lower_end_effector = cur_frame.p;
      Point<double> cur_z = {cur_frame.R.matrix[0][2], cur_frame.R.matrix[1][2],
                             cur_frame.R.matrix[2][2]};

      angular_error = acos(cur_z * z_vec);
      diff_lower = lower_end_effector_pos - cur_lower_end_effector;
      diff_no_z = {diff_lower.x, diff_lower.y, 0.0};
      targeting_error = diff_no_z.magnitude();

      if (targeting_error > 50.0) {
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

      RCLCPP_INFO(node_->get_logger(),
                  "iter %d  targeting_error=%.4f  angular_error(rad)=%.6f",
                  iters, targeting_error, angular_error);

      double dist = (params.upper_base.z.value +
                     params.tunable_params.upper_base_z_offset.value) -
                    (params.lower_base.z.value +
                     params.tunable_params.lower_base_z_offset.value);

      Point<double> cur_upper_end_effector =
          cur_lower_end_effector +
          cur_z * ((dist - cur_lower_end_effector.z) / cur_z.z);

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

      if (upper_dif_thetas.x * prev_upper_diffs.x < 1.0)
        UPDATE_RATE_1 *= DECAY_RATE;
      if (upper_dif_thetas.y * prev_upper_diffs.y < 1.0)
        UPDATE_RATE_2 *= DECAY_RATE;
      if (lower_diff_thetas.x * prev_lower_diffs.x < 1.0)
        UPDATE_RATE_3 *= DECAY_RATE;
      if (lower_diff_thetas.x * prev_lower_diffs.x < 1.0)
        UPDATE_RATE_4 *= DECAY_RATE;

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
        move(desired_thetas);
      }

    } while (iters < MAX_ITERS &&
             (diff_no_z.magnitude() > MAX_TARGETING_ERROR ||
              angular_error > MAX_ANGULAR_ERROR));

    RCLCPP_INFO(node_->get_logger(),
                "Final iter %d  targeting_error=%.4f  angular_error(rad)=%.6f",
                iters, targeting_error, angular_error);
    RCLCPP_INFO(node_->get_logger(), "Z contribution to position error: %.4f",
                diff_lower.z);

    if (iters == MAX_ITERS)
      throw std::runtime_error("took too long to adjust");

    return {
        cur_thetas.theta_1 - un_adjusted_thetas.theta_1,
        cur_thetas.theta_2 - un_adjusted_thetas.theta_2,
        cur_thetas.theta_3 - un_adjusted_thetas.theta_3,
        cur_thetas.theta_4 - un_adjusted_thetas.theta_4,
        cur_thetas.theta_5,
    };
  }

  // ------------------------------------------------------------------
  // Base move – replaces move_robot_with_slider_positions + init_galil
  // ------------------------------------------------------------------
  encoder_error_struct move(Thetas<double> new_thetas) {
    if (big_motion(new_thetas, cur_thetas)) {
      Thetas<double> mid;
      mid.theta_1 = (new_thetas.theta_1 + cur_thetas.theta_1) / 2.0;
      mid.theta_2 = (new_thetas.theta_2 + cur_thetas.theta_2) / 2.0;
      mid.theta_3 = (new_thetas.theta_3 + cur_thetas.theta_3) / 2.0;
      mid.theta_4 = (new_thetas.theta_4 + cur_thetas.theta_4) / 2.0;
      mid.theta_5 = cur_thetas.theta_5;
      move(mid);
      return move(new_thetas);
    }

    // --- Backlash compensation (unchanged logic) ---
    Thetas<double> targeting_thetas = new_thetas;
    if (compensating) {
      // left slider
      if (left_forward) {
        if (new_thetas.theta_1 < cur_thetas.theta_1 &&
            std::abs(new_thetas.theta_1 - cur_thetas.theta_1) > 0.01) {
          targeting_thetas.theta_1 -= theta_1_backlash;
          left_forward = false;
        }
      } else {
        if (new_thetas.theta_1 > cur_thetas.theta_1 &&
            std::abs(new_thetas.theta_1 - cur_thetas.theta_1) > 0.01) {
          targeting_thetas.theta_1 += theta_1_backlash;
          left_forward = true;
        }
      }
      // right slider
      if (right_forward) {
        if (new_thetas.theta_2 < cur_thetas.theta_2 &&
            std::abs(new_thetas.theta_2 - cur_thetas.theta_2) > 0.01) {
          targeting_thetas.theta_2 -= theta_2_backlash;
          right_forward = false;
        }
      } else {
        if (new_thetas.theta_2 > cur_thetas.theta_2 &&
            std::abs(new_thetas.theta_2 - cur_thetas.theta_2) > 0.01) {
          targeting_thetas.theta_2 += theta_2_backlash;
          right_forward = true;
        }
      }
      // left middle slider
      if (left_middle_forward) {
        if (new_thetas.theta_3 < cur_thetas.theta_3 &&
            std::abs(new_thetas.theta_3 - cur_thetas.theta_3) > 0.01) {
          targeting_thetas.theta_3 -= theta_3_backlash;
          left_middle_forward = false;
        }
      } else {
        if (new_thetas.theta_3 > cur_thetas.theta_3 &&
            std::abs(new_thetas.theta_3 - cur_thetas.theta_3) > 0.01) {
          targeting_thetas.theta_3 += theta_3_backlash;
          left_middle_forward = true;
        }
      }
      // right middle slider
      if (right_middle_forward) {
        if (new_thetas.theta_4 < cur_thetas.theta_4 &&
            std::abs(new_thetas.theta_4 - cur_thetas.theta_4) > 0.01) {
          targeting_thetas.theta_4 -= theta_4_backlash;
          right_middle_forward = false;
        }
      } else {
        if (new_thetas.theta_4 > cur_thetas.theta_4 &&
            std::abs(new_thetas.theta_4 - cur_thetas.theta_4) > 0.01) {
          targeting_thetas.theta_4 += theta_4_backlash;
          right_middle_forward = true;
        }
      }
    }

    // --- Build and send the ROS 2 action goal ---
    const float MM_PER_UNIT = 1.0f; // adjust if thetas are already in mm

    auto goal = ActuateSlides::Goal();
    goal.command = "MOVE";
    goal.target_positions = {
        static_cast<float>(targeting_thetas.theta_1 * MM_PER_UNIT),
        static_cast<float>(targeting_thetas.theta_2 * MM_PER_UNIT),
        static_cast<float>(targeting_thetas.theta_3 * MM_PER_UNIT),
        static_cast<float>(targeting_thetas.theta_4 * MM_PER_UNIT),
    };
    goal.speed = 1.0f;

    send_goal_and_wait(goal);

    // Encoder errors are zeroed out — position tracking stays in
    // cur_thetas rather than being corrected from hardware encoders.
    cur_thetas = new_thetas;
    return encoder_error_struct{};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ActuateSlides>::SharedPtr action_client_;

  void reset_direction_flags() {
    left_forward = true;
    right_forward = true;
    left_middle_forward = true;
    right_middle_forward = true;
  }

  void init_ros() {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    node_ = rclcpp::Node::make_shared("robot_controller");
    action_client_ =
        rclcpp_action::create_client<ActuateSlides>(node_, "actuate_slides");

    RCLCPP_INFO(node_->get_logger(),
                "Waiting for actuate_slides action server...");
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      throw std::runtime_error(
          "actuate_slides action server not available after 10 s");
    }
    RCLCPP_INFO(node_->get_logger(), "Action server ready");
  }

  /// Sends a goal and spins until a result arrives. Throws on abort/cancel.
  void send_goal_and_wait(const ActuateSlides::Goal &goal) {
    auto send_options = rclcpp_action::Client<ActuateSlides>::SendGoalOptions();

    // Optional: log feedback as it arrives
    send_options.feedback_callback =
        [this](ActuateSlidesGAH::SharedPtr,
               const std::shared_ptr<const ActuateSlides::Feedback> fb) {
          RCLCPP_DEBUG(node_->get_logger(),
                       "Action feedback: state=%s  progress=%.2f",
                       fb->state.c_str(), fb->progress);
        };

    auto goal_handle_future =
        action_client_->async_send_goal(goal, send_options);

    // Spin until the goal is accepted
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Failed to send goal to actuate_slides");
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      throw std::runtime_error("Goal was rejected by actuate_slides server");
    }

    // Spin until the result arrives
    auto result_future = action_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Failed to get result from actuate_slides");
    }

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      throw std::runtime_error(
          std::string("actuate_slides action did not succeed: ") +
          result.result->message);
    }

    RCLCPP_INFO(node_->get_logger(), "Action succeeded: %s",
                result.result->message.c_str());
  }

  bool big_motion(Thetas<double> new_thetas, Thetas<double> old_thetas) {
    if (std::abs(new_thetas.theta_1 - old_thetas.theta_1) < 7.0 &&
        std::abs(new_thetas.theta_2 - old_thetas.theta_2) < 7.0 &&
        std::abs(new_thetas.theta_3 - old_thetas.theta_3) < 7.0 &&
        std::abs(new_thetas.theta_4 - old_thetas.theta_4) < 7.0) {
      return false;
    }
    return true;

    // Dead code below preserved from original for reference:
    Transform<double> t_new = get_end_effector(new_thetas);
    Transform<double> t_old = get_end_effector(old_thetas);
    Transform<double> t_diff = t_old * t_new.inverse();
    Point<double> z = {t_diff.R.matrix[0][2], t_diff.R.matrix[1][2],
                       t_diff.R.matrix[2][2]};
    Point<double> Z = {0, 0, 1};
    return (t_diff.p.magnitude() > 15.0 || acos(z * Z) * 180.0 / M_PI > 10.0);
  }
};

#endif // ROBOT_CONTROLLER_CALIBRATION
