#include <chrono>
#include <memory>
#include <array>
#include <vector>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <my_robot_interfaces/action/actuate_slides.hpp>

using namespace std::chrono_literals;

using ActuateSlides = my_robot_interfaces::action::ActuateSlides;
using GoalHandleActuateSlides = rclcpp_action::ClientGoalHandle<ActuateSlides>;

class RobotControllerSimple
{
public:
  RobotControllerSimple()
  {
    init_ros();
  }

  void home()
  {
    RCLCPP_INFO(node_->get_logger(), "Sending HOME command");

    auto goal = ActuateSlides::Goal();
    goal.command = "HOME";
    goal.target_positions = {0.0f, 0.0f, 0.0f, 0.0f};
    goal.speed = 1.0f;

    send_goal_and_wait(goal);

    RCLCPP_INFO(node_->get_logger(), "Homing complete");
  }

  void move(std::array<float, 4> pos, float speed)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "Sending MOVE command: [%.2f, %.2f, %.2f, %.2f]",
      pos[0], pos[1], pos[2], pos[3]);

    auto goal = ActuateSlides::Goal();
    goal.command = "MOVE";
    goal.target_positions = pos;
    goal.speed = speed;

    send_goal_and_wait(goal);

    RCLCPP_INFO(node_->get_logger(), "Move complete");
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ActuateSlides>::SharedPtr action_client_;

  void init_ros()
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    node_ = rclcpp::Node::make_shared("robot_controller_simple");

    action_client_ =
      rclcpp_action::create_client<ActuateSlides>(node_, "actuate_slides");

    RCLCPP_INFO(node_->get_logger(), "Waiting for action server...");
    if (!action_client_->wait_for_action_server(10s)) {
      throw std::runtime_error("Action server not available");
    }

    RCLCPP_INFO(node_->get_logger(), "Action server ready");
  }

  void send_goal_and_wait(const ActuateSlides::Goal &goal)
  {
    auto send_options =
      rclcpp_action::Client<ActuateSlides>::SendGoalOptions();

    send_options.feedback_callback =
      [this](
        GoalHandleActuateSlides::SharedPtr,
        const std::shared_ptr<const ActuateSlides::Feedback> fb)
      {
        RCLCPP_INFO(
          node_->get_logger(),
          "反馈: state=%s progress=%.2f",
          fb->state.c_str(),
          fb->progress);
      };

    auto goal_future =
      action_client_->async_send_goal(goal, send_options);

    if (rclcpp::spin_until_future_complete(node_, goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("Failed to send goal");
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
      throw std::runtime_error("Goal rejected");

    auto result_future =
      action_client_->async_get_result(goal_handle);

    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("Failed to get result");
    }

    auto result = result_future.get();

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
      throw std::runtime_error("Action failed");

    RCLCPP_INFO(node_->get_logger(), "Action success");
  }
};

int main()
{
  try {
    RobotControllerSimple robot;

    // 你现在先放最简单的几组测试点
    std::vector<std::array<float, 4>> test_configs = {
      {0.0f, 10.0f, 0.0f, 10.0f},
      {0.0f, 10.0f, 0.0f, 10.0f},
      {0.0f, 10.0f, 0.0f, 10.0f}
    };

    std::cout << "==== Start experiment runner ====" << std::endl;

    for (size_t i = 0; i < test_configs.size(); i++) {
      std::cout << "\n===== Trial " << i + 1 << " =====" << std::endl;

      std::cout << "Step 1: HOME" << std::endl;
      robot.home();

      std::cout << "Step 2: MOVE to target = ["
                << test_configs[i][0] << ", "
                << test_configs[i][1] << ", "
                << test_configs[i][2] << ", "
                << test_configs[i][3] << "]" << std::endl;

      robot.move(test_configs[i], 1.0f);

      std::cout << "Robot reached target." << std::endl;
      std::cout << "现在可以手动测量 / 观察。按回车继续..." << std::endl;
      std::cin.get();

      std::cout << "Step 3: HOME again" << std::endl;
      robot.home();

      std::cout << "Trial " << i + 1 << " finished." << std::endl;
      std::cout << "按回车进入下一组..." << std::endl;
      std::cin.get();
    }

    std::cout << "==== All trials finished ====" << std::endl;
  }
  catch (const std::exception &e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
