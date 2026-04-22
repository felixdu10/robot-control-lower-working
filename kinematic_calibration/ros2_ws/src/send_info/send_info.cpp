#include "templated_classes/Templated_Point.h"
#include "robot_controller.h"
#include "kinematics.h"
#include "inverse_kinematics.h"
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

char getch() {
  struct termios oldt, newt;
  char ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  newt.c_lflag &= ~(ICANON | ECHO); // raw mode
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore
  return ch;
}
bool kbhit() {
  struct timeval tv = {0L, 0L};
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}

rclcpp::Node::SharedPtr node = nullptr;
std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr>
    publishers;

void sendToTopic(Transform<double> T, std::string topic) {
  if (!node) {
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("robot_visualizer");
  }
  if (publishers.find(topic) == publishers.end()) {
    publishers[topic] =
        node->create_publisher<geometry_msgs::msg::Pose>(topic, 10);
  }
  auto pub = publishers[topic];
  geometry_msgs::msg::Pose msg;
  auto q = T.R.toQuaternion();
  msg.orientation.w = q[0];
  msg.orientation.x = q[1];
  msg.orientation.y = q[2];
  msg.orientation.z = q[3];
  msg.position.x = T.p.x / 1000.0;
  msg.position.y = T.p.y / 1000.0;
  msg.position.z = T.p.z / 1000.0;

  pub->publish(msg);
}

Transform<double> getLinkTransform(Point<double> a, Point<double> b,
                                   double z = 0) {
  Point<double> x_hat = (b - a).normalize();
  Point<double> z_hat = {0, 0, 1};
  Point<double> y_hat = cross(z_hat, x_hat);
  a = {a.x, a.y, a.z + z};
  return Transform<double>(x_hat, y_hat, z_hat, a);
}

void sendLinks(Thetas<double> cur_pos, Parameters<double> &params) {
  Transform<double> lower_left_distal_link =
      getLinkTransform(get_linkage_B(cur_pos, params, BOTTOM_LEFT),
                       get_linkage_C(cur_pos, params, BOTTOM_LEFT));

  sendToTopic(lower_left_distal_link, "/lower_left_distal_link");
  Transform<double> lower_right_distal_link =
      getLinkTransform(get_linkage_B(cur_pos, params, BOTTOM_RIGHT),
                       get_linkage_C(cur_pos, params, BOTTOM_RIGHT));
  sendToTopic(lower_right_distal_link, "/lower_right_distal_link");
}

int main() {
  const double step_size = 2.5;
  RobotController robot_controller(true);
  Parameters<double> params = get_default_parameters<double>();
  while (true) {
    // take in input
    bool up_pressed = false;
    bool down_pressed = false;
    bool left_pressed = false;
    bool right_pressed = false;

    char key = getch();

    up_pressed = (key == 'w');
    down_pressed = (key == 's');
    left_pressed = (key == 'a');
    right_pressed = (key == 'd');

    Point<double> direction = {.x = left_pressed * -1.0 + right_pressed * 1.0,
                               .y = up_pressed * 1.0 + down_pressed * -1.0,
                               .z = 0};

    if (direction.magnitude() != 0.0) {
      direction = direction.normalize() * step_size;
    }
    // get current position
    Thetas<double> cur_pos = robot_controller.cur_thetas;
    // turn current position into transforms and publish to topics
    Transform<double> EE = get_end_effector(cur_pos);
    // get desired position
    EE.p = EE.p + direction;
    Transform<double> desired_pos = EE;
    // inverse kinematics on desired position
    if (direction.magnitude() != 0.0) {
      // move robot
      robot_controller.move(get_thetas(EE));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // send current position to nodes
    //
    cur_pos = robot_controller.cur_thetas;
    sendLinks(cur_pos, params);
    rclcpp::spin_some(node);
  }
}
