#include "galil_control_calls.h"
#include "inverse_kinematics.h"

int main() {
  init_galil(home);
  HomeUpBlocking(1, 1);
  HomeLowBlocking(1, 1);
  init_galil(5);
  Point target = {0, 420, -115};
  Robot inverse_robot;
  NewTransform T(0, 0, 0, 0, 0, 0);
  for (double i = 0; i < 2 * M_PI; i += M_PI / 8) {
    approach_definition def = {target, M_PI / 6, i};
    slider_positions positions = inverse_kinematics(def, T, inverse_robot);
    move_robot_with_slider_positions(positions);
  }
  stop_galil();
}
