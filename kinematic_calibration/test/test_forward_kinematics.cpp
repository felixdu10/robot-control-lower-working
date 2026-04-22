#include "../templated_classes/Templated_Point.h"
#include "kinematics.h"
#include <cassert>
namespace External {
#include "../forward_kinematics.cpp"
}
void test_get_linkage_B() {
  Thetas<double> thetas = {
      100, 100, 100, 100, 0,
  };
  Parameters<double> p = get_default_parameters<double>();
  Point<double> point = get_linkage_B(thetas, p, TOP_RIGHT);
  Point<double> expected = {.x = 101.22008, .y = 242.66122, .z = 31.5};
  assert(point == expected);
}

void test_get_linkage_C() {
  Thetas<double> thetas = {
      123, 95, 100, 100, 0,
  };
  Parameters<double> p = get_default_parameters<double>();
  Point<double> point = get_linkage_C(thetas, p, TOP_RIGHT);
  Point<double> expected = {.x = 44.32653, .y = 341.85742, .z = 31.5};
  assert(point == expected);
}

void test_forward(Thetas<double> thetas) {
  External::Robot forward_robot;
  External::slider_positions positions = {thetas.theta_1, thetas.theta_3,
                                          thetas.theta_4, thetas.theta_2,
                                          thetas.theta_5};
  External::Point p = External::get_end_effector(positions, forward_robot);
  Parameters<double> params = get_default_parameters<double>();
  Transform<double> T_EE = get_end_effector(thetas, params);
  assert(is_close(T_EE.p.x, p.x));
  assert(is_close(T_EE.p.y, p.y));
  assert(is_close(T_EE.p.z, p.z));
}
int main() {
  test_get_linkage_B();
  test_get_linkage_C();
  Thetas<double> thetas = {100, 100, 100, 100, 0};
  test_forward(thetas);
  thetas = {100, 100, 100, 100, 2};
  test_forward(thetas);
  thetas = {130, 120, 110, 90, 30};
  test_forward(thetas);
  thetas = {100, 130, 70, 80, 20};
  test_forward(thetas);
  thetas = {150, 80, 80, 100, 0};
  test_forward(thetas);
  thetas = {113, 113, 146.21, 149.52};
  test_forward(thetas);

  return 0;
}
