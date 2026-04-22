#include "auto.h"
#include "kinematics.h"
#include "templated_classes/Templated_Point.h"

void test_kinematic_derivatives() {
  constexpr int num_params = 5;
  Thetas<Auto<num_params, double>> t;
  t.theta_1.epsilon[0] = 1;
  t.theta_2.epsilon[1] = 1;
  t.theta_3.epsilon[2] = 1;
  t.theta_4.epsilon[3] = 1;
  t.theta_5.epsilon[4] = 1;
  Parameters<Auto<num_params, double>> params =
      get_default_parameters<Auto<num_params, double>>();
  Transform<Auto<num_params, double>> T = get_end_effector(t, params);
  std::cout << T.p.x.epsilon[0] << std::endl;
}

void test_basic_equation() {
  Auto<1, double> seed_var(10);
  seed_var.epsilon[0] = 1;
  Auto<1, double> y(1);
  Auto<1, double> z(3);
  Auto<1, double> result = sin(seed_var * y) + (seed_var * z) * cos(y + z);
  std::cout << result.epsilon[0] << std::endl;
}

void test_magnitude() {
  Point<Auto<1, double>> p;
  Auto<1, double> x = 2;
  Auto<1, double> y = 4.5;
  Auto<1, double> z = -14.5;
  x.epsilon[0] = 1;
  p = {x, y, z};
  Auto<1, double> val = p.magnitude();
  std::cout << val.epsilon[0] << std::endl;
}
int main() {
  test_basic_equation();
  test_magnitude();
  // test_kinematic_derivatives();
}
