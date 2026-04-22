#include "./scripts/pivot_needle.h"
#include "./scripts/pivot_robot.h"
#include "NewTransform.h"
#include "PointCloudTransform.h"
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include <fstream>
#include <iostream>
#include <string>

// slider_positions home_positions = {
//     BASE_TO_SLIDER_MAX - 53.92 - HALF_SLIDER_WIDTH,
//     BASE_TO_SLIDER_MAX - 19.94923 - HALF_SLIDER_WIDTH,
//     BASE_TO_SLIDER_MAX - 16.63853 - HALF_SLIDER_WIDTH,
//     BASE_TO_SLIDER_MAX - 53.92 - HALF_SLIDER_WIDTH,
//     20
// };

NewTransform get_average_transform(vector<NewTransform> transforms) {
  Quaternion q_sums;
  Matrix t_sums(3, 1, {0, 0, 0});
  for (int i = 0; i < transforms.size(); i++) {
    Quaternion q_i = transforms[i].to_quaternion();
    q_sums.w += q_i.w;
    q_sums.x += q_i.x;
    q_sums.y += q_i.y;
    q_sums.z += q_i.z;
    t_sums = t_sums + transforms[i].to_transform().p_AB;
  }
  q_sums.w = q_sums.w / transforms.size();
  q_sums.x = q_sums.x / transforms.size();
  q_sums.y = q_sums.y / transforms.size();
  q_sums.z = q_sums.z / transforms.size();
  NewTransform T;
  T.from_quaternion(q_sums);
  T.matrix[0][3] = t_sums.matrixArray[0] / transforms.size();
  T.matrix[1][3] = t_sums.matrixArray[1] / transforms.size();
  T.matrix[2][3] = t_sums.matrixArray[2] / transforms.size();
  return T;
};

int main() {
  bool up_left_near, up_right_near, low_left_near, low_right_near;
  std::ofstream results_file("dummy.txt");

  // init_galil(1);
  // std::cout
  //    << "Homing high. Enter 1 if the slider is behind the limit switch: A F"
  //    << std::endl;
  // std::cin >> up_left_near;
  // std::cin >> up_right_near;
  //// HomeUpBlocking(up_left_near, up_right_near);
  // std::cout
  //     << "Homing low. Enter 1 if the slider is behind the limit switch: B E"
  //     << std::endl;
  // std::cin >> low_left_near;
  // std::cin >> low_right_near;
  //  HomeLowBlocking(low_left_near, low_right_near);
  //  stop_galil();
  /*init_galil(3);
  GoToUpBlocking(60,60);
  stop_galil();*/
  //  std::ifstream grid_file("gridy.txt");

  // if (!grid_file.is_open()) {
  //   std::cout << "grid file not open";
  //   return -1;
  // }
  // std::ifstream slider_file("sliders.txt");
  // if (!slider_file.is_open()) {
  //   std::cout << "slider file not open";
  //   return -1;
  // }
  std::string command = "";
  // std::cout << "Begin needle marker calibration? After continuing, start
  // pivoting the needle such that new optical frames can be read." <<
  // std::endl;
  std::cin >> command;
  std::cout << "getting transforms from out.txt" << std::endl;
  vector<NewTransform> F_M2Ns;
  Matrix p_bottom_linkage_home_in_optical_coordinates;
  do {
    NewTransform F_M2N = get_needle_pivot_transform(
        p_bottom_linkage_home_in_optical_coordinates);
    F_M2N.print();
    std::cout
        << "Begin needle marker calibration again (r: redo, a: add  another, "
           "c: continue to robot pivot)? After continuing, start pivoting the  "
           "needle such that new optical frames can be read"
        << std::endl;
    std::cin >> command;
    if (command == "a" || command == "c") {
      F_M2Ns.push_back(F_M2N);
    }
  } while (command != "c");

  NewTransform F_M2N = get_average_transform(F_M2Ns);
  // NewTransform F_M2N(0, 0, M_PI, .00558085, 28.3789, -5.49509);
  results_file << "F_M2N" << std::endl << F_M2N.to_string();
  // NewTransform F_M2N = F_M2Ns[0];
  /*vector<NewTransform> F_M1Rs;
  std::cout << "Begin robot base calibration? After continuing, start pivoting
  around the robot base." << std::endl; std::cin >> command; do { NewTransform
  F_M1R = get_robot_pivot_transform(F_M2N); F_M1R.print(); std::cout << "Begin
  robot base calibration again? (r: redo, a: add another trial, c: continue to
  benchtop test) After continuing, start pivoting around the robot base. enter c
  to move on" << std::endl; std::cin >> command; if(command == "a" || command ==
  "c") { F_M1Rs.push_back(F_M1R);
      }
  } while(command != "c");
  NewTransform F_M1R = get_average_transform(F_M1Rs);
  //NewTransform F_M1R = F_M1Rs[0];*/
  // HomeUpBlocking(1,1);
  // stop_galil();
  Robot forward_robot;
  // Point target = get_end_effector(home_positions, forward_robot);
  // home_positions.needle_extension -= 1;
  // Point injection = get_end_effector(home_positions, forward_robot);
  // inverse_kinematics({target, injection}, forward_robot);
  Matrix rot(vector<Matrix>({forward_robot.x_prime.to_matrix(),
                             forward_robot.y_prime.to_matrix(),
                             forward_robot.z_prime.to_matrix()}));
  Transform expected_F_RN(
      rot, forward_robot.bottom_linkage.extended_end_effector.to_matrix());
  NewTransform F_RN(expected_F_RN);
  NewTransform F_NR = F_RN.inverse();
  int num_measurements_needed = 10;
  int num_measurements_taken = 0;
  vector<NewTransform> F_OM1_list;
  vector<NewTransform> F_OM2_list;
  NewTransform previous_F_OM1;
  NewTransform previous_F_OM2;
  std::cout << "collect home positions" << std::endl;
  std::cin >> command;
  while (num_measurements_taken < num_measurements_needed) {

    NewTransform F_OM2;
    NewTransform F_OM1;

    std::string file_path = "./out.txt";
    read_transform(file_path, F_OM1, true);
    read_transform(file_path, F_OM2, false);
    if (!(F_OM1 == previous_F_OM1 || F_OM2 == previous_F_OM2)) {
      F_OM1_list.push_back(F_OM1);
      F_OM2_list.push_back(F_OM2);
      num_measurements_taken++;
      std::cout << "measurement " << num_measurements_taken << " taken"
                << std::endl;
      previous_F_OM2 = F_OM2;
      previous_F_OM1 = F_OM1;
    }
    delay_ms(200);
  }
  NewTransform F_OM1 = get_average_transform(F_OM1_list);
  NewTransform F_OM2 = get_average_transform(F_OM2_list);
  NewTransform home_F_OM2 = F_OM2;
  results_file << "F_OM home" << std::endl << home_F_OM2.to_string();
  (F_OM1.inverse() * F_OM2).print();
  (F_OM1.inverse() * F_OM2 * F_M2N).print();
  // 5 72 -42.27
  NewTransform F_M1R = F_OM1.inverse() * F_OM2 * F_M2N * F_NR;
  // F_M1R.matrix[0][3] = -37.5;
  // F_M1R.matrix[1][3] = -5;
  // F_M1R.matrix[2][3] = -71.99;
  // F_M1R.matrix[0][0] = -1;
  // F_M1R.matrix[0][1] = 0;
  // F_M1R.matrix[0][2] = 0;
  // F_M1R.matrix[1][0] = 0;
  // F_M1R.matrix[1][1] = -1;
  // F_M1R.matrix[1][2] = 0;
  // F_M1R.matrix[2][0] = 0;
  // F_M1R.matrix[2][1] = 0;
  // F_M1R.matrix[2][2] = 1;
  results_file << "F_M1R" << std::endl << F_M1R.to_string();
  F_RN.print();
  NewTransform F_OM1_inv = F_OM1.inverse();
  (F_M1R.inverse() * F_OM1_inv * F_OM2 * F_M2N).print();
  Point last_point = {0, 0, 0};
  double mag_tot = 0;
  int num_trials = 0;
  // init_galil(3);
  while (true) {
    Robot inverse_robot;
    std::string line;
    // grid_file >> line;
    if (line == "") {
      break;
    }
    std::string x = line.substr(1, line.find(",") - 1);
    std::string y =
        line.substr(line.find(",") + 1, line.length() - line.find(",") - 2);
    std::cout << x << ", " << y << std::endl;
    results_file << x << ", " << y << std::endl;
    approach_definition def = {{std::stod(x), std::stod(y), -64.9}, 0, 0};
    slider_positions position =
        inverse_kinematics(def, NewTransform(0, 0, 0, 0, 0, 0), inverse_robot);
    inverse_robot.bottom_linkage.extended_end_effector.print_desmos();
    position.print(false);
    std::cout << "move robot and take readings?" << std::endl;
    delay_ms(500);
    // std::cin >> command;
    if (command == "q") {
      break;
    }

    // move_robot_with_slider_positions(position);
    vector<NewTransform> F_OM1s;
    vector<NewTransform> F_OM2s;
    NewTransform F_OM1(0, 0, 0, 0, 0, 0);
    NewTransform F_OM2(0, 0, 0, 0, 0, 0);
    while (F_OM1s.size() < 10) {
      NewTransform newF1(0, 0, 0, 0, 0, 0);
      read_transform("./out.txt", newF1, true);
      NewTransform newF2(0, 0, 0, 0, 0, 0);
      read_transform("./out.txt", newF2, false);
      if (!(newF1 == F_OM1 || newF2 == F_OM2)) {
        F_OM1s.push_back(newF1);
        F_OM1 = newF1;
        F_OM2s.push_back(newF2);
        F_OM2 = newF2;
      }
      delay_ms(100);
    }
    F_OM1 = get_average_transform(F_OM1s);
    F_OM2 = get_average_transform(F_OM2s);
    results_file << "F_OM1" << std::endl << F_OM1.to_string();
    results_file << "F_OM2" << std::endl << F_OM2.to_string();

    Matrix rot(vector<Matrix>({inverse_robot.x_prime.to_matrix(),
                               inverse_robot.y_prime.to_matrix(),
                               inverse_robot.z_prime.to_matrix()}));
    Transform expected_F_RN(
        rot, inverse_robot.bottom_linkage.extended_end_effector.to_matrix());
    NewTransform New_expected_F_RN(expected_F_RN);
    NewTransform F_OM1_inverse = F_OM1.inverse();
    NewTransform actual_F_RN = F_M1R.inverse() * F_OM1_inverse * F_OM2 * F_M2N;
    std::cout << "Measured F_RN: " << std::endl;
    actual_F_RN.print();
    results_file << "F_RN measured" << std::endl << actual_F_RN.to_string();
    std::cout << "Expected F_RN: " << std::endl;
    New_expected_F_RN.print();
    std::cout << "difference transform" << std::endl;
    (New_expected_F_RN.inverse() * actual_F_RN).print();
    results_file << "F_RN expected" << std::endl
                 << New_expected_F_RN.to_string();
    Point needle = LOWER_END_EFFECTOR_TO_NEEDLEPOINT;
    needle.z = -115;
    std::cout << "Expected needle: " << std::endl;
    Point expected = (New_expected_F_RN * needle);
    results_file << "expected needle" << std::endl
                 << (New_expected_F_RN * needle).to_string(false);
    expected.print();
    std::cout << "Actual needle: " << std::endl;
    Point actual = (actual_F_RN * needle);
    results_file << "actual needle" << std::endl
                 << (actual_F_RN * needle).to_string(false);
    actual.print();
    std::cout << "difference vector: " << std::endl;
    (actual - expected).print();

    results_file << "difference vector" << std::endl
                 << (actual - expected).to_string(false);
    std::cout << "difference magnitude: " << std::endl;
    std::cout << (actual - expected).magnitude() << std::endl;

    results_file << "difference magnitude" << std::endl
                 << (actual - expected).magnitude() << std::endl;
    /*Point origin = {0,0,0};
    Point needle = LOWER_END_EFFECTOR_TO_NEEDLEPOINT;
    needle.z -= 150;
    Point new_pos = (F_OM1.inverse()* F_OM2 * needle);
    (last_point - new_pos).print();
    //std::cout << "magnitude diff: " << (last_point - new_pos).magnitude() <<
    std::endl; if(num_trials != 0) mag_tot += (last_point -
    new_pos).magnitude(); num_trials++; last_point = new_pos;*/
  }
  // std::cout << "average error over " << num_trials-1 << " trials of 10
  // samples for each trial: " << mag_tot / (num_trials-1) << std::endl;
  // stop_galil();
  return 0;
}
