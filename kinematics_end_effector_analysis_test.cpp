#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "kinematic_structs.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <ostream>

int main() {
Robot forward_robot;
Robot inverse_robot;
  std::ofstream output_csv;
  output_csv.open("output.csv");
  bool Point_1 = false;
  approach_definition def = {{0, 356, -115},
                             0,
                             0};
  slider_positions positions_1 =
      inverse_kinematics(def, NewTransform(0, 0, 0, 0, 0, 0), inverse_robot);
  //positions_1.print();
  Point ee_1 = get_end_effector({sliderXs[0], positions_1.left_slider_y, 0},
                   {sliderXs[1], positions_1.left_middle_slider_y, 0},
                   {sliderXs[2], positions_1.right_middle_slider_y, 0},
                   {sliderXs[3], positions_1.right_slider_y, 0}, UPPER_BASE,
                   LOWER_BASE, positions_1.needle_extension, forward_robot);
  /*Point ee_2 = get_end_effector({sliderXs[0], positions_1.left_slider_y + .03, 0},
        {sliderXs[1], positions_1.left_middle_slider_y, 0},
        {sliderXs[2], positions_1.right_middle_slider_y, 0},
        {sliderXs[3], positions_1.right_slider_y, 0}, UPPER_BASE,
        LOWER_BASE, positions_1.needle_extension, forward_robot);
std::cout << (ee_1 - ee_2).magnitude() << std::endl;*/
        def.target.y=456; 
  slider_positions positions_2 =
      inverse_kinematics(def, NewTransform(0, 0, 0, 0, 0, 0), inverse_robot);
  //positions_2.print();

  get_end_effector({sliderXs[0], positions_2.left_slider_y, 0},
                   {sliderXs[1], positions_2.left_middle_slider_y, 0},
                   {sliderXs[2], positions_2.right_middle_slider_y, 0},
                   {sliderXs[3], positions_2.right_slider_y, 0}, UPPER_BASE,
                   LOWER_BASE, positions_2.needle_extension, forward_robot)
      .print();
  float max_slider_dif =
      std::max(abs(positions_1.left_slider_y - positions_2.left_slider_y),
               std::max(abs(positions_1.left_middle_slider_y -
                            positions_2.left_middle_slider_y),
                        std::max(abs(positions_1.right_middle_slider_y -
                                     positions_2.right_middle_slider_y),
                                 abs(positions_1.right_slider_y -
                                     positions_2.right_slider_y))));
  float left_pm =
      positions_1.left_slider_y < positions_2.left_slider_y ? 1 : -1;
  float left_middle_pm =
      positions_1.left_middle_slider_y < positions_2.left_middle_slider_y ? 1
                                                                          : -1;
  float right_middle_pm =
      positions_1.right_middle_slider_y < positions_2.right_middle_slider_y
          ? 1
          : -1;
  float right_pm =
      positions_1.right_slider_y < positions_2.right_slider_y ? 1 : -1;
  float offset = 0;
  Point prev = get_end_effector(
      {sliderXs[0], positions_1.left_slider_y + offset, 0},
      {sliderXs[1], positions_1.left_middle_slider_y + offset, 0},
      {sliderXs[2], positions_1.right_middle_slider_y + offset, 0},
      {sliderXs[3], positions_1.right_slider_y + offset, 0}, UPPER_BASE,
      LOWER_BASE, positions_1.needle_extension, forward_robot);
  offset += .03;
  while (offset < max_slider_dif) {
    float left_slider_y_plus_offset = positions_1.left_slider_y + offset * left_pm;
    float left_middle_slider_y_plus_offset = positions_1.left_middle_slider_y + offset * left_pm;
    float right_middle_slider_y_plus_offset = positions_1.right_middle_slider_y + offset * left_pm;
    float right_slider_y_plus_offset = positions_1.right_slider_y + offset * left_pm;
    if(left_slider_y_plus_offset - positions_2.left_slider_y * left_pm > 0) {
        left_slider_y_plus_offset = positions_2.left_slider_y;
    }
    if(left_middle_slider_y_plus_offset - positions_2.left_middle_slider_y * left_middle_pm > 0) {
        left_middle_slider_y_plus_offset = positions_2.left_middle_slider_y;
    }
    if(right_middle_slider_y_plus_offset - positions_2.right_middle_slider_y * right_middle_pm > 0) {
        right_middle_slider_y_plus_offset = positions_2.right_middle_slider_y;
    }
    if(right_slider_y_plus_offset - positions_2.right_slider_y * right_pm > 0) {
        right_slider_y_plus_offset = positions_2.right_slider_y;
    }
    Point new_point = get_end_effector(
        {sliderXs[0], left_slider_y_plus_offset, 0},
        {sliderXs[1],
         left_middle_slider_y_plus_offset, 0},
        {sliderXs[2],
         right_middle_slider_y_plus_offset, 0},
        {sliderXs[3],right_slider_y_plus_offset, 0},
        UPPER_BASE, LOWER_BASE, positions_1.needle_extension, forward_robot);
    new_point.print();
    output_csv << offset << ", " << abs((new_point - prev).x) << ", " << abs((new_point - prev).y) << ", "<< abs((new_point - prev).z) << std::endl;
    prev = new_point;
    offset += .03;
    //if(offset > max_slider_dif) {
        /*std::cout << left_slider_y_plus_offset << " " << left_middle_slider_y_plus_offset << " " << right_middle_slider_y_plus_offset << " " << right_slider_y_plus_offset << std::endl;
        get_end_effector(
            {sliderXs[0], left_slider_y_plus_offset, 0},
            {sliderXs[1],
             left_middle_slider_y_plus_offset, 0},
            {sliderXs[2],
             right_middle_slider_y_plus_offset, 0},
            {sliderXs[3],right_slider_y_plus_offset, 0},
            UPPER_BASE, LOWER_BASE, positions_2.needle_extension).print();*/
    //}
  }

  //prev.print();
  /*double left_slider_y = 106.565;
  double left_middle_slider_y =  BASE_TO_SLIDER_MAX-34.5634;
  double right_middle_slider_y =BASE_TO_SLIDER_MAX-29.8236;
  double right_slider_y = 106.565;

  int prev_count = 0;
  Point end_effector = get_end_effector(
      {-63, left_slider_y, 0}, {-21, left_middle_slider_y, 0},
      {21.07, right_middle_slider_y, 0}, {63, right_slider_y, 0},
      UPPER_BASE, LOWER_BASE, 0);
  end_effector.print();*/

  /**
   * Uncomment the two loops for the slider lengths you want to find the ideal
   * slider length for according to your condition
   *
   */
  /*//for(left_slider_y = 100; left_slider_y < 120; left_slider_y += .03) {
      for(left_middle_slider_y = BASE_TO_SLIDER_MIN+HALF_SLIDER_WIDTH;
  left_middle_slider_y < BASE_TO_SLIDER_MAX - HALF_SLIDER_WIDTH;
  left_middle_slider_y += .03) { for(right_middle_slider_y = BASE_TO_SLIDER_MIN
  + HALF_SLIDER_WIDTH; right_middle_slider_y < BASE_TO_SLIDER_MAX -
  HALF_SLIDER_WIDTH; right_middle_slider_y += .03) {
              //for(right_slider_y = 100; right_slider_y < 120; right_slider_y
  += .03) { Point end_effector = get_end_effector(
                      {-63, left_slider_y, 0}, {-21, left_middle_slider_y, 0},
                      {21.07, right_middle_slider_y, 0}, {63, right_slider_y,
  0}, UPPER_BASE, LOWER_BASE, 0); if(highest_count != prev_count) { output_csv
  << (BASE_TO_SLIDER_MAX) - left_slider_y <<", " <<(BASE_TO_SLIDER_MAX) -
  left_middle_slider_y<< ", " <<(BASE_TO_SLIDER_MAX) - right_middle_slider_y <<
  ", "<< (BASE_TO_SLIDER_MAX) - right_slider_y << std::endl; prev_count =
  highest_count;
                  }
              }
          }
      //}
  //}

  //end_effector.print();*/
  output_csv.close();
}