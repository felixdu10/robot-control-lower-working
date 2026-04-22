#include "Point.h"
#include "Robot.h"
#include "NewTransform.h"
#include "kinematic_structs.h"
#include <cassert>
#include <cmath>
#include <cstdlib>

#ifndef FORWARD_KINEMATICS
#define FORWARD_KINEMATICS

double highest_z = 0;
int highest_count = 0;
int count = 0;

Point get_joint(bool is_left_joint, double midpoint_distance,
                double transmission_length, double proximal_length, Point base,
                Point slider, linkage_array& robot_linkage) {
  double theta =
      std::acos((pow(midpoint_distance, 2) - pow(transmission_length, 2) -
                 pow((base - slider).magnitude(), 2)) /
                (-2 * transmission_length * (base - slider).magnitude()));
  if (!is_left_joint) {
    theta *= -1;
  }
  NewTransform rotate_z(0, 0, theta, 0, 0, 0);
  Point slider_to_base_vec = (base - slider).normalize();
  Point rotated_left_to_base_vec = rotate_z * slider_to_base_vec;
  Point midpoint = transmission_length * rotated_left_to_base_vec + slider;

  Point base_to_midpoint_vec = (midpoint - base).normalize();
  Point joint = proximal_length * base_to_midpoint_vec + base;
    
  if (is_left_joint) {
    robot_linkage.left_midpoint = midpoint;
    robot_linkage.left_joint = joint;
    //std::cout << "left midpoint" << std::endl;
  } else {
    robot_linkage.right_midpoint = midpoint;
    robot_linkage.right_joint = joint;
    //std::cout << "right midpoint" << std::endl;
  }
  //midpoint.print();

  return joint;
}

Point get_linkage_end_effector(bool is_upper, Point left_slider,
                               Point right_slider, Point base,
                               double transmission_length, double proximal_length,
                               double distal_length, double midpoint_distance,
                               Point end_effector_vect, double z, Robot& robot) {
  Point left_joint = get_joint(true, midpoint_distance, transmission_length,
                               proximal_length, base, left_slider, is_upper ? robot.top_linkage : robot.bottom_linkage);
  //std::cout << "left joint" << std::endl;
  //left_joint.print();
  Point right_joint = get_joint(false, midpoint_distance, transmission_length,
                                proximal_length, base, right_slider, is_upper ? robot.top_linkage : robot.bottom_linkage);
  //std::cout << "right joint" << std::endl;
  //right_joint.print();
  double h = std::sqrt(pow(distal_length, 2) -
                      pow((right_joint - left_joint).magnitude(), 2) / 4);
  
  // std::cout << "end effector values midpoint" << std::endl;
  // std::cout << "h " << h << std::endl;
  Point midpoint = (right_joint - left_joint) * 0.5;
  // midpoint.print();
  Point perpendicular_vector = {.x = -midpoint.y, .y = midpoint.x, .z = 0};
  // perpendicular_vector.print();
  Point resized_perp_vec = perpendicular_vector.normalize() * h;
  // resized_perp_vec.print();
  Point end_effector = (left_joint + right_joint) * 0.5 + resized_perp_vec;
  //extended vector for upper linkage
  double extended_h = h + end_effector_vect.magnitude();
  Point extended_resized_perp_vec = perpendicular_vector.normalize() * extended_h;
  Point extended_end_effector = (left_joint + right_joint) * 0.5 + extended_resized_perp_vec;
   
  //std::cout << "end effector without extension" << std::endl;
  //end_effector.print();
  if(is_upper) {
    robot.top_linkage.extended_end_effector = extended_end_effector;
    robot.top_linkage.extended_end_effector.z = z;
    robot.top_linkage.linkage_end_effector = end_effector;
    robot.top_linkage.linkage_end_effector.z = z;
    end_effector = extended_end_effector;
  } else {
    robot.bottom_linkage.linkage_end_effector = end_effector;
    robot.bottom_linkage.linkage_end_effector.z = z;
  }
  if (!is_upper) {
    end_effector = ((end_effector - left_joint).normalize() *
                    (distal_length + end_effector_vect.magnitude())) +
                   left_joint;
    robot.bottom_linkage.extended_end_effector = end_effector;
    robot.bottom_linkage.extended_end_effector.z = z;
  }
  // end_effector.print();
  end_effector.z = z;
  return end_effector;
}

Point get_needle_point_based_on_end_effector_positions(
    Point upper_linkage_end_effector, Point lower_linkage_end_effector,
    double needle_extension, Robot& robot) {
  //z direction is the direction of the needle towards the sky
  Point z_prime =
      (upper_linkage_end_effector - lower_linkage_end_effector).normalize();
  count++;
  //double z = z_prime.z;
  // upper_linkage_end_effector.print();
  // lower_linkage_end_effector.print();
  Point upper_base_with_corrected_z = UPPER_BASE;
  upper_base_with_corrected_z.z = upper_linkage_end_effector.z;
  Point x_prime = cross(upper_linkage_end_effector-upper_base_with_corrected_z, z_prime).normalize();
  Point y_prime = cross(z_prime, x_prime);
  robot.x_prime = x_prime;
  robot.y_prime = y_prime;
  robot.z_prime = z_prime;
  z_prime = z_prime * (LOWER_END_EFFECTOR_TO_NEEDLEPOINT.z - needle_extension);
  y_prime = y_prime * LOWER_END_EFFECTOR_TO_NEEDLEPOINT.y;
  x_prime = x_prime * LOWER_END_EFFECTOR_TO_NEEDLEPOINT.x;
  Point ee = lower_linkage_end_effector + z_prime + y_prime + x_prime;
  // This is for finding the best slider positions. Edit the condition to search
  // for different things
  /*
  if(z > highest_z && abs(ee.x) < .1) {
    std::cout << z << " "<< ee.x << std::endl;
    highest_z = z;
    highest_count = count;
  }*/
  return ee;
}

Point get_end_effector(Point left, Point left_middle, Point right_middle,
                       Point right, Point top_base, Point bottom_base,
                       double needle_extension, Robot& robot) {
  // Z coordinates will be determined in post to make calculations easier, as
  // the z component is static and pre-determined.
  assert(left.z == 0 && right.z == 0 && right_middle.z == 0 &&
         left_middle.z == 0 && top_base.z == 0 && bottom_base.z == 0);
  Point upper_linkage_end_effector = get_linkage_end_effector(
      true, left, right, top_base, UPPER_TRANSMISSION_LENGTH,
      UPPER_PROXIMAL_LENGTH, UPPER_DISTAL_LENGTH, UPPER_MIDPOINT_DISTANCE,
      UPPER_END_EFFECTOR_VECTOR, UPPER_LINKAGE_Z, robot);

  //std::cout << "upper linkage end effector" << std::endl;
  //upper_linkage_end_effector.print();
  Point lower_linkage_end_effector = get_linkage_end_effector(
      false, left_middle, right_middle, bottom_base, LOWER_TRANSMISSION_LENGTH,
      LOWER_PROXIMAL_LENGTH, LOWER_DISTAL_LENGTH, LOWER_MIDPOINT_DISTANCE,
      LOWER_END_EFFECTOR_VECTOR, LOWER_LINKAGE_Z, robot);
  //std::cout << "lower linkage end effector" << std::endl;
  //lower_linkage_end_effector.print();
  Point needle_point = get_needle_point_based_on_end_effector_positions(
      upper_linkage_end_effector, lower_linkage_end_effector, needle_extension, robot);
  return needle_point;
}

Point get_end_effector(slider_positions sliders, Robot& robot) {
  return get_end_effector({sliderXs[0], sliders.left_slider_y, 0}, {sliderXs[1], sliders.left_middle_slider_y, 0}, {sliderXs[2], sliders.right_middle_slider_y, 0}, {sliderXs[3], sliders.right_slider_y, 0}, UPPER_BASE, LOWER_BASE, sliders.needle_extension, robot);
}

#endif