/**
 * @file kinematic_structs.h
 * @author Tyler Lehrfeld
 * @brief This file will hold struct definitions and constants needed for
 * forward and inverse kinematics
 * @version 0.1
 * @date 2025-03-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef KINEMATIC_STRUCTS
#define KINEMATIC_STRUCTS

#include "Point.h"
#include <ostream>
#include <sstream>

const double UPPER_TRANSMISSION_LENGTH = 115;
const double UPPER_PROXIMAL_LENGTH = 116;
const double UPPER_DISTAL_LENGTH = 125;
const double UPPER_MIDPOINT_DISTANCE = 58;
const double LOWER_TRANSMISSION_LENGTH = 110;
const double LOWER_PROXIMAL_LENGTH = 92;
const double LOWER_DISTAL_LENGTH = 99;
const double LOWER_MIDPOINT_DISTANCE = 46;
const double UPPER_LINKAGE_Z = 31.5;
const double LOWER_LINKAGE_Z = 0;
const Point LOWER_END_EFFECTOR_TO_NEEDLEPOINT = {.x = 0, .y = 47.7, .z = -64.9};
const double UPPER_TO_LOWER_BASE_LENGTH = 38;
const Point LOWER_BASE = {0, 186 + UPPER_TO_LOWER_BASE_LENGTH, 0};
const Point UPPER_BASE = {0, 186, 0};
const Point UPPER_END_EFFECTOR_VECTOR = {0, 19.25, 0};
const Point LOWER_END_EFFECTOR_VECTOR = {14, 0, 0};
const double sliderXs[4] = {-63, -21, 21, 63};
const double BASE_TO_SLIDER_MIN = 48;
const double BASE_TO_SLIDER_MAX = 173;
const double HALF_SLIDER_WIDTH = 6.35;
const double MIN_NEEDLE_EXTENSION = 0;
// TODO: make this accurate
const double MAX_NEEDLE_EXTENSION = 150;
// TODO: make this accurate
const double MAX_LINKAGE_END_EFFECTOR_DISTANCE = 50;

/**
 * @brief A struct that defines how the robot will approach a target
 *
 */
struct approach_definition {
  // in mm
  Point target;
  /**
   * Imagine looking at the workspace from behind the robot such that the motors
   * are close and the needle is far. z is up, x is right, and y is forward.
   * Using spherical angle coordinates
   * theta is the angle of the approach away from the z axis: the polar angle θ
   * between the radial line (needle) and a given polar axis (z). phi is the
   * angle to rotate on the x-y plane:  the angle of rotation of the radial line
   * (needle) around the polar axis (z). angles are in degrees
   */
  double theta;
  double phi;
};

/**
 * @brief A struct that defines how the robot will approach a target
 *
 */
struct target_and_injection_point_approach {
  Point target;
  Point injection_point;
};

/**
 * @brief a struct that contains the y positions of all the sliders
 *
 */
struct slider_positions {
  double left_slider_y;
  double left_middle_slider_y;
  double right_middle_slider_y;
  double right_slider_y;
  double needle_extension;

  std::string get_slider_string(bool my_coords) {
    std::stringstream slider_string_stream;
    if (my_coords) {
      slider_string_stream << left_slider_y << std::endl
                           << left_middle_slider_y << std::endl
                           << right_middle_slider_y << std::endl
                           << right_slider_y << std::endl
                           << needle_extension << std::endl;

    } else {
      double _right = BASE_TO_SLIDER_MAX - right_slider_y - HALF_SLIDER_WIDTH;
      double _left = BASE_TO_SLIDER_MAX - left_slider_y - HALF_SLIDER_WIDTH;
      double _right_middle =
          BASE_TO_SLIDER_MAX - right_middle_slider_y - HALF_SLIDER_WIDTH;
      double _left_middle =
          BASE_TO_SLIDER_MAX - left_middle_slider_y - HALF_SLIDER_WIDTH;

      slider_string_stream << _left << std::endl
                           << _left_middle << std::endl
                           << _right_middle << std::endl
                           << _right << std::endl
                           << needle_extension << std::endl;
    }
    return slider_string_stream.str();
  }

  void print(bool my_coords) { std::cout << get_slider_string(my_coords); }
};

struct linkage_end_effectors {
  Point upper;
  Point lower;
};

struct linkage_array {
  Point extended_end_effector;
  Point linkage_end_effector;
  Point base;
  Point left_joint;
  Point right_joint;
  Point left_midpoint;
  Point right_midpoint;
};

const double precision = .01;

inline bool isclose(double a, double b) {
  return a + precision > b && a - precision < b;
}
#endif
