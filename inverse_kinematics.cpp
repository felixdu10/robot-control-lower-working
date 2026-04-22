#include "inverse_kinematics.h"
#include "NewTransform.h"
#include "Point.h"
#include "Robot.h"
#include "kinematic_structs.h"
#include <cassert>
#include <cmath>
#include <stdexcept>

/**
 * @brief Get the slider positions of the robot based on the approach angle to a
 * point RELATIVE TO THE PATIENT.
 *
 * @param needle_to_patient_approach A struct that defines a point and approach
 * angle for the needle path
 * @param T_RP Transform from Patient frame to robot frame
 * (T_RP*v_patient=v_robot)
 * @return slider_positions
 */
slider_positions
inverse_kinematics(approach_definition needle_to_patient_approach,
                   NewTransform T_RP, Robot &robot) {
  Point mock_injection_point = {sin(needle_to_patient_approach.phi) *
                                    cos(needle_to_patient_approach.theta),
                                sin(needle_to_patient_approach.phi) *
                                    sin(needle_to_patient_approach.theta),
                                cos(needle_to_patient_approach.phi)};

  mock_injection_point =
      mock_injection_point + needle_to_patient_approach.target;
  return inverse_kinematics(
      {T_RP * needle_to_patient_approach.target, T_RP * mock_injection_point},
      robot);
}

/**
 * @brief Get the slider positions of the robot based on a target and injection
 * point RELATIVE TO THE PATIENT.
 *
 * @param needle_to_patient_approach A struct that defines target and injection
 * points
 * @param T_RP Transform from Patient frame to robot frame
 * (T_RP*v_patient=v_robot)
 * @return slider_positions
 */
slider_positions invinverse_kinematics(
    target_and_injection_point_approach needle_to_patient_approach,
    NewTransform T_RP, Robot &robot) {
  return inverse_kinematics({T_RP * needle_to_patient_approach.injection_point,
                             T_RP * needle_to_patient_approach.target},
                            robot);
}

/**
 * @brief Get the slider positions of the robot based on the injection point and
 * target in ROBOT COORDINATES
 *
 * @param needle_to_patient_approach
 * @return slider_positions
 */
slider_positions inverse_kinematics(
    target_and_injection_point_approach needle_to_patient_approach,
    Robot &robot) {

  double needle_extension = 0;
  linkage_end_effectors end_effectors =
      get_linkage_end_effector(needle_to_patient_approach, LOWER_LINKAGE_Z,
                               UPPER_LINKAGE_Z, needle_extension, robot);
  robot.top_linkage.linkage_end_effector = end_effectors.upper;
  robot.bottom_linkage.linkage_end_effector = end_effectors.lower;
  // std::cout << "lower and upper end effectors" << std::endl;
  // end_effectors.lower.print();
  // end_effectors.upper.print();
  slider_positions sliders;
  sliders.needle_extension = needle_extension;
  get_slider_positions(sliders, end_effectors.lower, false, robot);
  get_slider_positions(sliders, end_effectors.upper, true, robot);
  robot.sliders = sliders;
  return sliders;
}

void check_end_effector_dists(Point upper_end_effector,
                              Point lower_end_effector) {
  Point base = UPPER_BASE;
  if ((upper_end_effector - base).magnitude() >
      UPPER_DISTAL_LENGTH + UPPER_PROXIMAL_LENGTH +
          static_cast<Point>(UPPER_END_EFFECTOR_VECTOR).magnitude()) {
    throw std::runtime_error("The upper linkage is too far from the robot");
  }
  base = LOWER_BASE;
  if ((lower_end_effector - base).magnitude() >
      LOWER_DISTAL_LENGTH + LOWER_PROXIMAL_LENGTH +
          static_cast<Point>(LOWER_END_EFFECTOR_VECTOR).magnitude()) {
    throw std::runtime_error("The lower linkage is too far from the robot");
  }
  if (lower_end_effector.y < LOWER_BASE.y ||
      upper_end_effector.y < LOWER_BASE.y) {
    throw std::runtime_error("The end effectors are too close to the robot");
  }
  // TODO change 50 to a correct number
  Point upper_end_effector_flattened = upper_end_effector;
  upper_end_effector_flattened.z = 0;
  double mag = (upper_end_effector_flattened - lower_end_effector).magnitude();
  if (mag > 19) {
    std::cout << mag
              << ": too big of difference between lower and upper end effectors"
              << std::endl;
    throw std::runtime_error(

        "The end effectors are too far appart from each other");
  }
};
/**
 * @brief Get the linkage end effector based on its z value. See kinematics
 * report section 5.2
 *
 * @param robot_approach
 * @param z z value of end effector (0 or LOWER_LINKAGE_Z for lower,
 * UPPER_LINKAGE_Z for upper)
 * @param is_upper_linkage boolean flag for upper linkage vs lower linkage
 * (upper: true, lower: false)
 * @return Point
 */
linkage_end_effectors
get_linkage_end_effector(target_and_injection_point_approach robot_approach,
                         double z_lower, double z_upper,
                         double &needle_extension, Robot &robot) {
  Point base = LOWER_BASE;
  base.z = z_lower;
  Point upper_base = UPPER_BASE;
  upper_base.z = UPPER_LINKAGE_Z;
  Point z_prime =
      (robot_approach.injection_point - robot_approach.target).normalize();
  Point needle_at_z =
      (robot_approach.injection_point - robot_approach.target) *
          ((UPPER_LINKAGE_Z - robot_approach.target.z) /
           ((robot_approach.injection_point - robot_approach.target).z)) +
      robot_approach.target;
  if ((needle_at_z - base).magnitude() == 0) {
    throw std::runtime_error("can't be on top of lower base");
  }
  Point x_prime = cross((needle_at_z - upper_base), z_prime).normalize();
  Point y_prime = cross(z_prime, x_prime).normalize();
  robot.x_prime = x_prime;
  robot.y_prime = y_prime;
  robot.z_prime = z_prime;
  double z_needle = (-robot_approach.target.z +
                     (LOWER_END_EFFECTOR_TO_NEEDLEPOINT.y * y_prime).z) /
                    z_prime.z;
  needle_extension = z_needle + LOWER_END_EFFECTOR_TO_NEEDLEPOINT.z;
  x_prime = x_prime * LOWER_END_EFFECTOR_TO_NEEDLEPOINT.x;
  y_prime = y_prime * LOWER_END_EFFECTOR_TO_NEEDLEPOINT.y;
  // we can only vary the needle's z coordinate in the needle frame
  z_prime = z_prime * -z_needle;
  Point lower_end_effector =
      robot_approach.target - x_prime - y_prime - z_prime;
  z_prime = z_prime.normalize();
  Point upper_end_effector =
      (((z_upper - z_lower) / z_prime.z)) * z_prime + lower_end_effector;
  robot.top_linkage.extended_end_effector = upper_end_effector;
  robot.bottom_linkage.extended_end_effector = lower_end_effector;
  check_end_effector_dists(upper_end_effector, lower_end_effector);
  assert(isclose(lower_end_effector.z, z_lower));
  assert(isclose(upper_end_effector.z, z_upper));
  upper_end_effector.z = 0;
  upper_base.z = 0;
  Point upper_end_effector_vector = UPPER_END_EFFECTOR_VECTOR;
  upper_end_effector = ((upper_end_effector - upper_base).normalize() *
                        ((upper_end_effector - upper_base).magnitude() -
                         upper_end_effector_vector.magnitude())) +
                       upper_base;

  Point lower_end_effector_vector = LOWER_END_EFFECTOR_VECTOR;
  lower_end_effector.z = 0;
  Point left_joint = intersection_of_two_circles(
      lower_end_effector, LOWER_BASE,
      LOWER_DISTAL_LENGTH + lower_end_effector_vector.magnitude(),
      LOWER_PROXIMAL_LENGTH, true);

  lower_end_effector =
      (lower_end_effector - left_joint).normalize() * LOWER_DISTAL_LENGTH +
      left_joint;
  lower_end_effector.z = z_lower;
  upper_end_effector.z = z_upper;
  return {upper_end_effector, lower_end_effector};
}

// Algorithm taken from https://paulbourke.net/geometry/circlesphere/
/**
 * @brief Get the points at the intersection of two circles. WARNING: only call
 * when you know there are two points of intersection. Only takes into account x
 * and y coordinates.
 *
 * @param center_a center point of the first circle
 * @param center_b center point of the second circle
 * @param radius_a radius of the first circle
 * @param radius_b radius of the second circle
 * @param left_point boolean flag if you want the leftmost (closest x to -inf):
 * true, or right point: false
 * @return Point
 */
Point intersection_of_two_circles(Point center_a, Point center_b,
                                  double radius_a, double radius_b,
                                  bool left_point) {

  center_a.z = 0;
  center_b.z = 0;
  // plus 3 for assurance that we are making triangles. Almost never should
  // our linkages be that close to eachother.
  double d = (center_a - center_b).magnitude();
  if (d + 3 > radius_b + radius_b || radius_b + 3 > d + radius_a ||
      radius_a + 3 > d + radius_b) {
    throw std::runtime_error("The circles do not intersect.");
  }
  double a = (pow(radius_a, 2) - pow(radius_b, 2) + pow(d, 2)) / (2 * d);
  double h = std::sqrt(pow(radius_a, 2) - pow(a, 2));
  Point P2 = (center_b - center_a) * (a / d) + center_a;
  Point P3A = {.x = P2.x + h * (center_b.y - center_a.y) / d,
               .y = P2.y - h * (center_b.x - center_a.x) / d,
               .z = 0};
  Point P3B = {.x = P2.x - h * (center_b.y - center_a.y) / d,
               .y = P2.y + h * (center_b.x - center_a.x) / d,
               .z = 0};
  if ((left_point && P3A.x < P3B.x) || (!left_point && P3A.x > P3B.x)) {
    return P3A;
  } else {
    return P3B;
  }
}

/**
 * @brief Get the y of slider based on midpoint location on the proxmial link.
 * This essentially solves for y of a circle of radius transmission and center
 * midpoint as a function of x.
 *
 * @param midpoint
 * @param x static x coordinate of the slider
 * @param transmission_length
 * @return double
 */
double get_y_of_slider_based_on_midpoint_location(Point midpoint, double x,
                                                  double transmission_length) {
  assert(midpoint.z == 0);
  return -1 * sqrt(pow(transmission_length, 2) - pow(x - midpoint.x, 2)) +
         midpoint.y;
}

/**
 * @brief Get the slider positions for a linkage based on the end effector
 * position.
 *
 * @param slider_positions_struct
 * @param end_effector
 * @param upper
 */
void get_slider_positions(slider_positions &slider_positions_struct,
                          Point end_effector, bool upper, Robot &robot) {
  end_effector.z = 0;
  Point left_joint = intersection_of_two_circles(
      end_effector, upper ? UPPER_BASE : LOWER_BASE,
      upper ? UPPER_DISTAL_LENGTH : LOWER_DISTAL_LENGTH,
      upper ? UPPER_PROXIMAL_LENGTH : LOWER_PROXIMAL_LENGTH, true);
  // std::cout << "left joint" << std::endl;
  // left_joint.print();
  Point right_joint = intersection_of_two_circles(
      end_effector, upper ? UPPER_BASE : LOWER_BASE,
      upper ? UPPER_DISTAL_LENGTH : LOWER_DISTAL_LENGTH,
      upper ? UPPER_PROXIMAL_LENGTH : LOWER_PROXIMAL_LENGTH, false);
  // std::cout << "right joint" << std::endl;
  // right_joint.print();

  Point base = upper ? UPPER_BASE : LOWER_BASE;
  double midpoint_distance =
      upper ? UPPER_MIDPOINT_DISTANCE : LOWER_MIDPOINT_DISTANCE;
  Point left_midpoint =
      ((left_joint - base).normalize() * midpoint_distance) + base;
  // std::cout << "left midpoint" << std::endl;
  // left_midpoint.print();
  Point right_midpoint =
      ((right_joint - base).normalize() * midpoint_distance) + base;
  if (upper) {
    robot.top_linkage.right_joint = right_joint;
    robot.top_linkage.left_joint = left_joint;
    robot.top_linkage.left_midpoint = left_midpoint;
    robot.top_linkage.right_midpoint = right_midpoint;
  } else {
    robot.bottom_linkage.right_joint = right_joint;
    robot.bottom_linkage.left_joint = left_joint;
    robot.bottom_linkage.left_midpoint = left_midpoint;
    robot.bottom_linkage.right_midpoint = right_midpoint;
  }
  // std::cout << "right midpoint" << std::endl;
  // right_joint.print();
  if (upper) {
    slider_positions_struct.left_slider_y =
        get_y_of_slider_based_on_midpoint_location(left_midpoint, sliderXs[0],
                                                   UPPER_TRANSMISSION_LENGTH);
    slider_positions_struct.right_slider_y =
        get_y_of_slider_based_on_midpoint_location(right_midpoint, sliderXs[3],
                                                   UPPER_TRANSMISSION_LENGTH);
  } else {
    slider_positions_struct.left_middle_slider_y =
        get_y_of_slider_based_on_midpoint_location(left_midpoint, sliderXs[1],
                                                   LOWER_TRANSMISSION_LENGTH);
    slider_positions_struct.right_middle_slider_y =
        get_y_of_slider_based_on_midpoint_location(right_midpoint, sliderXs[2],
                                                   LOWER_TRANSMISSION_LENGTH);
  }
}
