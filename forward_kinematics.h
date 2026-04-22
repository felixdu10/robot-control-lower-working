
#ifndef KINEMATICS
#define KINEMATICS

#include "Point.h"
#include "Robot.h"
#include "NewTransform.h"
#include "kinematic_structs.h"
/**
 * @brief Get a left or right joint. The joint is where the proximal links meet
 * the distal links
 *
 * @param is_left_joint
 * @param midpoint_distance
 * @param transmission_length
 * @param proximal_length
 * @param base
 * @param slider
 * @return Point
 */
Point get_joint(bool is_left_joint, double midpoint_distance,
                double transmission_length, double proximal_length, Point base,
                Point slider, linkage_array& robot_array);

/**
 * @brief Get the end effector of each linkage.
 * The top and bottom have slightly different extentions, but the main math is
 * just getting the tip of an isosceles triangle.
 *
 * @param is_upper
 * @param left_slider
 * @param right_slider
 * @param base
 * @param transmission_length
 * @param proximal_length
 * @param distal_length
 * @param midpoint_distance
 * @param end_effector_vect
 * @param z
 * @return Point
 */
Point get_linkage_end_effector(bool is_upper, Point left_slider,
                               Point right_slider, Point base,
                               double transmission_length,
                               double proximal_length, double distal_length,
                               double midpoint_distance,
                               Point end_effector_vect, double z, Robot& robot);

/**
 * @brief Get the end effector (needle tip) based on the top and bottom linkage
 * positions.
 *
 * @param left
 * @param left_middle
 * @param right_middle
 * @param right
 * @param top_base
 * @param bottom_base
 * @param needle_extension
 * @return Point
 */
Point get_end_effector(Point left, Point left_middle, Point right_middle,
                       Point right, Point top_base, Point bottom_base,
                       double needle_extension, Robot& robot);

Point get_needle_point_based_on_end_effector_positions(
    Point upper_linkage_end_effector, Point lower_linkage_end_effector,
    double needle_extension, Robot& robot);

Point get_end_effector(slider_positions sliders, Robot& robot);
#endif