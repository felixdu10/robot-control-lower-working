#include "Point.h"
#include "Robot.h"
#include "NewTransform.h"
#include "kinematic_structs.h"

#ifndef INVERSE_KINEMATICS
#define INVERSE_KINEMATICS

Point intersection_of_two_circles(Point center_a, Point center_b,
                                  double radius_a, double radius_b,
                                  bool left_point);

linkage_end_effectors
get_linkage_end_effector(target_and_injection_point_approach robot_approach,
                         double z_lower, double z_upper,
                         double &needle_extension, Robot& robot);

void get_slider_positions(slider_positions &slider_positions_struct,
                          Point end_effector, bool upper, Robot& robot);

slider_positions inverse_kinematics(
    target_and_injection_point_approach needle_to_patient_approach, Robot& robot);

slider_positions inverse_kinematics(
    target_and_injection_point_approach needle_to_patient_approach,
    NewTransform T_RP, Robot& robot);

slider_positions inverse_kinematics(approach_definition needle_to_patient_approach,
                    NewTransform T_RP, Robot& robot);

#endif
