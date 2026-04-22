#ifndef ROBOT
#define ROBOT

#include "kinematic_structs.h"

/**
 * 
 */
class Robot {
public:
  Point origin = {0, 0, 0};
  Point needle_tip = {0, 0, 0};
  Point x_prime = {0, 0, 0};
  Point y_prime = {0, 0, 0};
  Point z_prime = {0, 0, 0};
  linkage_array top_linkage = {
      .extended_end_effector = {0, 0, 0},
      .linkage_end_effector = {0, 0, 0},
      .base = UPPER_BASE,
      .left_joint = {0, 0, 0},
      .right_joint = {0, 0, 0},
      .left_midpoint = {0, 0, 0},
      .right_midpoint = {0, 0, 0},
  };
  linkage_array bottom_linkage = {
      .extended_end_effector = {0, 0, 0},
      .linkage_end_effector = {0, 0, 0},
      .base = LOWER_BASE,
      .left_joint = {0, 0, 0},
      .right_joint = {0, 0, 0},
      .left_midpoint = {0, 0, 0},
      .right_midpoint = {0, 0, 0},
  };
  slider_positions sliders = {
      .left_slider_y = 0,
      .left_middle_slider_y = 0,
      .right_middle_slider_y = 0,
      .right_slider_y = 0,
      .needle_extension = 0,
  };
  bool is_valid(std::string& error_string) {
    bool valid = true;
    error_string = "";
    if(sliders.left_slider_y - HALF_SLIDER_WIDTH < BASE_TO_SLIDER_MIN || sliders.left_slider_y + HALF_SLIDER_WIDTH > BASE_TO_SLIDER_MAX) {
        error_string += "left slider y invalid";
        valid = false;
    }
    if(sliders.left_middle_slider_y - HALF_SLIDER_WIDTH < BASE_TO_SLIDER_MIN || sliders.left_middle_slider_y + HALF_SLIDER_WIDTH > BASE_TO_SLIDER_MAX) {
        error_string += "left middle slider y invalid\n";
        valid = false;
    }
    if(sliders.right_middle_slider_y - HALF_SLIDER_WIDTH < BASE_TO_SLIDER_MIN || sliders.right_middle_slider_y + HALF_SLIDER_WIDTH > BASE_TO_SLIDER_MAX) {
        error_string += "right middle slider y invalid\n";
        valid = false;
    }
    if(sliders.right_slider_y - HALF_SLIDER_WIDTH < BASE_TO_SLIDER_MIN || sliders.right_slider_y + HALF_SLIDER_WIDTH > BASE_TO_SLIDER_MAX) {
        error_string += "right slider y invalid\n";
        valid = false;
    }
    if(sliders.needle_extension < MIN_NEEDLE_EXTENSION || sliders.needle_extension > MAX_NEEDLE_EXTENSION) {
        error_string += "left slider y invalid\n";
        valid = false;
    }
    if((top_linkage.extended_end_effector - bottom_linkage.extended_end_effector).magnitude() > MAX_LINKAGE_END_EFFECTOR_DISTANCE) {
        error_string += "linkages too far apart\n";
        valid = false;
    }
    double right = BASE_TO_SLIDER_MAX -sliders.right_slider_y - HALF_SLIDER_WIDTH;
    double left = BASE_TO_SLIDER_MAX - sliders.left_slider_y - HALF_SLIDER_WIDTH;
    double right_middle = BASE_TO_SLIDER_MAX -sliders.right_middle_slider_y - HALF_SLIDER_WIDTH;
    double left_middle = BASE_TO_SLIDER_MAX - sliders.left_middle_slider_y - HALF_SLIDER_WIDTH;
    if(right < 40 || right > 75 || left < 40 || left > 75 || right_middle > 45 || left_middle > 45 || right_middle < 7 || left_middle < 7) {
        error_string += "sliders not constrained enough\n";
        valid = false;
    }
    
    return valid;
  }
  
};

#endif