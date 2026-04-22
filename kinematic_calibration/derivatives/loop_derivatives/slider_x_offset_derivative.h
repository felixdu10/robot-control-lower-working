#ifndef SLIDERXDER
#define SLIDERXDER
#include "../../jacobian.h"
#include "../../kinematics.h"

inline Point c_i_j_derivative_wrt_x_slider_offset() {
	B_i_derivative_wrt_slider_x_offset();
	
}

inline float error_derivative_wrt_x_slider_offset(Thetas thetas,
                                                  Parameters params, int i,
                                                  Measurement m) {
  // error = |c|-distal_length
  Point c = get_actual_C(params, thetas, i, m);
  return (c_i_j_derivative_wrt_x_slider_offset() * c)/c.magnitude();
}

#endif
