#ifndef CAL_KINEMATICS
#define CAL_KINEMATICS

#include "./templated_classes/Templated_Point.h"
#include "./templated_classes/Templated_Transform.h"
#include <chrono>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

#define num_loops 4

enum linkage_values {
  TOP_LEFT = 0,
  TOP_RIGHT = 1,
  BOTTOM_LEFT = 2,
  BOTTOM_RIGHT = 3,
};

template <typename T> struct Measurement {
  Transform<T> F_OM1;
  Transform<T> F_OM2;
  template <typename U> Measurement<U> convert() const {
    Measurement<U> M;
    M.F_OM1 = F_OM1.template convert<U>();
    M.F_OM2 = F_OM2.template convert<U>();
    return M;
  }
};

template <typename T> struct Thetas {
  T theta_1;
  T theta_2;
  T theta_3;
  T theta_4;
  T theta_5;
  template <typename U> Thetas<U> convert() const {
    Thetas<U> thetas;
    thetas.theta_1 = U(theta_1);
    thetas.theta_2 = U(theta_2);
    thetas.theta_3 = U(theta_3);
    thetas.theta_4 = U(theta_4);
    thetas.theta_5 = U(theta_5);
    return thetas;
  }
  void print() {
    std::cout << "left slider y: " << theta_1 << " right slider y: " << theta_2
              << " left middle slider y:" << theta_3
              << " right middle slider y: " << theta_4 << std::endl;
  }
};

template <typename T> struct Param {
  T value;
  bool tunable;
  constexpr Param<T>() : value(T(0)), tunable(false) {}
  constexpr Param<T>(T val, bool _tunable) : value(val), tunable(_tunable) {}
};

template <typename T> struct Loop_parameters {
  Param<T> x_slider_offset;
  Param<T> y_slider_offset;
  Param<T> transmission_link_length;
  Param<T> proximal_link_length;
  Param<T> proximal_link_midpoint;
  Param<T> distal_link_length;
};

template <typename T> struct Tunable_parameters {

  /**
   * 0: top left
   * 1: top right
   * 2: bottom left
   * 3: bottom right
   **/
  Loop_parameters<T> loop_parameters[num_loops];
  Param<T> top_needle_to_holder_distance;
  Param<T> bottom_needle_to_holder_distance;
  Param<T> top_holder_to_linkage_distance;
  Param<T> bottom_holder_to_linkage_distance;
  Param<T> needle_offset;
  Param<T> lower_base_z_offset;
  Param<T> upper_base_z_offset;
  Param<T> D1;
  Param<T> D2;
  Param<T> D3;
  Param<T> D4;
  Param<T> lower_slope;
  Param<T> upper_slope;
  Param<T> pivot_x;
  Param<T> pivot_y;
  Param<T> pivot_z;
};

template <typename T> struct Parameters {
  Tunable_parameters<T> tunable_params;
  Point<Param<T>> upper_base;
  Point<Param<T>> lower_base;
};

//-3.96485
// 6
// 1.28137
// 0.549444
template <typename T> constexpr Parameters<T> get_default_parameters() {
  Parameters<T> params;
  Param<T> val;
  T sliderXs[num_loops] = {T(-63), T(63), T(-21), T(21)};

  T sliderYs_offset[num_loops] = {T(0), T(0), T(0), T(0)};
  for (int i = 0; i < num_loops; i++) {
    params.tunable_params.loop_parameters[i].x_slider_offset = {sliderXs[i],
                                                                false};
    params.tunable_params.loop_parameters[i].y_slider_offset = {
        sliderYs_offset[i] /*T(0)*/, true};
    params.tunable_params.loop_parameters[i].transmission_link_length =
        i < 2 ? Param<T>(T(115), true) : Param<T>(T(110), true);
    params.tunable_params.loop_parameters[i].proximal_link_midpoint =
        i < 2 ? Param<T>(T(58), true) : Param<T>(T(46), true);
    params.tunable_params.loop_parameters[i].proximal_link_length =
        i < 2 ? Param<T>(T(116), true) : Param<T>(T(92), true);
    params.tunable_params.loop_parameters[i].distal_link_length =
        i < 2 ? Param<T>(T(125), true) : Param<T>(T(99), true);
  }
  params.tunable_params.top_needle_to_holder_distance = Param<T>(T(47.7), true);
  params.tunable_params.bottom_needle_to_holder_distance =
      Param<T>(T(47.7), true);
  params.tunable_params.top_holder_to_linkage_distance =
      Param<T>(T(19.25), true);
  params.tunable_params.bottom_holder_to_linkage_distance =
      Param<T>(T(14), true);
  params.tunable_params.needle_offset = Param<T>(T(64.9), true);
  params.tunable_params.lower_base_z_offset = Param<T>(T(0), true);
  // check what true value is:
  params.tunable_params.upper_base_z_offset = Param<T>(T(40), true);
  params.tunable_params.D1 = Param<T>(T(62.5), true);
  params.tunable_params.D2 = Param<T>(T(62.5), true);
  params.tunable_params.D3 = Param<T>(T(62.5), true);
  params.tunable_params.D4 = Param<T>(T(62.5), true);
  params.tunable_params.upper_slope = Param<T>(T(0), false);
  params.tunable_params.lower_slope = Param<T>(T(0), false);
  params.lower_base.x = Param<T>(T(0), true);
  params.lower_base.y = Param<T>(T(224), true);
  params.lower_base.z = Param<T>(T(0), true);
  params.upper_base.x = Param<T>(T(0), true);
  params.upper_base.y = Param<T>(T(186), true);
  params.upper_base.z = Param<T>(T(0), true);
  params.tunable_params.pivot_x = Param<T>(T(-0.237684), true);
  params.tunable_params.pivot_y = Param<T>(T(29.5324), true);
  params.tunable_params.pivot_z = Param<T>(T(-0.858548), true);

  return params;
}

template <typename T>
constexpr int get_num_tunable_params(const Parameters<T> parameters) {
  int num_tunables = 0;
  for (int i = 0; i < num_loops; ++i) {
    num_tunables +=
        parameters.tunable_params.loop_parameters[i].x_slider_offset.tunable;
    num_tunables +=
        parameters.tunable_params.loop_parameters[i].y_slider_offset.tunable;
    num_tunables += parameters.tunable_params.loop_parameters[i]
                        .transmission_link_length.tunable;
    num_tunables += parameters.tunable_params.loop_parameters[i]
                        .proximal_link_length.tunable;
    num_tunables += parameters.tunable_params.loop_parameters[i]
                        .proximal_link_midpoint.tunable;
    num_tunables +=
        parameters.tunable_params.loop_parameters[i].distal_link_length.tunable;
  }
  num_tunables +=
      parameters.tunable_params.top_holder_to_linkage_distance.tunable;
  num_tunables +=
      parameters.tunable_params.bottom_holder_to_linkage_distance.tunable;
  num_tunables += parameters.tunable_params.needle_offset.tunable;
  num_tunables += parameters.tunable_params.lower_base_z_offset.tunable;
  num_tunables += parameters.tunable_params.upper_base_z_offset.tunable;
  num_tunables +=
      parameters.tunable_params.top_needle_to_holder_distance.tunable;
  num_tunables +=
      parameters.tunable_params.bottom_needle_to_holder_distance.tunable;
  num_tunables += parameters.tunable_params.D1.tunable;
  num_tunables += parameters.tunable_params.D2.tunable;
  num_tunables += parameters.tunable_params.D3.tunable;
  num_tunables += parameters.tunable_params.D4.tunable;
  num_tunables += parameters.upper_base.x.tunable;
  num_tunables += parameters.upper_base.y.tunable;
  num_tunables += parameters.upper_base.z.tunable;
  num_tunables += parameters.lower_base.x.tunable;
  num_tunables += parameters.lower_base.y.tunable;
  num_tunables += parameters.lower_base.z.tunable;
  num_tunables += parameters.tunable_params.lower_slope.tunable;
  num_tunables += parameters.tunable_params.upper_slope.tunable;
  num_tunables += parameters.tunable_params.pivot_x.tunable;
  num_tunables += parameters.tunable_params.pivot_y.tunable;
  num_tunables += parameters.tunable_params.pivot_z.tunable;

  return num_tunables;
}

template <typename T> void set_val(int *count, Param<T> *p, T *arr) {
  if (p->tunable) {
    arr[*count] = p->value;
    *count = *count + 1;
  }
};

template <typename T>
constexpr inline int Parameters_to_array(Parameters<T> parameters, T *array) {
  int num_tunables = get_num_tunable_params(parameters);
  Tunable_parameters<T> tunables = parameters.tunable_params;
  int count = 0;
  for (int i = 0; i < num_loops; ++i) {
    set_val(&count, &tunables.loop_parameters[i].x_slider_offset, array);
    set_val(&count, &tunables.loop_parameters[i].y_slider_offset, array);
    set_val(&count, &tunables.loop_parameters[i].transmission_link_length,
            array);
    set_val(&count, &tunables.loop_parameters[i].proximal_link_length, array);
    set_val(&count, &tunables.loop_parameters[i].proximal_link_midpoint, array);
    set_val(&count, &tunables.loop_parameters[i].distal_link_length, array);
  }
  set_val(&count, &tunables.top_holder_to_linkage_distance, array);
  set_val(&count, &tunables.bottom_holder_to_linkage_distance, array);
  set_val(&count, &tunables.needle_offset, array);
  set_val(&count, &tunables.lower_base_z_offset, array);
  set_val(&count, &tunables.upper_base_z_offset, array);
  set_val(&count, &tunables.top_needle_to_holder_distance, array);
  set_val(&count, &tunables.bottom_needle_to_holder_distance, array);
  set_val(&count, &tunables.D1, array);
  set_val(&count, &tunables.D2, array);
  set_val(&count, &tunables.D3, array);
  set_val(&count, &tunables.D4, array);
  set_val(&count, &tunables.upper_slope, array);
  set_val(&count, &tunables.lower_slope, array);
  set_val(&count, &parameters.upper_base.x, array);
  set_val(&count, &parameters.upper_base.y, array);
  set_val(&count, &parameters.upper_base.z, array);
  set_val(&count, &parameters.lower_base.x, array);
  set_val(&count, &parameters.lower_base.y, array);
  set_val(&count, &parameters.lower_base.z, array);
  set_val(&count, &tunables.pivot_x, array);
  set_val(&count, &tunables.pivot_y, array);
  set_val(&count, &tunables.pivot_z, array);
  return count;
}

template <typename T> void set_param(Param<T> *par, int *count, const T *arr) {
  if (par->tunable) {
    par->value = arr[*count];
    *count = *count + 1;
  }
}

template <typename T>
inline Parameters<T> array_to_Parameters(T const *tunable_params) {
  Parameters<T> p = get_default_parameters<T>();
  int num_tunable_params = get_num_tunable_params(p);
  int count = 0;
  for (int i = 0; i < num_loops; i++) {
    set_param<T>(&p.tunable_params.loop_parameters[i].x_slider_offset, &count,
                 tunable_params);
    set_param<T>(&p.tunable_params.loop_parameters[i].y_slider_offset, &count,
                 tunable_params);
    set_param<T>(&p.tunable_params.loop_parameters[i].transmission_link_length,
                 &count, tunable_params);
    set_param<T>(&p.tunable_params.loop_parameters[i].proximal_link_length,
                 &count, tunable_params);
    set_param<T>(&p.tunable_params.loop_parameters[i].proximal_link_midpoint,
                 &count, tunable_params);
    set_param<T>(&p.tunable_params.loop_parameters[i].distal_link_length,
                 &count, tunable_params);
  }
  set_param<T>(&p.tunable_params.top_holder_to_linkage_distance, &count,
               tunable_params);
  set_param<T>(&p.tunable_params.bottom_holder_to_linkage_distance, &count,
               tunable_params);
  set_param<T>(&p.tunable_params.needle_offset, &count, tunable_params);
  set_param<T>(&p.tunable_params.lower_base_z_offset, &count, tunable_params);
  set_param<T>(&p.tunable_params.upper_base_z_offset, &count, tunable_params);
  set_param<T>(&p.tunable_params.top_needle_to_holder_distance, &count,
               tunable_params);
  set_param<T>(&p.tunable_params.bottom_needle_to_holder_distance, &count,
               tunable_params);
  set_param<T>(&p.tunable_params.D1, &count, tunable_params);
  set_param<T>(&p.tunable_params.D2, &count, tunable_params);
  set_param<T>(&p.tunable_params.D3, &count, tunable_params);
  set_param<T>(&p.tunable_params.D4, &count, tunable_params);
  set_param<T>(&p.tunable_params.lower_slope, &count, tunable_params);
  set_param<T>(&p.tunable_params.upper_slope, &count, tunable_params);
  set_param<T>(&p.upper_base.x, &count, tunable_params);
  set_param<T>(&p.upper_base.y, &count, tunable_params);
  set_param<T>(&p.upper_base.z, &count, tunable_params);
  set_param<T>(&p.lower_base.x, &count, tunable_params);
  set_param<T>(&p.lower_base.y, &count, tunable_params);
  set_param<T>(&p.lower_base.z, &count, tunable_params);
  set_param<T>(&p.tunable_params.pivot_x, &count, tunable_params);
  set_param<T>(&p.tunable_params.pivot_y, &count, tunable_params);
  set_param<T>(&p.tunable_params.pivot_z, &count, tunable_params);

  return p;
}

inline Parameters<double> adjust_params(Parameters<double> params) {
  std::mt19937_64 rng(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  int num_params = get_num_tunable_params(params);
  double *arr = new double[num_params];
  Parameters_to_array(params, arr);
  for (int i = 0; i < num_params; ++i) {
    arr[i] += dist(rng);
  }
  params = array_to_Parameters(arr);
  delete[] arr;
  // for (int i = 0; i < num_loops; i++) {
  //   params.tunable_params.loop_parameters[i].x_slider_offset.value +=
  //   dist(rng); params.tunable_params.loop_parameters[i].y_slider_offset.value
  //   += dist(rng);
  //   params.tunable_params.loop_parameters[i].transmission_link_length.value
  //   +=
  //       dist(rng);
  //   params.tunable_params.loop_parameters[i].proximal_link_midpoint.value +=
  //       dist(rng);
  //   params.tunable_params.loop_parameters[i].proximal_link_length.value +=
  //       dist(rng);
  //   params.tunable_params.loop_parameters[i].distal_link_length.value +=
  //       dist(rng);
  // }
  // params.tunable_params.top_needle_to_holder_distance.value += dist(rng);
  // params.tunable_params.bottom_needle_to_holder_distance.value += dist(rng);
  // params.tunable_params.top_holder_to_linkage_distance.value += dist(rng);
  // params.tunable_params.bottom_holder_to_linkage_distance.value += dist(rng);
  // params.tunable_params.needle_offset.value += dist(rng);
  // params.tunable_params.lower_base_z_offset.value += dist(rng);
  // params.tunable_params.upper_base_z_offset.value += dist(rng);
  // params.tunable_params.D1.value += dist(rng);
  // params.tunable_params.D2.value += dist(rng);
  // params.tunable_params.D3.value += dist(rng);
  // params.tunable_params.D4.value += dist(rng);
  // params.upper_base.x.value += dist(rng);
  // params.upper_base.y.value += dist(rng);
  // params.lower_base.x.value += dist(rng);
  // params.lower_base.y.value += dist(rng);
  return params;
}

template <typename T>
Point<T> get_upper_linkage_n_vec(const Thetas<T> &thetas,
                                 const Parameters<T> &parameters) {
  Point<T> upper_C = get_linkage_C(thetas, parameters, TOP_LEFT);
  Point<T> D = get_D(thetas, parameters);
  return (upper_C - D).normalize();
}

template <typename T>
Point<T> get_upper_linkage_E(const Thetas<T> &thetas,
                             const Parameters<T> &parameters) {
  Point<T> P1 = get_upper_linkage_P(thetas, parameters);
  Point<T> P2 = get_lower_linkage_P(thetas, parameters);
  Point<T> z = (P1 - P2).normalize();
  Point<T> n_vec = get_upper_linkage_n_vec<T>(thetas, parameters);
  Point<T> x = cross(n_vec, z).normalize();
  Point<T> y = cross(z, x);
  T e1 = parameters.tunable_params.top_needle_to_holder_distance.value;
  return P1 + e1 * y;
}

template <typename T>
Point<T> get_lower_linkage_E(const Thetas<T> &thetas,
                             const Parameters<T> &parameters) {
  Point<T> P1 = get_upper_linkage_P(thetas, parameters);
  Point<T> P2 = get_lower_linkage_P(thetas, parameters);
  Point<T> z = (P1 - P2).normalize();
  Point<T> n_vec = get_upper_linkage_n_vec<T>(thetas, parameters);
  Point<T> x = cross(n_vec, z).normalize();
  Point<T> y = cross(z, x);
  T e2 = parameters.tunable_params.bottom_needle_to_holder_distance.value;
  return P2 + e2 * y;
}

template <typename T>
Transform<T> get_end_effector(const Thetas<T> &thetas,
                              const Parameters<T> &parameters) {
  Point<T> E1 = get_upper_linkage_E<T>(thetas, parameters);
  Point<T> E2 = get_lower_linkage_E<T>(thetas, parameters);
  Point<T> diff = (E2 - E1).normalize();
  Point<T> N =
      E2 +
      (parameters.tunable_params.needle_offset.value + thetas.theta_5) * (diff);
  Point<T> n_vec = get_upper_linkage_n_vec<T>(thetas, parameters);
  Point<T> z = T(-1) * diff;
  Point<T> x = cross(n_vec, z).normalize();
  Point<T> y = cross(z, x);
  Transform<T> EE_frame(x, y, z, N);
  return EE_frame;
}

template <typename T> Transform<T> get_end_effector(const Thetas<T> &thetas) {
  return get_end_effector(thetas, get_default_parameters<T>());
}
template <typename T>
Point<T> get_D(const Thetas<T> &thetas, const Parameters<T> &parameters) {
  Point upper_C = get_linkage_C(thetas, parameters, TOP_LEFT);
  T ratio1 = parameters.tunable_params.D1.value /
             parameters.tunable_params.loop_parameters[TOP_LEFT]
                 .distal_link_length.value;
  T ratio2 = parameters.tunable_params.D2.value /
             parameters.tunable_params.loop_parameters[TOP_RIGHT]
                 .distal_link_length.value;
  Point<T> B_1 = get_linkage_B(thetas, parameters, TOP_LEFT);
  Point<T> B_2 = get_linkage_B(thetas, parameters, TOP_RIGHT);
  Point<T> Q1 = (T(1) - ratio1) * upper_C + ratio1 * B_1;
  Point<T> Q2 = (T(1) - ratio2) * upper_C + ratio2 * B_2;
  return third_point_in_triangle(Q2, Q1, parameters.tunable_params.D3.value,
                                 parameters.tunable_params.D4.value);
}

template <typename T>
Point<T> get_upper_linkage_P(const Thetas<T> &thetas,
                             const Parameters<T> &parameters) {
  Point<T> upper_C = get_linkage_C(thetas, parameters, TOP_RIGHT);
  Point<T> D = get_D(thetas, parameters);
  T n1 = parameters.tunable_params.top_holder_to_linkage_distance.value;
  Point<T> P = upper_C + (upper_C - D) * (n1 / (upper_C - D).magnitude());
  Point<T> base = {parameters.upper_base.x.value, parameters.upper_base.y.value,
                   parameters.upper_base.z.value};
  P.z -= parameters.tunable_params.upper_slope.value * (P - base).magnitude();
  return P;
}

template <typename T>
Point<T> get_linkage_C(const Thetas<T> &thetas, const Parameters<T> &parameters,
                       linkage_values val) {
  Point<T> B_j;
  Point<T> B_k;
  T d_j;
  T d_k;
  if (val == TOP_RIGHT || val == TOP_LEFT) {
    B_j = get_linkage_B(thetas, parameters, TOP_LEFT);
    B_k = get_linkage_B(thetas, parameters, TOP_RIGHT);
    d_j = parameters.tunable_params.loop_parameters[TOP_LEFT]
              .distal_link_length.value;
    d_k = parameters.tunable_params.loop_parameters[TOP_RIGHT]
              .distal_link_length.value;
  } else if (val == BOTTOM_LEFT || val == BOTTOM_RIGHT) {
    B_j = get_linkage_B(thetas, parameters, BOTTOM_LEFT);
    B_k = get_linkage_B(thetas, parameters, BOTTOM_RIGHT);
    d_j = parameters.tunable_params.loop_parameters[BOTTOM_LEFT]
              .distal_link_length.value;
    d_k = parameters.tunable_params.loop_parameters[BOTTOM_RIGHT]
              .distal_link_length.value;
  }
  return third_point_in_triangle(B_j, B_k, d_k, d_j);
}

template <typename T>
Point<T> get_lower_linkage_P(const Thetas<T> &thetas,
                             const Parameters<T> &parameters) {
  Point<T> lower_C = get_linkage_C(thetas, parameters, BOTTOM_LEFT);
  Point<T> lower_left_B = get_linkage_B(thetas, parameters, BOTTOM_LEFT);
  T dll = parameters.tunable_params.loop_parameters[BOTTOM_LEFT]
              .distal_link_length.value;
  T extension =
      parameters.tunable_params.bottom_holder_to_linkage_distance.value;
  T ratio = (dll + extension) / dll;
  Point<T> P = lower_left_B + (lower_C - lower_left_B) * ratio;
  Point<T> base = {parameters.lower_base.x.value, parameters.lower_base.y.value,
                   parameters.lower_base.z.value};
  P.z -= parameters.tunable_params.lower_slope.value * (P - base).magnitude();
  return P;
}

template <typename T>
Point<T> get_linkage_B(const Thetas<T> &thetas, const Parameters<T> &parameters,
                       linkage_values val) {
  Point<T> base;
  Point<T> S_i;
  if (val == TOP_RIGHT || val == TOP_LEFT) {
    base = {parameters.upper_base.x.value, parameters.upper_base.y.value,
            parameters.upper_base.z.value};
    base.z += parameters.tunable_params.upper_base_z_offset.value;
    if (val == TOP_RIGHT) {
      S_i = {parameters.tunable_params.loop_parameters[TOP_RIGHT]
                 .x_slider_offset.value,
             thetas.theta_2 +
                 parameters.tunable_params.loop_parameters[TOP_RIGHT]
                     .y_slider_offset.value,
             base.z};
    } else if (val == TOP_LEFT) {
      S_i = {parameters.tunable_params.loop_parameters[TOP_LEFT]
                 .x_slider_offset.value,
             thetas.theta_1 +
                 parameters.tunable_params.loop_parameters[TOP_LEFT]
                     .y_slider_offset.value,
             base.z};
    }
  } else if (val == BOTTOM_LEFT || val == BOTTOM_RIGHT) {
    base = {parameters.lower_base.x.value, parameters.lower_base.y.value,
            parameters.lower_base.z.value};
    base.z += parameters.tunable_params.lower_base_z_offset.value;
    if (val == BOTTOM_LEFT) {
      S_i = {parameters.tunable_params.loop_parameters[BOTTOM_LEFT]
                 .x_slider_offset.value,
             thetas.theta_3 +
                 parameters.tunable_params.loop_parameters[BOTTOM_LEFT]
                     .y_slider_offset.value,
             base.z};
    } else if (val == BOTTOM_RIGHT) {
      S_i = {parameters.tunable_params.loop_parameters[BOTTOM_RIGHT]
                 .x_slider_offset.value,
             thetas.theta_4 +
                 parameters.tunable_params.loop_parameters[BOTTOM_RIGHT]
                     .y_slider_offset.value,
             base.z};
    }
  }
  T proximal_link_length =
      parameters.tunable_params.loop_parameters[val].proximal_link_length.value;
  T proximal_link_midpoint = parameters.tunable_params.loop_parameters[val]
                                 .proximal_link_midpoint.value;
  T transmission_link_length = parameters.tunable_params.loop_parameters[val]
                                   .transmission_link_length.value;
  Point<T> M;
  try {
    if (val == BOTTOM_LEFT || val == TOP_LEFT) {
      M = third_point_in_triangle(S_i, base, proximal_link_midpoint,
                                  transmission_link_length);
    } else if (val == BOTTOM_RIGHT || val == TOP_RIGHT) {
      M = third_point_in_triangle(base, S_i, transmission_link_length,
                                  proximal_link_midpoint);
    }
  } catch (std::runtime_error &e) {
    if constexpr (std::is_same_v<T, double>) {
      std::cout << thetas.theta_1 << " " << thetas.theta_2 << " "
                << thetas.theta_3 << " " << thetas.theta_4 << std::endl;
    }
    throw std::runtime_error(e.what());
  }
  T ratio = proximal_link_length / proximal_link_midpoint;
  Point<T> B_i = ratio * (M - base) + base;
  return B_i;
}
// template <typename T>
// inline Thetas<T> inverse_kinematics(Parameters<T> parameters, Transform<T>
// transform);

//     l1
//  P3____
// l2\    P2
//    \ t/ l3
//     P1
//
template <typename T>
inline Point<T> third_point_in_triangle(Point<T> P1, Point<T> P2, T l1, T l2) {
  T z = P1.z;
  P1.z = T(0);
  P2.z = T(0);
  Point<T> P3;
  Point<T> diff = P2 - P1;
  if (diff.magnitude() > l1 + l2 || l1 + diff.magnitude() < l2 ||
      l2 + diff.magnitude() < l1) {
    throw std::runtime_error("Points are too far apart to form triangle");
  }
  Point<T> norm = diff.normalize();

  // we will always rotate clockwize from diff.
  Point<T> rotated_norm = {-norm.y, norm.x, T(0)};
  T l3 = (P1 - P2).magnitude();

  T l2cos_t = (l2 * l2 + l3 * l3 - l1 * l1) / (T(2) * l3);
  T h = sqrt(l2 * l2 - l2cos_t * l2cos_t);
  P3 = P1 + (l2cos_t * norm) + (h * rotated_norm);
  P3.z = z;
  return P3;
};

int get_positions(std::vector<Thetas<double>> &positions);

template <typename T>
Transform<T> F_NEE(const Thetas<T> &thetas, const Parameters<T> &params) {
  Point P1 = get_upper_linkage_P(thetas, params);
  Point P2 = get_lower_linkage_P(thetas, params);
  Point<T> z = (P1 - P2).normalize();
  Point<T> x = cross(get_upper_linkage_n_vec(thetas, params), z).normalize();
  Point<T> y = cross(z, x);
  Transform<T> N_Frame(x, y, z, P2);
  Transform<T> EE_frame = get_end_effector(thetas, params);
  return N_Frame.inverse() * EE_frame;
}

template <typename T> Transform<T> F_NEE(const Thetas<T> &thetas) {
  return F_NEE(thetas, get_default_parameters<T>());
};

#endif // CAL_KINEMATICS
