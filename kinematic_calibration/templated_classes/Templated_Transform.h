#ifndef TEMPLATED_TRANSFORM
#define TEMPLATED_TRANSFORM

#include "Templated_Point.h"
#include <cmath>

template <typename T> class Rotation {
public:
  T matrix[3][3] = {};
  Rotation<T>() {
    for (int i = 0; i < 3; ++i) {
      matrix[i][i] = T(1);
    }
  }
  Rotation<T>(T theta_x, T theta_y, T theta_z) {

    Rotation<T> X;
    Rotation<T> Y;
    Rotation<T> Z;

    T cx = cos(theta_x);
    T cy = cos(theta_y);
    T cz = cos(theta_z);
    T sx = sin(theta_x);
    T sy = sin(theta_y);
    T sz = sin(theta_z);
    T X_matrix[3][3] = {{T(1), T(0), T(0)}, {T(0), cx, -sx}, {T(0), sx, cx}};
    T Y_matrix[3][3] = {{cy, T(0), sy}, {T(0), T(1), T(0)}, {-sy, T(0), cy}};
    T Z_matrix[3][3] = {{cz, -sz, T(0)}, {sz, cz, T(0)}, {T(0), T(0), T(1)}};

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        X.matrix[i][j] = X_matrix[i][j];
        Y.matrix[i][j] = Y_matrix[i][j];
        Z.matrix[i][j] = Z_matrix[i][j];
      }
    }
    Rotation<T> R = Z * Y * X;
    *this = R;
  }

  Rotation<T>(const Rotation<T> &R) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        matrix[i][j] = R.matrix[i][j];
      }
    }
  }
  Rotation<T> operator*(const Rotation<T> &R1) const {
    Rotation<T> ret;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        ret.matrix[i][j] = T(0);
        for (int k = 0; k < 3; k++) {
          ret.matrix[i][j] += matrix[i][k] * R1.matrix[k][j];
        }
      }
    }
    return ret;
  }
  Point<T> operator*(const Point<T> &p) const {
    Point<T> ret;
    ret.x = p.x * matrix[0][0] + p.y * matrix[0][1] + p.z * matrix[0][2];
    ret.y = p.x * matrix[1][0] + p.y * matrix[1][1] + p.z * matrix[1][2];
    ret.z = p.x * matrix[2][0] + p.y * matrix[2][1] + p.z * matrix[2][2];
    return ret;
  }
  std::array<T, 4> toQuaternion() const {
    // returns {w, x, y, z}
    T r00 = matrix[0][0], r01 = matrix[0][1], r02 = matrix[0][2];
    T r10 = matrix[1][0], r11 = matrix[1][1], r12 = matrix[1][2];
    T r20 = matrix[2][0], r21 = matrix[2][1], r22 = matrix[2][2];

    T trace = r00 + r11 + r22;
    T qw, qx, qy, qz;
    if (trace > T(0)) {
      T s = T(0.5) / sqrt(trace + T(1.0));
      qw = T(0.25) / s;
      qx = (r21 - r12) * s;
      qy = (r02 - r20) * s;
      qz = (r10 - r01) * s;
    } else if (r00 > r11 && r00 > r22) {
      T s = T(2.0) * sqrt(T(1.0) + r00 - r11 - r22);
      qw = (r21 - r12) / s;
      qx = T(0.25) * s;
      qy = (r01 + r10) / s;
      qz = (r02 + r20) / s;
    } else if (r11 > r22) {
      T s = T(2.0) * sqrt(T(1.0) + r11 - r00 - r22);
      qw = (r02 - r20) / s;
      qx = (r01 + r10) / s;
      qy = T(0.25) * s;
      qz = (r12 + r21) / s;
    } else {
      T s = T(2.0) * sqrt(T(1.0) + r22 - r00 - r11);
      qw = (r10 - r01) / s;
      qx = (r02 + r20) / s;
      qy = (r12 + r21) / s;
      qz = T(0.25) * s;
    }
    return {qw, qx, qy, qz};
  }

  Rotation inverse() {
    Rotation<T> inv;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        inv.matrix[i][j] = matrix[j][i];
      }
    }
    return inv;
  }
  template <typename U> Rotation<U> convert() const {
    Rotation<U> rot;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        rot.matrix[i][j] = U(matrix[i][j]);
      }
    }
    return rot;
  }
};

template <typename T> class Transform {
public:
  Rotation<T> R;
  Point<T> p;
  Transform<T>() {
    R = Rotation<T>(T(0), T(0), T(0));
    p = {T(0), T(0), T(0)};
  }
  Transform<T>(T theta_x, T theta_y, T theta_z, T x, T y, T z) {
    R = Rotation<T>(theta_x, theta_y, theta_z);
    p = {x, y, z};
  }
  Transform<T>(Point<T> x_hat, Point<T> y_hat, Point<T> z_hat,
               Point<T> translation) {
    R.matrix[0][0] = x_hat.x;
    R.matrix[1][0] = x_hat.y;
    R.matrix[2][0] = x_hat.z;
    R.matrix[0][1] = y_hat.x;
    R.matrix[1][1] = y_hat.y;
    R.matrix[2][1] = y_hat.z;
    R.matrix[0][2] = z_hat.x;
    R.matrix[1][2] = z_hat.y;
    R.matrix[2][2] = z_hat.z;
    p = translation;
  }

  void print() const {
    std::cout << "R:" << std::endl;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {

        std::cout << R.matrix[i][j] << " ";
      }
      std::cout << std::endl;
    }

    std::cout << "p:" << std::endl;
    std::cout << p.x << " " << p.y << " " << p.z << std::endl;
  }
  Transform<T> operator*(const Transform<T> &transform) const {
    Transform<T> ret;
    ret.R = R * transform.R;
    ret.p = (R * transform.p) + p;
    return ret;
  };

  Point<T> operator*(const Point<T> &p2) const { return R * p2 + p; }
  Transform<T> inverse() {
    Transform<T> ret;
    ret.R = R.inverse();
    ret.p = ret.R * (T(-1) * p);
    return ret;
  }
  template <typename U> Transform<U> convert() const {
    Transform<U> transform;
    transform.p = p.template convert<U>();
    transform.R = R.template convert<U>();
    return transform;
  }
};

#endif // TEMPLATED_TRANSFORM
