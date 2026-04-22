/**
 * @file NewTransform.h
 * @author Tyler Lehrfeld
 * @brief This file defines a transformation matrix class that can multiply a
 * 4x4 transform matrix by another to get a new transform
 * @version 0.1
 * @date 2025-02-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Matrix.h"
#include "Point.h"
#include "Transform.h"
#include <cassert>
#include <iostream>
#include <math.h>

#ifndef NEW_TRANSFORM
#define NEW_TRANSFORM

struct Quaternion {
  double w, x, y, z;
};

class NewTransform {
public:
  double matrix[4][4];
  NewTransform(){};

  /**
   * @brief Construct a new Transform object by giving an x angle, a y angle,
   * and a z angle. This will rotate the identity matrix by theta_x, theta_y,
   * theta_z and translate by (x, y, z)
   *
   * @param theta_x rotate around the x axis
   * @param theta_y rotate around the y axis
   * @param theta_z rotate around the z axis
   * @param x
   * @param y
   * @param z
   */
  NewTransform(double theta_x, double theta_y, double theta_z, double x,
               double y, double z) {
    base_constructor(theta_x, theta_y, theta_z, x, y, z);
  }

  NewTransform(Point translation, double rotation[3][3]) {
    matrix[0][0] = rotation[0][0];
    matrix[0][1] = rotation[0][1];
    matrix[0][2] = rotation[0][2];
    matrix[0][3] = translation.x;
    matrix[1][0] = rotation[1][0];
    matrix[1][1] = rotation[1][1];
    matrix[1][2] = rotation[1][2];
    matrix[1][3] = translation.x;
    matrix[2][0] = rotation[2][0];
    matrix[2][1] = rotation[2][1];
    matrix[2][2] = rotation[2][2];
    matrix[2][3] = translation.x;
    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
  }

  NewTransform(Transform T) {
    matrix[0][0] = T.R_AB.matrixArray[0];
    matrix[0][1] = T.R_AB.matrixArray[1];
    matrix[0][2] = T.R_AB.matrixArray[2];
    matrix[0][3] = T.p_AB.matrixArray[0];
    matrix[1][0] = T.R_AB.matrixArray[3];
    matrix[1][1] = T.R_AB.matrixArray[4];
    matrix[1][2] = T.R_AB.matrixArray[5];
    matrix[1][3] = T.p_AB.matrixArray[1];
    matrix[2][0] = T.R_AB.matrixArray[6];
    matrix[2][1] = T.R_AB.matrixArray[7];
    matrix[2][2] = T.R_AB.matrixArray[8];
    matrix[2][3] = T.p_AB.matrixArray[2];
    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
  }
  /**
   * @brief Destroy the Transform object
   *
   */
  ~NewTransform() {}

  /**
   * @brief multiply transforms together
   *
   * @param T1
   * @return Transform
   */
  NewTransform operator*(const NewTransform &T1) const {
    NewTransform new_transform;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        new_transform.matrix[i][j] = this->matrix[i][0] * T1.matrix[0][j] +
                                     this->matrix[i][1] * T1.matrix[1][j] +
                                     this->matrix[i][2] * T1.matrix[2][j] +
                                     this->matrix[i][3] * T1.matrix[3][j];
      }
    }
    return new_transform;
  }

  /**
   * @brief We transform a point p1 by rotating and translating it.
   *
   * @param p1
   * @return Point
   */
  Point operator*(Point &p1) const {
    return {.x = this->matrix[0][0] * p1.x + this->matrix[0][1] * p1.y +
                 this->matrix[0][2] * p1.z + this->matrix[0][3],
            .y = this->matrix[1][0] * p1.x + this->matrix[1][1] * p1.y +
                 this->matrix[1][2] * p1.z + this->matrix[1][3],
            .z = this->matrix[2][0] * p1.x + this->matrix[2][1] * p1.y +
                 this->matrix[2][2] * p1.z + this->matrix[2][3]};
  }

  //  friend Point operator*(NewTransform T, Point p1) {
  //    return {.x = T.matrix[0][0] * p1.x + T.matrix[0][1] * p1.y +
  //                 T.matrix[0][2] * p1.z + T.matrix[0][3],
  //            .y = T.matrix[1][0] * p1.x + T.matrix[1][1] * p1.y +
  //                 T.matrix[1][2] * p1.z + T.matrix[1][3],
  //            .z = T.matrix[2][0] * p1.x + T.matrix[2][1] * p1.y +
  //                 T.matrix[2][2] * p1.z + T.matrix[2][3]};
  //  }

  bool operator==(NewTransform &T) {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        if (this->matrix[i][j] != T.matrix[i][j]) {
          return false;
        }
      }
    }
    return true;
  }
  /**
   * @brief return the inverse of the transform matrix
   *
   * @return NewTransform
   */
  NewTransform inverse() {
    NewTransform inv;
    double rot[3][3];
    double trans[3];

    // Extract rotation matrix
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        rot[i][j] = matrix[i][j];
      }
      trans[i] = matrix[i][3]; // Extract translation vector
    }

    // Compute inverse rotation (transpose of rotation matrix)
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        inv.matrix[i][j] = rot[j][i];
      }
    }

    // Compute inverse translation
    for (int i = 0; i < 3; i++) {
      inv.matrix[i][3] =
          -(inv.matrix[i][0] * trans[0] + inv.matrix[i][1] * trans[1] +
            inv.matrix[i][2] * trans[2]);
    }

    // Set last row
    inv.matrix[3][0] = 0;
    inv.matrix[3][1] = 0;
    inv.matrix[3][2] = 0;
    inv.matrix[3][3] = 1;

    return inv;
  }

  Transform to_transform() {
    Transform T;
    T.p_AB = Matrix(3, 1, {matrix[0][3], matrix[1][3], matrix[2][3]});
    T.R_AB = Matrix(3, 3,
                    {matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0],
                     matrix[1][1], matrix[1][2], matrix[2][0], matrix[2][1],
                     matrix[2][2]});
    return T;
  }

  /**
   * @brief The base constructor that creates a new
   *
   */
  void base_constructor(double theta_x, double theta_y, double theta_z,
                        double x, double y, double z) {
    matrix[0][0] = cos(theta_z) * cos(theta_y);
    matrix[0][1] =
        cos(theta_z) * sin(theta_y) * sin(theta_x) - sin(theta_z) * cos(x);
    matrix[0][2] = cos(theta_z) * sin(theta_y) * cos(theta_x) +
                   sin(theta_z) * sin(theta_x);
    matrix[0][3] = x;
    matrix[1][0] = sin(theta_z) * cos(theta_y);
    matrix[1][1] = sin(theta_z) * sin(theta_y) * sin(theta_x) +
                   cos(theta_z) * cos(theta_x);
    matrix[1][2] = sin(theta_z) * sin(theta_y) * cos(theta_x) -
                   cos(theta_z) * sin(theta_x);
    matrix[1][3] = y;
    matrix[2][0] = -sin(theta_y);
    matrix[2][1] = cos(theta_y) * sin(theta_x);
    matrix[2][2] = cos(theta_y) * cos(theta_x);
    matrix[2][3] = z;
    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
  }

  std::string to_string() {
    std::stringstream ss;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        ss << matrix[i][j] << " ";
      }
      ss << std::endl;
    }
    std::string s = ss.str();
    return s;
  }
  /**
   * @brief print a transform so that it is readable
   *
   */
  void print() {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        std::cout << matrix[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }

  Quaternion to_quaternion() {
    Quaternion q;
    double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];

    if (trace > 0.0) {
      double s = 0.5 / sqrt(trace + 1.0);
      q.w = 0.25 / s;
      q.x = (matrix[2][1] - matrix[1][2]) * s;
      q.y = (matrix[0][2] - matrix[2][0]) * s;
      q.z = (matrix[1][0] - matrix[0][1]) * s;
    } else {
      if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2]) {
        double s = 2.0 * sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]);
        q.w = (matrix[2][1] - matrix[1][2]) / s;
        q.x = 0.25 * s;
        q.y = (matrix[0][1] + matrix[1][0]) / s;
        q.z = (matrix[0][2] + matrix[2][0]) / s;
      } else if (matrix[1][1] > matrix[2][2]) {
        double s = 2.0 * sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]);
        q.w = (matrix[0][2] - matrix[2][0]) / s;
        q.x = (matrix[0][1] + matrix[1][0]) / s;
        q.y = 0.25 * s;
        q.z = (matrix[1][2] + matrix[2][1]) / s;
      } else {
        double s = 2.0 * sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]);
        q.w = (matrix[1][0] - matrix[0][1]) / s;
        q.x = (matrix[0][2] + matrix[2][0]) / s;
        q.y = (matrix[1][2] + matrix[2][1]) / s;
        q.z = 0.25 * s;
      }
    }

    return q;
  }

  void from_quaternion(const Quaternion &q) {
    // Normalize the quaternion to ensure a valid rotation matrix
    double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    double w = q.w / norm;
    double x = q.x / norm;
    double y = q.y / norm;
    double z = q.z / norm;

    // Compute the rotation matrix elements
    matrix[0][0] = 1 - 2 * (y * y + z * z);
    matrix[0][1] = 2 * (x * y - z * w);
    matrix[0][2] = 2 * (x * z + y * w);
    matrix[0][3] = 0;

    matrix[1][0] = 2 * (x * y + z * w);
    matrix[1][1] = 1 - 2 * (x * x + z * z);
    matrix[1][2] = 2 * (y * z - x * w);
    matrix[1][3] = 0;

    matrix[2][0] = 2 * (x * z - y * w);
    matrix[2][1] = 2 * (y * z + x * w);
    matrix[2][2] = 1 - 2 * (x * x + y * y);
    matrix[2][3] = 0;

    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
  }
  Point p() { return {matrix[0][3], matrix[1][3], matrix[2][3]}; }
  Matrix R() {
    return Matrix(3, 3,
                  {matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0],
                   matrix[1][1], matrix[1][2], matrix[2][0], matrix[2][1],
                   matrix[2][2]});
  }
};

#endif
