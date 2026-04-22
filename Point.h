/**
 * @file Point.h
 * @author Tyler Lehrfeld
 * @brief Define the point class and its operations
 * @version 0.1
 * @date 2025-03-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef POINT
#define POINT

#include "Matrix.h"
#include <cmath>
#include <iostream>
#include <sstream>
struct Point {
  double x;
  double y;
  double z;
  /**
   * @brief print out the point
   *
   */
  void print() { std::cout << x << " " << y << " " << z << std::endl; }
  std::string to_string(bool semi) {
    std::stringstream ss;
    if (semi) {
      ss << x << " " << y << " " << z << ";" << std::endl;
    } else {
      ss << x << " " << y << " " << z << std::endl;
    }
    return ss.str();
  }
  void print_desmos() {
    std::cout << "(" << x << ", " << y << ", " << z << ")" << std::endl;
  }

  /**
   * @brief add a point to another point
   *
   * @param p2
   * @return Point
   */
  Point operator+(const Point &p2) const {
    return {.x = this->x + p2.x, .y = this->y + p2.y, .z = this->z + p2.z};
  }

  /**
   * @brief subtract p2 from p1 (this)
   *
   * @param p2
   * @return Point
   */
  Point operator-(Point &p2) {
    return {.x = this->x - p2.x, .y = this->y - p2.y, .z = this->z - p2.z};
  }

  /**
   * @brief scale a point
   *
   * @param scalar
   * @param m
   * @return Point
   */
  friend Point operator*(double scalar, Point &m) {
    return {.x = m.x * scalar, .y = m.y * scalar, .z = m.z * scalar};
  }

  /**
   * @brief dot product of two points
   *
   * @param p1
   * @param p2
   * @return double
   */
  friend double operator*(Point p1, Point &p2) {
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
  }

  /**
   * @brief scale a point
   *
   * @param scalar
   * @return Point
   */
  Point operator*(double scalar) {
    return {
        .x = this->x * scalar, .y = this->y * scalar, .z = this->z * scalar};
  }

  /**
   * @brief get the magnitude of a vector
   *
   * @return double
   */
  double magnitude() {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
  }

  /**
   * @brief normalize a vector
   *
   * @return Point
   */
  Point normalize() {
    double magnitude = this->magnitude();
    return {.x = this->x / magnitude,
            .y = this->y / magnitude,
            .z = this->z / magnitude};
  }

  Matrix to_matrix() { return Matrix(3, 1, {x, y, z}); }
};

/**
 * @brief return the cross product of two vectors
 *
 * @param p1
 * @param p2
 * @return Point
 */
inline Point cross(Point p1, Point p2) {
  return {.x = p1.y * p2.z - p1.z * p2.y,
          .y = p1.z * p2.x - p1.x * p2.z,
          .z = p1.x * p2.y - p1.y * p2.x};
}

#endif
