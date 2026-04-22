/**
 * @file Templated_Point.h
 * @author Tyler Lehrfeld
 * @brief Define the point class and its operations
 * @version 0.1
 * @date 2025-10-18
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef TEMPLATE_POINT
#define TEMPLATE_POINT
#include <cmath>
inline bool is_close(double a, double b) {
	return std::abs(a -b) < .0001;

}

#include <cmath>
#include <iostream>
#include <sstream>
template <typename T> struct Point {
  T x;
  T y;
  T z;
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
  Point operator-(const Point &p2) const {
    return {.x = this->x - p2.x, .y = this->y - p2.y, .z = this->z - p2.z};
  }

  /**
   * @brief scale a point
   *
   * @param scalar
   * @param m
   * @return Point
   */
  friend Point operator*(const T scalar, const Point &m) {
    return {.x = m.x * scalar, .y = m.y * scalar, .z = m.z * scalar};
  }

  /**
   * @brief dot product of two points
   *
   * @param p1
   * @param p2
   * @return double
   */
  friend T operator*(const Point p1, const Point &p2) {
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
  }

  /**
   * @brief scale a point
   *
   * @param scalar
   * @return Point
   */
  Point operator*(const T scalar) const {
    return {
        .x = this->x * scalar, .y = this->y * scalar, .z = this->z * scalar};
  }

  bool operator==(const Point<double> p) const {
    return is_close(p.x, x) && is_close(p.y, y) && is_close(p.z, z);
  }

  /**
   * @brief get the magnitude of a vector
   *
   * @return T
   */
  T magnitude() { return sqrt(x * x + y * y + z * z); }

  /**
   * @brief normalize a vector
   *
   * @return Point
   */
  Point normalize() {
    T magnitude = this->magnitude();
    return {.x = this->x / magnitude,
            .y = this->y / magnitude,
            .z = this->z / magnitude};
  }

  /**
   * @brief return the cross product of two vectors
   *
   * @param p1
   * @param p2
   * @return Point
   */
  friend Point cross(const Point p1, const Point p2) {
    return {.x = p1.y * p2.z - p1.z * p2.y,
            .y = p1.z * p2.x - p1.x * p2.z,
            .z = p1.x * p2.y - p1.y * p2.x};
  }
  template <typename U> Point<U> convert() const {
    Point<U> point;
    point.x = U(x);
    point.y = U(y);
    point.z = U(z);
    return point;
  }
};

#endif
