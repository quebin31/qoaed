#ifndef QOAED_POINT_HPP
#define QOAED_POINT_HPP 

#include <ostream>

#include "Traits.hpp"

namespace qoaed {

template <class T>
class Point2D { 
public:
  T x;
  T y;

  Point2D() {
    static_assert(
      tools::is_valid_coord_type<T>::value,
      "You passed an invalid type in Point2D, valids are "
      "{ short, int, long, float, double }"
    );
  } 

  Point2D(const T& x, const T& y) : x(x), y(y) {
    static_assert(
      tools::is_valid_coord_type<T>::value,
      "You passed an invalid type in Point2D, valids are "
      "{ short, int, long, float, double }"
    );
  }

  Point2D(const Point2D& p) : x(p.x), y(p.y) {
    static_assert(
      tools::is_valid_coord_type<T>::value,
      "You passed an invalid type in Point2D, valids are "
      "{ short, int, long, float, double }"
    );
  }

  bool operator==(const Point2D& p) const { return x == p.x && y == p.y;}
  bool operator!=(const Point2D& p) const { return x != p.x || y != p.y; }

  double distance_wo_sqrt(const Point2D& p) const { 
    T delta_x = p.x - x;
    T delta_y = p.y - y;
    return static_cast<double>(delta_x * delta_x + delta_y * delta_y);
  }
};

template <class T>
class Point3D {
public:
  T x;
  T y;
  T z;

  Point3D() { 
    static_assert(
      tools::is_valid_coord_type<T>::value,
      "You passed an invalid type in Point3D, valids are "
      "{ short, int, long, float, double }"
    );
  }

  Point3D(const T& x, const T& y, const T& z) : x(x), y(y), z(z) {
    static_assert(
      tools::is_valid_coord_type<T>::value,
      "You passed an invalid type in Point3D, valids are "
      "{ short, int, long, float, double }"
    );
  }

  Point3D(const Point3D& p) : x(p.x), y(p.y), z(p.z) {
    static_assert(
      tools::is_valid_coord_type<T>::value,
      "You passed an invalid type in Point3D, valids are "
      "{ short, int, long, float, double }"
    );
  }

  bool operator==(const Point3D& p) const { return x == p.x && y == p.y && z == p.z; }
  bool operator!=(const Point3D& p) const { return x != p.x || y != p.y || z != p.z; }

  double distance_wo_sqrt(const Point3D& p) const {
    T delta_x = p.x - x;
    T delta_y = p.y - y;
    T delta_z = p.z - z;
    return static_cast<double>(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
  }
};

}

#endif
