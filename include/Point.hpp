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
      "{ short, unsigned short, int, unsigned int, long, unsigned long, float, double }"
    );
  } 

  Point2D(const T& x, const T& y) : x(x), y(y) {
    static_assert(
      tools::is_valid_coord_type<T>::value,
      "You passed an invalid type in Point2D, valids are "
      "{ short, unsigned short, int, unsigned int, long, unsigned long, float, double }"
    );
  }

  Point2D(const Point2D& p) : x(p.x), y(p.y) {
    static_assert(
      tools::is_valid_coord_type<T>::value,
      "You passed an invalid type in Point2D, valids are "
      "{ short, unsigned short, int, unsigned int, long, unsigned long, float, double }"
    );
  }

  bool operator==(const Point2D& p) const { return x == p.x && y == p.y;}
  bool operator!=(const Point2D& p) const { return x != p.x || y != p.y; }
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
      "{ short, unsigned short, int, unsigned int, long, unsigned long, float, double }"
    );
  }

  Point3D(const T& x, const T& y, const T& z) : x(x), y(y), z(z) {
    static_assert(
      tools::is_valid_coord_type<T>::value,
      "You passed an invalid type in Point3D, valids are "
      "{ short, unsigned short, int, unsigned int, long, unsigned long, float, double }"
    );
  }

  Point3D(const Point3D& p) : x(p.x), y(p.y), z(p.z) {
    static_assert(
      tools::is_valid_coord_type<T>::value,
      "You passed an invalid type in Point3D, valids are "
      "{ short, unsigned short, int, unsigned int, long, unsigned long, float, double }"
    );
  }


  bool operator==(const Point3D& p) const { return x == p.x && y == p.y && z == p.z; }
  bool operator!=(const Point3D& p) const { return x != p.x || y != p.y || z != p.z; }
};

}

#endif
