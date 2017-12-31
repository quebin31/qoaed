#ifndef QOAED_REGION_QUADTREE_HPP
#define QOAED_REGION_QUADTREE_HPP 

#include "PointQuadtree.hpp"

#include <map>
#include <array>

namespace qoaed {

template <class Value, class CoordType = long>
class RegionQuadtree {
public:
  using value_type = Value;
  using reference = Value&;
  using const_reference = Value const&;
  using Point = Point2D<CoordType>;

private: 

  class Obj {
  public:
    Point         point;
    mutable Value val;

    Obj(const Point& p, const Value& val) :
      point(p), val(val) {}

    Obj(const CoordType& x, const CoordType& y, const Value& val) :
      point(x,y), val(val) {}
  };


  class Node;
  using Childs = typename std::array<Node*, 4>;
  using Box    = typename std::map<CoordType, std::map<CoordType, Obj>>;

  class Node {
  public:
    // min for left bottom point
    // max for right top point
    Point   min;
    Point   max;
    Childs  childs;
    Box     box;

    Node(const Point& min, const Point& max) : 
      min(min), max(max) { childs = {0,0,0,0}; }

    Node(const CoordType& minx, const CoordType& miny, const CoordType& maxx, const CoordType& maxy) :
      min(minx, miny), max(maxx, maxy) { childs = {0,0,0,0}; }

    void insert(const CoordType& x, const CoordType& y, const Value& val) {
    }

    void remove(const CoordType& x, const CoordType& y) {
    }

    auto find(const CoordType& x, const CoordType& y) {
    } 
  };



};

}

#endif
