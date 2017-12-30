#ifndef QOAED_REGION_QUADTREE_HPP
#define QOAED_REGION_QUADTREE_HPP 

#include "PointQuadtree.hpp"

#include <map>
#include <array>

namespace qoaed {

template <class Value, class Key = long>
class RegionQuadtree {
private: 

  class Obj {
  public:
    Key x, y;
    mutable Value val;

    Obj(const Key& x, const Key& y, const Value& val) :
      x(x), y(y), val(val) {}
  };

  using Box = typename std::map<Key, std::map<Key, Obj>>;

  class Node;
  using Childs = typename std::array<Node*, 4>;

  class Node {
  public:
    Key min_x, min_y;
    Key max_x, max_y;
    Childs  childs;
    Box     box;

    Node(const Key& minx, const Key& miny, const Key& maxx, const Key& maxy) :
      min_x(minx), min_y(miny), max_x(maxx), max_y(maxy) 
      { childs = {0, 0, 0, 0 }; }

    void insert(const Key& x, const Key& y, const Value& val) {
    }

    void remove(const Key& x, const Key& y) {
    }

    auto find(const Key& x, const Key& y) {
    } 
  };



};

}

#endif
