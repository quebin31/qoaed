#ifndef QOAED_POINT_OCTREE_HPP
#define QOAED_POINT_OCTREE_HPP 

#include <set>
#include <list>
#include <array>
#include <stack>
#include <queue>
#include <functional>

#include <iostream>

#include "Point.hpp"

// Octants are identified in this way
// When z is greater_eq than o.z
//
//      y
//
//      |
//   1  |  0
//      |
//------o------ x
//      |
//   2  |  3
//      |
//
// When z is less than o.z
//
//      y
//
//      |
//   5  |  4
//      |
//------o------ x
//      |
//   6  |  7
//      |
//
// See what_octant for more information


namespace qoaed {

template <class Value, class CoordType = long>
class PointOctree {
public:
  using value_type = Value;
  using reference = Value&;
  using const_reference = Value const&;
  using Point = Point3D<CoordType>;

private: 

  class Node;

  class Node {
  public:
    Point   point;
    Node*   childs[8];
    mutable Value val;

    Node(const Point& p, const Value& val) : 
      point(p), val(val) { 
        for (int i = 0; i < 8; ++i)
          childs[i] = 0;
      }

    Node(const CoordType& x, const CoordType& y, const CoordType& z, const Value& val):
      point(x,y,z), val(val) { 
        childs = new Node*[8];
        for (int i = 0; i < 8; ++i)
          childs[i] = 0;
      }
  };

public:
  
  class NodeVisitor {
  friend class PointOctree;

  private:
    Node* n;

  public:
    NodeVisitor(Node* n): n(n) {}
    
    const CoordType& get_x() const { return n->point.x; }
    const CoordType& get_y() const { return n->point.y; }
    const CoordType& get_z() const { return n->point.z; }
    const Point& get_point() const { return n->point; }
    Value& operator*() const { return n->val; }
    operator    bool() const { return (bool) n; }
  };

  class Cube {
  friend class PointOctree;

  private:
    // min is the left bottom front point
    // max is the rigth top back point
    Point min;
    Point max;

  public:
    Cube(const Point& min, const Point& max) : min(min), max(max) {}

    Cube(const CoordType& min_x, const CoordType& min_y, const CoordType& min_z,
         const CoordType& max_x, const CoordType& max_y, const CoordType& max_z) :
      min(min_x, min_y, min_z), max(max_x, max_y, max_z) {}

    bool contains(const Point& p) const {
      bool cx, cy, cz;
      cx = (p.x <= max.x && p.x >= min.x);
      cy = (p.y <= max.y && p.y >= min.y);
      cz = (p.z <= max.z && p.z >= min.z);
      return cx && cy && cz;
    }

    bool contains(const CoordType& x, const CoordType& y, const CoordType& z) const { return contains(Point(x,y,z)); }
  };

  using VisitorFunction = typename std::function<void (const NodeVisitor&)>;

private:
  Node* m_root;

public:
  PointOctree() : m_root(nullptr) {}
 ~PointOctree() = default;

  void insert(const Point& p, const Value& val) {
    Node** tmp = 0;
    if (find(p, tmp)) return;
    (*tmp) = new Node(p, val);
  }

  void insert(const CoordType& x, const CoordType& y, const CoordType& z, const Value& val) { insert(Point(x,y,z), val); }

  NodeVisitor find(const Point& p) {
    Node** tmp = 0;
    if (!find(p, tmp))
      throw std::runtime_error("Point (" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + 
          std::to_string(p.z) + ") not found");

    return NodeVisitor(*tmp);
  }

  NodeVisitor find(const CoordType& x, const CoordType& y, const CoordType& z) { return find(Point(x,y,z)); }

  PointOctree ranged_query(const Cube& cube, const VisitorFunction& visitor = [](auto& n){}) {
    PointOctree subtree;
    ranged_query(m_root, cube, subtree, visitor);
    return subtree;
  }

  void visit_dfs(const std::function<void (const NodeVisitor&)>& visitor, NodeVisitor start = NodeVisitor(0)) {
    if (!m_root) return;

    std::stack<Node*> cont;
    if (!start)
      start.n = m_root;

    cont.push(start.n);
    Node* tmp;
    while (!cont.empty()) {
      tmp = cont.top();
            cont.pop();

      if (visitor)
        visitor(start.n = tmp);

      for (int ii = 0; ii < 8; ++ii)
        if (tmp->childs[ii])
          cont.push(tmp->childs[ii]);
    }
  }

  void visit_bfs(const std::function<void (const NodeVisitor&)>& visitor, NodeVisitor start = NodeVisitor(0)) {
    if (!m_root) return;

    std::queue<Node*> cont;
    if (!start)
      start.n = m_root;

    cont.push(start.n);
    Node* tmp;
    while (!cont.empty()) {
      tmp = cont.front();
            cont.pop();

      visitor(start.n = tmp);

      for (int ii = 0; ii < 8; ++ii)
        if (tmp->childs[ii])
          cont.push(tmp->childs[ii]);
    }
  }


private:

  bool find(const Point& p, Node**& node) {
    node = std::addressof(m_root);
    while (*node) {
      if ((*node)->point == p) return true;
      node = std::addressof((*node)->childs[what_octant(p, *node)]);
    }
    return false;
  }

  // Tell me where this coord locates relative to Node orig
  int what_octant(const Point& p, Node* orig) {
    if (p.x >= orig->point.x) {
      if (p.y >= orig->point.y) {
        if (p.z >= orig->point.z)
          return 0;
        else
          return 4;
      } else {
        if (p.z >= orig->point.z)
          return 3;
        else
          return 7;
      }
    } else {
      if (p.y >= orig->point.y) {
        if (p.z >= orig->point.z)
          return 1;
        else 
          return 5;
      } else {
        if (p.z >= orig->point.z)
          return 2;
        else 
          return 6;
      }
    }
  }

  void ranged_query(Node* n, const Cube& cube, PointOctree& subtree, const VisitorFunction& visitor) {
    if (!n) return;

    if (cube.contains(n->point)) {
      subtree.insert(n->point, n->val);
      if (visitor)
        visitor(NodeVisitor(n));
    }

    if (n->point.x >= cube.min.x) {
      if (n->point.y >= cube.min.y) {
        if (n->point.z >= cube.min.z)
          ranged_query(n->childs[6], cube, subtree, visitor);
        if (n->point.z <= cube.max.z)
          ranged_query(n->childs[2], cube, subtree, visitor);
      }

      if (n->point.y <= cube.max.y) {
        if (n->point.z >= cube.min.z)
          ranged_query(n->childs[5], cube, subtree, visitor);
        if (n->point.z <= cube.max.z)
          ranged_query(n->childs[1], cube, subtree, visitor);
      }
    }
    if (n->point.x <= cube.max.x) {
      if (n->point.y >= cube.min.y) {
        if (n->point.z >= cube.min.z)
          ranged_query(n->childs[7], cube, subtree, visitor);
        if (n->point.z <= cube.max.z)
          ranged_query(n->childs[3], cube, subtree, visitor);
      }

      if (n->point.y <= cube.max.y) {
        if (n->point.z >= cube.min.z)
          ranged_query(n->childs[4], cube, subtree, visitor);
        if (n->point.z <= cube.max.z)
          ranged_query(n->childs[0], cube, subtree, visitor);
      }
    }
  }
  
  //radio_query...
  
};

}

#endif
