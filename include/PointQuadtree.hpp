#ifndef QOAED_POINT_QUADTREE_HPP
#define QOAED_POINT_QUADTREE_HPP

#include <set>
#include <list>
#include <array>
#include <stack>
#include <queue>
#include <iostream>
#include <functional>

#include "Point.hpp"

// Quadrants are identified in this way
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
// See what_quadrant for more information

namespace qoaed {

template <class Value, class CoordType = long>
class PointQuadtree {
public:
  using value_type = Value;
  using reference = Value&;
  using const_reference = Value const&;
  using Point = Point2D<CoordType>;

private:

  class Node;
  using Childs = std::array<Node*, 4>;

  class Node {
  public:
    Point point;
    Childs  childs;
    mutable Value  val;

    Node(const Point& point, const Value& val) : 
      point(point), val(val) { childs = {0,0,0,0}; }

    Node(const CoordType& x, const CoordType& y, const Value& val): 
      point(x,y), val(val) { childs = {0,0,0,0}; }
  };


public:

  class NodeVisitor {
  friend class PointQuadtree;

  private:
    Node* n;

  public:
    NodeVisitor(Node* n): n(n) {}

    const CoordType& get_x() const { return n->point.x; }
    const CoordType& get_y() const { return n->point.y; }
    const Point& get_point() const { return n->point; }
    Value& operator*() const { return n->val; }
    operator   bool()  const { return (bool) n; }
  };

  class Rect {
  friend class PointQuadtree;

  private:
    // min states for the point in the left bottom of the rect
    // max states for the point in the right top of the rect
    Point min;
    Point max;

  public:
    Rect(const Point& min, const Point& max): 
      min(min),
      max(max) {};

    Rect(const CoordType& min_x, const CoordType& min_y, const CoordType& max_x, const CoordType& max_y) :
      min(min_x, min_y),
      max(max_x, max_y) {}

    bool contains(const Point& p) const {
      bool cx, cy;
      cx = (p.x <= max.x && p.x >= min.x);
      cy = (p.y <= max.y && p.y >= min.y);
      return cx && cy;
    }

    bool contains(const CoordType& x, const CoordType& y) const { return contains(Point(x,y)); }
  };

  class Circ {
  friend class PointQuadtree;

  private:
    Point       origin;
    unsigned long radius;

  public:
    Circ(const CoordType& ox, const CoordType& oy, const unsigned long& r) :
      origin(ox,oy), radius(r) {}

    // TODO
    bool contains(const CoordType& x, const CoordType& y) const {
      bool cx, cy;
      return cx && cy;
    }
  };

  using VisitorFunction = typename std::function<void (const NodeVisitor&)>;

private:
  Node* m_root;

public:

  PointQuadtree() : m_root(0) {}

  void insert(const Point& p, const Value& val) {
    Node** tmp;
    if (find(p, tmp)) return;
    (*tmp) = new Node(p, val);
  }

  void insert(const CoordType& x, const CoordType& y, const Value& val) { insert(Point(x,y), val); }

  NodeVisitor find(const Point& p) {
    Node** tmp;
    if (!find(p, tmp))
      throw std::runtime_error("Point (" + std::to_string(p.x) + ", " + std::to_string(p.y) + ") not found");
    return NodeVisitor(*tmp);
  }

  NodeVisitor find(const CoordType& x, const CoordType& y) { return find(Point(x,y)); }
  
  PointQuadtree ranged_query(const Rect& rect, const VisitorFunction& visitor = [](auto& n){}) {
    PointQuadtree subtree;
    ranged_query(m_root, rect, subtree, visitor);
    return subtree;
  }

  //PointQuadtree radio_query(const Circ& circ, const VisitorFunction& visitor = [](auto& n){}) {

  //}
  
  void visit_dfs(const VisitorFunction& visitor, NodeVisitor start = NodeVisitor(0)) {
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

      for (int ii = 0; ii < 4; ++ii) 
        if (tmp->childs[ii])
          cont.push(tmp->childs[ii]);
    }
  }

  void visit_bfs(const VisitorFunction& visitor, NodeVisitor start = NodeVisitor(0)) {
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

      for (int ii = 0; ii < 4; ++ii)
        if (tmp->childs[ii])
          cont.push(tmp->childs[ii]);
    }
  }

private:

  bool find(const Point& p, Node**& node) {
    node = &m_root;
    while (*node) {
      if ((*node)->point == p) return true;
      node = &((*node)->childs[what_quadrant(p,*node)]);
    }
    return false;
  }

  // Tell me where this coord locates relative to Node orig
  int what_quadrant(const Point& p, Node* orig) {
    if (p.x > orig->point.x && p.y >= orig->point.y)
      return 0;
    if (p.x <= orig->point.x && p.y > orig->point.y)
      return 1;
    if (p.x < orig->point.x && p.y <= orig->point.y)
      return 2;
    if (p.x >= orig->point.x && p.y < orig->point.y)
      return 3;
  }

  void ranged_query(Node* n, const Rect& rect, PointQuadtree& subtree, const VisitorFunction& visitor) {
    if (!n) return;

    if (rect.contains(n->point)) {
      subtree.insert(n->point, n->val);
      if (visitor)
        visitor(NodeVisitor(n));
    }

    if (n->point.x <= rect.max.x) {
      if (n->point.y >= rect.min.y)
        ranged_query(n->childs[3], rect, subtree, visitor);
      if (n->point.y <= rect.max.y)
        ranged_query(n->childs[0], rect, subtree, visitor);
    }

    if (n->point.x >= rect.min.x) {
      if (n->point.y >= rect.min.y)
        ranged_query(n->childs[2], rect, subtree, visitor);
      if (n->point.y <= rect.max.y)
        ranged_query(n->childs[1], rect, subtree, visitor);
    }
  }
};

}

#endif
