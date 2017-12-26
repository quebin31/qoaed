#ifndef QOAED_POINT_QUADTREE_HPP
#define QOAED_POINT_QUADTREE_HPP

#include <set>
#include <list>
#include <array>
#include <stack>
#include <queue>
#include <functional>


namespace qoaed {

template <class Value, class Key = long>
class PointQuadtree {
private:

  struct Node;
  using Childs = std::array<Node*, 4>;

  struct Node {
    Key    x, y;
    Value  val;
    Childs childs;

    Node(const Key& x, const Key& y, const Value& val):
      x(x), y(y), val(val) { childs = {0, 0, 0, 0}; }
  };

public:

  class NodeVisitor {
  private:
    Node* n;
    friend class PointQuadtree;

  public:
    NodeVisitor(Node* n): n(n) {}

    const Key& get_x() const { return n->x; }
    const Key& get_y() const { return n->y; }
    Value& operator*() const { return n->val; }
    operator   bool()  const { return (bool) n; }
  };

  using Nodes = std::list<NodeVisitor>;

  struct Rect {
    Key min_x, min_y;
    Key max_x, max_y;

    Rect(const Key& min_x, const Key& min_y, const Key& max_x, const Key& max_y):
      min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y) {}

    bool contains(const Key& x, const Key& y) const {
      bool cx, cy;
      cx = (x <= max_x && x >= min_x);
      cy = (y <= max_y && x >= min_y);
      return cx && cy;
    }
  };

private:
  Node* m_root;

  enum Quadrants { NW, NE, SE, SW };

public:

  PointQuadtree() : m_root(0) {}

  void insert(const Key& x, const Key& y, const Value& val) {
    Node** tmp;
    if (find(x,y,tmp)) return;
    (*tmp) = new Node(x,y,val);
  }

  PointQuadtree ranged_query(const Rect& rect) {
    PointQuadtree subtree;
    ranged_query(m_root, rect, subtree);
    return subtree;
  }

  //Nodes spherical_query(const Key& x, const Key& y, const Key& radius) {

  //}
  
  void visit_dfs(const std::function<void (const Node&)>& visitor, NodeVisitor start = NodeVisitor(0)) {
    if (!m_root) return;

    std::stack<Node*> cont;
    if (!start)
      start.n = m_root;

    cont.push(start.n);
    Node* tmp;
    while (!cont.empty()) {
      tmp = cont.top();
            cont.pop();

      visitor(*tmp);

      for (int ii = 0; ii < 4; ++ii) 
        if (tmp->childs[ii])
          cont.push(tmp->childs[ii]);
    }
  }

  void visit_bfs(const std::function<void (const Node&)>& visitor, NodeVisitor start = NodeVisitor(0)) {
    if (!m_root) return;

    std::queue<Node*> cont;
    if (!start)
      start.n = m_root;


    cont.push(start.n);
    Node* tmp;
    while (!cont.empty()) {
      tmp = cont.front();
            cont.pop();

      visitor(*tmp);

      for (int ii = 0; ii < 4; ++ii)
        if (tmp->childs[ii])
          cont.push(tmp->childs[ii]);
    }
  }

private:

  bool find(const Key& x, const Key& y, Node**& node) {
    node = &m_root;
    while (*node) {
      if ((*node)->x == x && (*node)->y == y) return true;
      node = &((*node)->childs[what_quadrant(x,y,*node)]);
    }
    return false;
  }

  // Tell me where this coord locates relative to Node orig
  int what_quadrant(const Key& x, const Key& y, Node* orig) {
    if (x < orig->x && y < orig->y)
      return SW;
    if (x < orig->x && y > orig->y)
      return NW;
    if (x > orig->x && y < orig->y)
      return SE;
    if (x > orig->x && y > orig->y)
      return NE;
  }

  void ranged_query(Node* n, const Rect& rect, PointQuadtree& subtree) {
    if (!n) return;

    if (rect.contains(n->x, n->y)) subtree.insert(n->x, n->y, n->val);

    if (rect.min_x < n->x && rect.min_y < n->y)
      ranged_query(n->childs[SW], rect, subtree);

    if (rect.min_x < n->x && rect.min_y > n->y)
      ranged_query(n->childs[NW], rect, subtree);

    if (rect.max_x > n->x && rect.max_y < n->y)
      ranged_query(n->childs[NE], rect, subtree);

    if (rect.max_x > n->x && rect.max_y > n->y)
      ranged_query(n->childs[SE], rect, subtree);
  }

};

}

#endif
