#ifndef QOAED_POINT_OCTREE_HPP
#define QOAED_POINT_OCTREE_HPP 

#include <set>
#include <list>
#include <array>
#include <stack>
#include <queue>
#include <functional>

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

template <class Value, class Key = long>
class PointOctree {
private: 

  class Node;
  using Childs = std::array<Node*, 8>;

  class Node {
  public:
    Key     x, y, z;
    Childs  childs;
    mutable Value val;

    Node(const Key& x, const Key& y, const Key& z, const Value& val):
      x(x), y(y), z(z), val(val) { childs = {0, 0, 0, 0, 0, 0, 0, 0}; }
  };

public:
  
  class NodeVisitor {
  friend class PointOctree;

  private:
    Node* n;

  public:
    NodeVisitor(Node* n): n(n) {}
    
    const Key& get_x() const { return n->x; }
    const Key& get_y() const { return n->y; }
    const Key& get_z() const { return n->z; }
    Value& operator*() const { return n->val; }
    operator    bool() const { return (bool) n; }
  };

  class Cube {
  friend class PointOctree;

  private:
    Key min_x, min_y, min_z;
    Key max_x, max_y, max_z;

  public:
    Cube(const Key& min_x, const Key& min_y, const Key& min_z,
         const Key& max_x, const Key& max_y, const Key& max_z) :
      min_x(min_x), min_y(min_y), min_z(min_z),
      max_x(max_x), max_y(max_y), max_z(max_z)  {}

    bool contains(const Key& x, const Key& y, const Key& z) {
      bool cx, cy, cz;
      cx = (x <= max_x && x >= min_x);
      cy = (y <= max_y && y >= min_y);
      cz = (z <= max_z && z >= min_z);
      return cx && cy && cz;
    }
  };

  using VisitorFunction = typename std::function<void (const NodeVisitor&)>;

private:
  Node* m_root;

public:
  PointOctree() : m_root(0) {}
 ~PointOctree() = default;

  void insert(const Key& x, const Key& y, const Key& z, const Value& val) {
    Node** tmp;

    if (find(x,y,z,tmp)) return;
    (*tmp) = new Node(x,y,z,val);
  }

  NodeVisitor find(const Key& x, const Key& y, const Key& z) {
    Node** tmp;
    if (!find(x,y,z,tmp))
      throw std::runtime_error("Point (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ") not found");

    return NodeVisitor(*tmp);
  }

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

  bool find(const Key& x, const Key& y, const Key& z, Node**& node) {
    node = &m_root;
    while (*node) {
      if ((*node)->x == x && (*node)->y == y && (*node)->z == z) return true;
      node = &((*node)->childs[what_octant(x,y,z,*node)]);
    }
    return false;
  }

  // Tell me where this coord locates relative to Node orig
  int what_octant(const Key& x, const Key& y, const Key& z, Node* orig) {
    if (x > orig->x && y >= orig->y) {
      if (z >= orig->z)
        return 0;
      else 
        return 4;
    }

    if (x <= orig->x && y > orig->y) {
      if (z >= orig->z)
        return 1;
      else 
        return 5;
    }

    if (x < orig->x && y <= orig->y) {
      if (z >= orig->z)
        return 2;
      else 
        return 6;
    }

    if (x >= orig->x && y < orig->y) {
      if (z >= orig->z)
        return 3;
      else 
        return 7;
    }
  }

  void ranged_query(Node* n, const Cube& cube, PointOctree& subtree, const VisitorFunction& visitor) {
    if (!n) return;

    if (cube.contains(n->x, n->y, n->z)) { 
      subtree.insert(n->x, n->y, n->z, n->val);
      if (visitor)
        visitor(NodeVisitor(n));
    }

    if (n->x >= cube.min_x) {
      if (n->y >= cube.min_y) {
        if (n->z >= cube.min_z)
          ranged_query(n->childs[6], cube, subtree, visitor);
        if (n->z <= cube.max_z)
          ranged_query(n->childs[2], cube, subtree, visitor);
      }

      if (n->y <= cube.max_y) {
        if (n->z >= cube.min_z)
          ranged_query(n->childs[5], cube, subtree, visitor);
        if (n->z <= cube.max_z)
          ranged_query(n->childs[1], cube, subtree, visitor);
      }
    }

    if (n->x <= cube.max_x) {
      if (n->y >= cube.min_y) {
        if (n->z >= cube.min_z)
          ranged_query(n->childs[7], cube, subtree, visitor);
        if (n->z <= cube.max_z)
          ranged_query(n->childs[3], cube, subtree, visitor);
      }

      if (n->y <= cube.max_y) {
        if (n->z >= cube.min_z)
          ranged_query(n->childs[4], cube, subtree, visitor);
        if (n->z <= cube.max_z)
          ranged_query(n->childs[0], cube, subtree, visitor);
      }
    }
  }
};

}

#endif
