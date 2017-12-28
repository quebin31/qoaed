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

  struct Node;
  using  Childs = std::array<Node*, 8>;

  struct Node {
    Key     x, y, z;
    Childs  childs;
    mutable Value val;

    Node(const Key& x, const Key& y, const Key& z, const Value& val):
      x(x), y(y), z(z), val(val) { childs = {0, 0, 0, 0, 0, 0, 0, 0}; }
  };

public:
  
  class NodeVisitor {
  private:
    Node* n;
    friend class PointOctree;

  public:
    NodeVisitor(Node* n): n(n) {}
    
    const Key& get_x() const { return n->x; }
    const Key& get_y() const { return n->y; }
    const Key& get_z() const { return n->z; }
    Value& operator*() const { return n->val; }
    operator    bool() const { return (bool) n; }
  };

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

      for (int ii = 0; ii < 8; ++ii)
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

};

}

#endif
