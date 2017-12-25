#ifndef QOAED_POINT_OCTREE_HPP
#define QOAED_POINT_OCTREE_HPP 

#include <set>
#include <list>
#include <array>

namespace qoaed {

template <class Value, class Key = long>
class Octree {
private: 

  struct Node;
  using Childs = std::array<Node*, 8>;

  struct Node {
    Key    x, y, z;
    Value  val;
    Childs childs;

    Node(const Key& x, const Key& y, const Key& z, const Value& val):
      x(x), y(y), z(z), val(val) { childs = {0, 0, 0, 0, 0, 0, 0, 0}; }
  };

public:
  
  class NodeVisitor {
  private:
    Node* n;

  public:
    NodeVisitor(Node* n): n(n) {}
    
    const Key& get_x() const { return n->x; }
    const Key& get_y() const { return n->y; }
    const Key& get_z() const { return n->z; }
    Value& operator*() const { return n->val; }
    operator   bool()  const { return (bool) n; }
  };

private:
  Node* m_root;

};

}

#endif
