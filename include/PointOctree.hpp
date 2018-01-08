#ifndef QOAED_POINT_OCTREE_HPP
#define QOAED_POINT_OCTREE_HPP 

#include <set>
#include <list>
#include <array>
#include <stack>
#include <queue>
#include <functional>
#include <cmath>

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
    Node*   parent;
    Node*   childs[8];
    mutable Value val;

    Node(const Point& p, const Value& val) : 
      point(p), parent(0), val(val) { 
        for (int i = 0; i < 8; ++i)
          childs[i] = 0;
      }

    Node(const CoordType& x, const CoordType& y, const CoordType& z, const Value& val):
      point(x,y,z), parent(0), val(val) { 
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
    // both points for a diagonal and it's guaranteed that 
    // they are respectively min and max coordenates of cube
    Point min;
    Point max;

  public:
    Cube(const Point& bot, const Point& top) {
      min.x = std::min(bot.x, top.x);
      min.y = std::min(bot.y, top.y);
      min.z = std::min(bot.z, top.z);

      max.x = std::max(bot.x, top.x);
      max.y = std::max(bot.y, top.y);
      max.z = std::max(bot.z, top.z);
    }

    Cube(const CoordType& bot_x, const CoordType& bot_y, const CoordType& bot_z,
         const CoordType& top_x, const CoordType& top_y, const CoordType& top_z) :
      Cube(Point(bot_x, bot_y, bot_z), Point(top_x, top_y, top_z)) {}

    Cube(const Point& center, double radio) {
      if (radio < 0) throw std::runtime_error("Radio cannot be negative");
      if (!std::is_same<CoordType, double>::value && !std::is_same<CoordType, float>::value)
        radio = std::round(radio);

      min.x = center.x - (CoordType) radio;
      min.y = center.y - (CoordType) radio;
      min.z = center.z - (CoordType) radio;

      max.x = center.x + (CoordType) radio;
      max.y = center.y + (CoordType) radio;
      max.z = center.z + (CoordType) radio;
    }

    bool contains(const Point& p) const {
      bool cx, cy, cz;
      cx = (p.x <= max.x && p.x >= min.x);
      cy = (p.y <= max.y && p.y >= min.y);
      cz = (p.z <= max.z && p.z >= min.z);
      return cx && cy && cz;
    }

    bool contains(const CoordType& x, const CoordType& y, const CoordType& z) const { return contains(Point(x,y,z)); }
  };

  class Sphere {
  friend class PointOctree;
  
  private:
    Point  center;
    double radio;

  public:
    Sphere(const Point& center, const double& radio) : center(center), radio(radio) {}

    Sphere(const CoordType& cx, const CoordType& cy, const double& radio) :
      center(cx, cy), radio(radio) {}

    Sphere(const Point& center, const Point& super) : center(center) {
      radio = std::sqrt(center.distance_wo_sqrt(super));
    }

    bool contains(const Point& p) const { return center.distance_wo_sqrt(p) <= (radio * radio); }
    bool contains(const CoordType& x, const CoordType& y, const CoordType& z) const { return contains(Point(x,y,z)); }
  };

  using VisitorFunction = typename std::function<void (const NodeVisitor&)>;

private:
  Node* m_root;

public:
  PointOctree() : m_root(nullptr) {}
 ~PointOctree() = default;

  void insert(const Point& p, const Value& val) {
    Node** tmp    = 0;
    Node*  parent = 0;
    if (find(p, tmp, parent)) return;   

    Node* n   = new Node(p, val);
    (*tmp)    = n;
    n->parent = parent;
  }

  void insert(const CoordType& x, const CoordType& y, const CoordType& z, const Value& val) { insert(Point(x,y,z), val); }

  NodeVisitor find(const Point& p) {
    Node** tmp    = 0;
    Node*  parent = 0;
    find(p, tmp, parent);
    return NodeVisitor(*tmp);
  }

  NodeVisitor find(const CoordType& x, const CoordType& y, const CoordType& z) { return find(Point(x,y,z)); }

  PointOctree cubic_query(const Cube& cube, const VisitorFunction& visitor = [](auto& n){}) {
    PointOctree subtree;
    cubic_query(m_root, cube, subtree, visitor);
    return subtree;
  }

  PointOctree spheric_query(const Sphere& sphere, const VisitorFunction& visitor = [](auto& n){}) {
    PointOctree subtree;
    spheric_query(m_root, sphere, subtree, visitor);
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

  bool find(const Point& p, Node**& node, Node*& parent) {
    node   = std::addressof(m_root);   
    parent = nullptr; 
    while (*node) {
      if ((*node)->point == p) return true;
      parent = (*node);
      node   = std::addressof((*node)->childs[what_octant(p, *node)]);
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

  void cubic_query(Node* n, const Cube& cube, PointOctree& subtree, const VisitorFunction& visitor) {
    if (!n) return;

    if (cube.contains(n->point)) {
      subtree.insert(n->point, n->val);
      if (visitor)
        visitor(NodeVisitor(n));
    }

    if (n->point.x >= cube.min.x) {
      if (n->point.y >= cube.min.y) {
        if (n->point.z >= cube.min.z)
          cubic_query(n->childs[6], cube, subtree, visitor);
        if (n->point.z <= cube.max.z)
          cubic_query(n->childs[2], cube, subtree, visitor);
      }

      if (n->point.y <= cube.max.y) {
        if (n->point.z >= cube.min.z)
          cubic_query(n->childs[5], cube, subtree, visitor);
        if (n->point.z <= cube.max.z)
          cubic_query(n->childs[1], cube, subtree, visitor);
      }
    }

    if (n->point.x <= cube.max.x) {
      if (n->point.y >= cube.min.y) {
        if (n->point.z >= cube.min.z)
          cubic_query(n->childs[7], cube, subtree, visitor);
        if (n->point.z <= cube.max.z)
          cubic_query(n->childs[3], cube, subtree, visitor);
      }

      if (n->point.y <= cube.max.y) {
        if (n->point.z >= cube.min.z)
          cubic_query(n->childs[4], cube, subtree, visitor);
        if (n->point.z <= cube.max.z)
          cubic_query(n->childs[0], cube, subtree, visitor);
      }
    }
  }

  void spheric_query(Node* n, const Sphere& sphere, PointOctree& subtree, const VisitorFunction& visitor) {
    if (!n) return;

    if (sphere.contains(n->point)) {
      subtree.insert(n->point, n->val);
      if (visitor)
        visitor(NodeVisitor(n));
    }

    Cube cube(sphere.center, sphere.radio);

    if (n->point.x >= cube.min.x) {
      if (n->point.y >= cube.min.y) {
        if (n->point.z >= cube.min.z)
          spheric_query(n->childs[6], sphere, subtree, visitor);
        if (n->point.z <= cube.max.z)
          spheric_query(n->childs[2], sphere, subtree, visitor);
      }

      if (n->point.y <= cube.max.y) {
        if (n->point.z >= cube.min.z)
          spheric_query(n->childs[5], sphere, subtree, visitor);
        if (n->point.z <= cube.max.z)
          spheric_query(n->childs[1], sphere, subtree, visitor);
      }
    }

    if (n->point.x <= cube.max.x) {
      if (n->point.y >= cube.min.y) {
        if (n->point.z >= cube.min.z)
          spheric_query(n->childs[7], sphere, subtree, visitor);
        if (n->point.z <= cube.max.z)
          spheric_query(n->childs[3], sphere, subtree, visitor);
      }

      if (n->point.y <= cube.max.y) {
        if (n->point.z >= cube.min.z)
          spheric_query(n->childs[4], sphere, subtree, visitor);
        if (n->point.z <= cube.max.z)
          spheric_query(n->childs[0], sphere, subtree, visitor);
      }
    }
  }
  
public:

    double distance(Point a,Point b)
   {
       return sqrt(pow(b.x-a.x,2.0)+pow(b.y-a.y,2.0)+pow(b.z-a.z,2.0)); 
   }
   
    void spheric_query_auxiliar(Node* n, const Sphere& sphere, PointOctree& subtree, const VisitorFunction& visitor, int avoid_octant) {
    if (!n) return;

    if (sphere.contains(n->point)) {
      subtree.insert(n->point, n->val);
      if (visitor)
        visitor(NodeVisitor(n));
    }

    Cube cube(sphere.center, sphere.radio);

    if (n->point.x >= cube.min.x) {
      if (n->point.y >= cube.min.y) {
        if (n->point.z >= cube.min.z && avoid_octant != 6)
          spheric_query_auxiliar(n->childs[6], sphere, subtree, visitor,-1);
        if (n->point.z <= cube.max.z && avoid_octant != 2)
          spheric_query_auxiliar(n->childs[2], sphere, subtree, visitor,-1);
      }

      if (n->point.y <= cube.max.y) {
        if (n->point.z >= cube.min.z && avoid_octant != 5)
          spheric_query_auxiliar(n->childs[5], sphere, subtree, visitor,-1);
        if (n->point.z <= cube.max.z  && avoid_octant != 1)
          spheric_query_auxiliar(n->childs[1], sphere, subtree, visitor,-1);
      }
    }

    if (n->point.x <= cube.max.x) {
      if (n->point.y >= cube.min.y) {
        if (n->point.z >= cube.min.z  && avoid_octant != 7)
          spheric_query_auxiliar(n->childs[7], sphere, subtree, visitor, -1);
        if (n->point.z <= cube.max.z && avoid_octant != 3)
          spheric_query_auxiliar(n->childs[3], sphere, subtree, visitor, -1);
      }

      if (n->point.y <= cube.max.y) {
        if (n->point.z >= cube.min.z && avoid_octant != 4)
          spheric_query_auxiliar(n->childs[4], sphere, subtree, visitor, -1);
        if (n->point.z <= cube.max.z && avoid_octant != 0)
          spheric_query_auxiliar(n->childs[0], sphere, subtree, visitor, -1);
      }
    }
  }
   
    bool padre_encierra_esfera(Point point, const double r, Point origin)
    {
        if(origin.x >= 0)
        {
            if(origin.y >=0)
            {
                if(origin.z >= 0)
                    return (origin.x - r >= point.x && origin.y -r >= point.y && origin.z -r >= point.z);//0
                else
                    return (origin.x - r >= point.x && origin.y -r >= point.y && origin.z +r >= point.z);//4
           }else
           {
                if(origin.z >= 0)
                     return (origin.x - r >= point.x && origin.y +r >= point.y && origin.z -r >= point.z);//3
                else
                     return (origin.x - r >= point.x && origin.y +r >= point.y && origin.z +r >= point.z);//7
           }
        }
        else
        {
            
            if(origin.y >=0)
            {
                if(origin.z >= 0)
                    return (origin.x + r >= point.x && origin.y -r >= point.y && origin.z -r >= point.z); //1
                else
                    return (origin.x + r >= point.x && origin.y -r >= point.y && origin.z +r >= point.z); //5
           }else
           {
                if(origin.z >= 0)
                     return (origin.x + r >= point.x && origin.y +r >= point.y && origin.z -r >= point.z); //2
                else
                     return (origin.x + r >= point.x && origin.y +r >= point.y && origin.z +r >= point.z); //6
           }
        }
    }
   
    PointOctree spheric_query_inner(Point origin, const double radio, const VisitorFunction& visitor = [](auto& n){})
    {
        Node** node_point;
        Node* node_parent;
        find (origin, node_point, node_parent);//node_parent no es usado
        
        Node* padre = *node_point;     
        PointOctree subtree;
        int octante_hijo = -1;
        int octante_complementario[8];
        octante_complementario[0]= 6;
        octante_complementario[6]= 0;
        octante_complementario[4]= 2;
        octante_complementario[2]= 4;
        octante_complementario[5]= 3;
        octante_complementario[3]= 5;
        octante_complementario[1]= 7;
        octante_complementario[7]= 1;
        std::set<int> octantes_hallados;
        
        if(padre != nullptr){
        
            bool considera = true;
            
            while(true){
              
                if(considera){
                    if(distance(padre->point, origin) <= radio) //ver si distancia de padre a origin esta dentro de radio
                    {
                        subtree.insert(padre->point, padre->val);
                        if (visitor)
                            visitor(NodeVisitor(padre));
                    }
                    
                    Sphere sphere(origin, radio); 
                    //ver si hijos de padre estan dentro de radio, escepto el hijo que es de octante_hijo
                    spheric_query_auxiliar(padre, sphere, subtree, visitor, octante_hijo);
                }
                
                if(padre->parent == nullptr) {return subtree;}
                octante_hijo = what_octant(padre->point, padre->parent);
                padre = padre->parent;
                
                if(padre_encierra_esfera(padre->point, radio, origin)){
                    if(octantes_hallados.find(octante_complementario[octante_hijo]) != octantes_hallados.end())
                        return subtree;
                    else
                    {
                        octantes_hallados.insert(octante_hijo);
                        considera = false;
                    }
                }
        }
        
    }
  }
  
};


}

#endif
