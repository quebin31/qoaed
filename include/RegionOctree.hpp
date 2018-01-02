#ifndef QOAED_REGION_OCTREE_HPP
#define QOAED_REGION_OCTREE_HPP

#include <map>
#include <array>

#include "Point.hpp"

namespace qoaed {
template <class Value, class Key = long>
class RegionOctree {
private:

  struct Obj {
    Point3D<Key> point;
    mutable Value val;

    Obj(const Key& x, const Key& y,const Key& z, const Value& val) :
    point(x,y,z), val(val) {}
  };

  struct Node;
  using Childs = typename std::array<Node*, 8>;
  using Box    = typename std::map<Key, std::map<Key, std::map<Key, Obj> > >;

  struct Node {
    Point3D<Key> min;
    Point3D<Key> max;
    Childs  childs;
    Box     box;

    Node(const Key& minx, const Key& miny, const Key& minz, const Key& maxx, const Key& maxy, const Key& maxz) :
      min(minx,miny,minz), max(maxx,maxy,maxz)
      { childs = {0, 0, 0, 0, 0, 0, 0, 0}; }

    void insert(const Point3D<Key>& p, const Value& val) {
      auto xit = box.find(p.x);
      if (xit == box.end()) {
        auto insertedx = box.emplace(p.x, std::map<Key,std::map<Key, Obj>() >());
        auto yit = insertedx->second.find(p.y);
        if(yit == insertedx->second.end()){
          auto insertedy = insertedx.emplace(p.y, std::map<Key,Obj>());
          std::get<0>(insertedy)->second.emplace(p.z, Obj(p.x,p.y,p.z,val));
        } else{
          auto zit = yit->second.find(p.z);
          if(zit==yit->second.end()){
            zit->second.emplace(p.z, Obj(p.x,p.y,p.z,val));
          } else {
            return;
          }
        }
      } else {
        auto yit = xit->second.find(p.y);
        if (yit == xit->second.end())
          xit->second.emplace(p.y, Obj(p.x,p.y,p.z,val));
        else
          return;
      }
    }

    void remove(const Point3D<Key>& p) {
      auto xit = box.find(p.x);
      if (xit == box.end()) return;

      auto yit = xit->second.find(p.y);
      if (yit == xit->second.end()) return;

      auto zit = yit->second.find(p.z);
      if (zit == yit->second.end()) return;

      yit->second.erase(zit);
      if(yit->second.empty()){
        xit->second.erase(yit);
        if (xit->second.empty())
          box.erase(xit);
      }
    }

    const Obj& find(const Point3D<Key>& p) {
      auto xit = box.find(p.x);
      if (xit == box.end())
        throw std::runtime_error("Cannot found x coord");

      auto yit = xit->second.find(p.y);
      if (yit == xit->second.end())
        throw std::runtime_error("Cannot found y coord");

      auto zit = yit->second.find(p.z);
      if(zit == yit->second.end())
        throw std:: runtime_error("Cannot found z coord");

      return zit->second;
    }

    // A leaf node doesn't has childs
    // A non-leaf node has all its childs
    bool is_leaf() { return !((bool) childs[0]); }
  };

  Node* m_root;

public:
  RegionOctree(const Point3D<Key>& p) : m_root(new Node(0, 0, 0, p.x, p.y, p.z)) {}
 ~RegionOctree() = default;

  void insert(const Point3D<Key>& p, const Value& val) {
    Node** tmp;
    if (find(p.x,p.y,p.z,tmp)) return;
    (*tmp) = new Node(p.x,p.y,p.z,val);
  }
  void insert(const Key& x, const Key& y, const Key& z, Value& val){
    insert(Point3D<Key>(x,y,z), val);
  }

private:

  bool find(const Point3D<Key>& p, Node**& node) {
    node = &m_root;
    while (*node) {
      try {
        (*node)->find(p);
        return true;
      } catch(std::exception& e) {
        int pos = what_quadrant(p,*node);
        node=&((*node)->childs[pos]);
      }
    }
    return false;
  }

  bool find(const Key& x, const Key& y,const Key& z, Node**& node){
    return find(Point3D<Key>(x,y,z), node);
  }

  int what_octant(const Point3D<Key>& p, Node* n) {
      Key middle_x = (n->min_x + n->max_x)/Key(2);
      Key middle_y = (n->min_y + n->max_y)/Key(2);
      Key middle_z = (n->min_z + n->max_z)/Key(2);

      // TODO: Esta bien que todas las comparaciones sean menor o igual ; mayor o igual?
      if(p.z>=n->min_z && p.z<=middle_z){
        if(p.x>=middle_x && p.x<=n->max_x){
            if(p.y>=middle_y && p.y<=n->max_y) return 0;
            else if(p.y>=n->min_y && p.y<middle_y) return 3;
        }
        else if(p.x>=n->min_x && p.x<middle_x){
            if(p.y>=middle_y && p.y<=n->max_y) return 1;
            else if(p.y>=n->min_y && p.y<middle_y) return 2;
        }
      }
      else if(p.z>middle_z && p.z<=n->max_z){
        if(p.x>=middle_x && p.x<=n->max_x){
            if(p.y>=middle_y && p.y<=n->max_y) return 5;
            else if(p.y>=n->min_y && p.y<middle_y) return 4;
        }
        else if(p.x>=n->min_x && p.x<middle_x){
            if(p.y>=middle_y && p.y<=n->max_y) return 6;
            else if(p.y>=n->min_y && p.y<middle_y) return 7;
        }
      }
  }

  void subdivide(Node* n) {
    Key middle_x = (n->min_x + n->max_x)/Key(2);
    Key middle_y = (n->min_y + n->max_y)/Key(2);
    Key middle_z = (n->min_z + n->max_z)/Key(2);

    n->childs[0] = new Node(middle_x, middle_y, n->min_z, n->max_x, n->max_y, middle_z);
    n->childs[1] = new Node(n->min_x, middle_y, n->min_z, middle_x, n->max_y, middle_z);
    n->childs[2] = new Node(n->min_x, n->min_y, n->min_z, middle_x, middle_y, middle_z);
    n->childs[3] = new Node(middle_x, n->min_y, n->min_z, n->max_x, middle_y, middle_z);

    n->childs[4] = new Node(middle_x, n->min_y, middle_z, n->max_x, middle_y, n->max_z);
    n->childs[5] = new Node(middle_x, middle_y, middle_z, n->max_x, n->max_y, n->max_z);
    n->childs[6] = new Node(n->min_x, middle_y, middle_z, middle_x, n->max_y, n->max_z);
    n->childs[7] = new Node(n->min_x, n->min_y, middle_z, middle_x, middle_y, n->max_z);
  }

};
}

#endif
