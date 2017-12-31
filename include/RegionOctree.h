#ifndef QOAED_REGION_OCTREE_HPP
#define QOAED_REGION_OCTREE_HPP

#include <map>
#include <array>

template <class Value, class Key = long>
class RegionOctree {
private:

  struct Obj {
    Key x, y, z;
    mutable Value val;

    Obj(const Key& x, const Key& y,const Key& z, const Value& val) :
      x(x), y(y), z(z), val(val) {}
  };

  struct Node;
  using Childs = typename std::array<Node*, 8>;
  using Box    = typename std::map<Key, std::map<Key, std::map<Key, Obj> > >;

  struct Node {
    Key min_x, min_y, min_z;
    Key max_x, max_y, max_z;
    Childs  childs;
    Box     box;

    Node(const Key& minx, const Key& miny, const Key& minz, const Key& maxx, const Key& maxy, const Key& maxz) :
      min_x(minx), min_y(miny), min_z(minz), max_x(maxx), max_y(maxy), max_z(maxz)
      { childs = {0, 0, 0, 0, 0, 0, 0, 0}; }

    void insert(const Key& x, const Key& y, const Key& z, const Value& val) {
      auto xit = box.find(x);
      if (xit == box.end()) {
        auto insertedx = box.emplace(x, std::map<Key,std::map<Key, Obj>() >());
        auto yit = insertedx->second.find(y);
        if(yit == insertedx->second.end()){
          auto insertedy = insertedx.emplace(y, std::map<Key,Obj>());
          std::get<0>(insertedy)->second.emplace(z, Obj(x,y,z,val));
        } else{
          auto zit = yit->second.find(z);
          if(zit==yit->second.end()){
            zit->second.emplace(z, Obj(x,y,z,val));
          } else {
            return;
          }
        }
      } else {
        auto yit = xit->second.find(y);
        if (yit == xit->second.end())
          xit->second.emplace(y, Obj(x,y,z,val));
        else
          return;
      }
    }

    void remove(const Key& x, const Key& y, const Key& z) {
      auto xit = box.find(x);
      if (xit == box.end()) return;

      auto yit = xit->second.find(y);
      if (yit == xit->second.end()) return;

      auto zit = yit->second.find(z);
      if (zit == yit->second.end()) return;

      yit->second.erase(zit);
      if(yit->second.empty()){
        xit->second.erase(yit);
        if (xit->second.empty())
          box.erase(xit);
      }
    }

    const Obj& find(const Key& x, const Key& y, const Key& z) {
      auto xit = box.find(x);
      if (xit == box.end())
        throw std::runtime_error("Cannot found x coord");

      auto yit = xit->second.find(y);
      if (yit == xit->second.end())
        throw std::runtime_error("Cannot found y coord");

      auto zit = yit->second.find(z);
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
  RegionOctree(const Key& max_x, const Key& max_y, const Key& max_z) : m_root(new Node(0, 0, 0, max_x, max_y, max_z)) {}
 ~RegionOctree() = default;

void insert(const Key& x, const Key& y, const Key& z, const Value& val) {
    Node** tmp;
    if (find(x,y,z,tmp)) return;
    else{
      *tmp=new Node(x,y,z,val);
    }

  }

private:

  bool find(const Key& x, const Key& y, const Key& z, Node**& node) {
    node = &m_root;
    while (*node) {
      try {
        (*node)->find(x,y,z);
        return true;
      } catch(std::exception& e) {
        int pos = what_quadrant(x,y,z,*node);
        node=&((*node)->childs[pos]);
      }
    }
    return false;
  }

  int what_octant(const Key& x, const Key& y, const Key& z, Node* n) {
      Key middle_x = (n->min_x + n->max_x)/Key(2);
      Key middle_y = (n->min_y + n->max_y)/Key(2);
      Key middle_z = (n->min_z + n->max_z)/Key(2);

      // TODO: Esta bien que todas las comparaciones sean menor o igual ; mayor o igual?
      if(z>=n->min_z && z<=middle_z){
        if(x>=middle_x && x<=n->max_x){
            if(y>=middle_y && y<=n->max_y) return 0;
            else if(y>=n->min_y && y<middle_y) return 3;
        }
        else if(x>=n->min_x && x<middle_x){
            if(y>=middle_y && y<=n->max_y) return 1;
            else if(y>=n->min_y && y<middle_y) return 2;
        }
      }
      else if(z>middle_z && z<=n->max_z){
        if(x>=middle_x && x<=n->max_x){
            if(y>=middle_y && y<=n->max_y) return 5;
            else if(y>=n->min_y && y<middle_y) return 4;
        }
        else if(x>=n->min_x && x<middle_x){
            if(y>=middle_y && y<=n->max_y) return 6;
            else if(y>=n->min_y && y<middle_y) return 7;
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


#endif
