#ifndef QOAED_REGION_OCTREE_HPP
#define QOAED_REGION_OCTREE_HPP

#include "Point.hpp"
#include <iostream>
#include <map>

template <class Value, class CoordType=long>
class RegionOctree{
public:
	using Point = Point3D<CoordType>;

private:

	struct Obj{
		Point point;
		Value value;
		Obj(const Point& point, const Value& value): point(point), value(value) {}
		Obj(const CoordType& x, const CoordType& y, const CoordType& z, const Value& value) : 
		point(x,y,z), value(value) {} 
	};

	struct Octant;
	using Childs = typename std::array<Octant*, 8>;
	using Box = typename std::map<CoordType, std::map<CoordType, std::map<CoordType, Obj> > >;

	struct Octant{
		Point min;
		Point max;
		Childs childs;
		Box box;
		int s;
		Octant(const Point& min, const Point& max) : min(min), max(max), s(0) {
			for(int i=0; i<8; i++)
				childs[i]=0;
		}
		Octant (const CoordType& minx, const CoordType& miny, const CoordType& minz, 
			   	const CoordType& maxx, const CoordType& maxy, const CoordType& maxz) :
			   	min(minx,miny,minz), max(maxx, maxy, maxz), s(0) {
			for(int i=0; i<8; i++)
				childs[i]=0;			   		
		}

	    void insert(const Point& p, const Value& val) {
	    	s++;
	    }

	    void insert(const CoordType& x, const CoordType& y, const CoordType& z, const Value& val){
	      	insert(Point(x,y,z), val);
	    }

	    void remove(const Point& p) {
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

	    void remove(const CoordType& x, const CoordType& y, const CoordType& z, Value& val){
	      	remove(Point(x,y,z));
	    }

		const Obj& find(const Point& p) {
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

		bool contains(const Point& p) const {
			bool cx, cy, cz;
			cx = (p.x<= max.x && p.x >min.x);
			cy = (p.y<= max.y && p.y >min.y);
			cz = (p.z<= max.z && p.z >=min.z);

			return cx && cy && cz;
		}

		bool contains(const CoordType& x, const CoordType& y, const CoordType& z) const { return contains(Point(x,y,z)); }

	};
public:
	class OctantVisitor {
	friend class RegionOctree;
	private:
		Octant* o;
	public:
		OctantVisitor(Octant* o) : o(o) {}

		const Point& get_PointMin() const { return o->min; }
		const Point& get_PointMax() const { return o->max; }
		operator bool() const { return (bool) o; } 

	};

	class Cube {
	friend class RegionOctree;
	private:
		Point min;
		Point max;
	public:
		Cube(const Point& bot, const Point& top) {
			min.x = std::min(bot.x, top.x);
			min.y = std::min(bot.y, top.y);
			min.z = std::min(bot.z, top.z);

			max.x = std::min(bot.x, top.x);
			max.y = std::min(bot.y, top.y);
			max.z = std::min(bot.z, top.z);
		}

		Cube(const CoordType& bx, const CoordType& by, const CoordType& bz,
			const CoordType& tx, const CoordType& ty, const CoordType& tz) :
			Cube(Point(bx,by,bz), Point(tx,ty,tz)) {}
		Cube(const Point& center, double radio){
			if(radio<0) throw std::runtime_error("Radio cannot be negative");

		}
	};
private:
	Octant* m_root;
public:
	RegionOctree() : m_root(nullptr) {}
	~RegionOctree() = default;
	
	void insert(const Point& p, const Value& val) {
    	Octant** tmp;
    	if (find(p.x,p.y,p.z,tmp)) return;
    	if ((*tmp)->s < 100){
    		//(*tmp)->insert(p,val);	
    	}
    	//subdivide((*tmp));
    	//int pos = what_octant(p, (*tmp));
    	//(*tmp)->childs[pos]->insert(p, val);
  	}

  	void insert(const CoordType& x, const CoordType& y, const CoordType& z,const Value& val){
    	insert(Point(x,y,z), val);
  	}
private:
  	bool find(const Point& p, Octant**& octant) {
    	octant = &m_root;
    	while (*octant) {
      		try {
        		(*octant)->find(p);
        		return true;
      		} catch(std::exception& e) {
        		int pos = what_octant(p,*octant);
        		octant=&((*octant)->childs[pos]);
      		}
    	}
    	return false;
    }

  	bool find(const CoordType& x, const CoordType& y,const CoordType& z, Octant**& octant){
    	return find(Point(x,y,z), octant);
  	}

  	int what_octant(const Point& p, Octant* n) {
      	CoordType middle_x = (n->min.x + n->max.x)/CoordType(2);
      	CoordType middle_y = (n->min.y + n->max.y)/CoordType(2);
      	CoordType middle_z = (n->min.z + n->max.z)/CoordType(2);

      	// TODO: Esta bien que todas las comparaciones sean menor o igual ; mayor o igual?
      if(p.z>=n->min.z && p.z<=middle_z){
        if(p.x>=middle_x && p.x<=n->max.x){
            if(p.y>=middle_y && p.y<=n->max.y) return 0;
            else if(p.y>=n->min.y && p.y<middle_y) return 3;
        }
        else if(p.x>=n->min.x && p.x<middle_x){
            if(p.y>=middle_y && p.y<=n->max.y) return 1;
            else if(p.y>=n->min.y && p.y<middle_y) return 2;
        }
      }
      else if(p.z>middle_z && p.z<=n->max.z){
        if(p.x>=middle_x && p.x<=n->max.x){
            if(p.y>=middle_y && p.y<=n->max.y) return 5;
            else if(p.y>=n->min.y && p.y<middle_y) return 4;
        }
        else if(p.x>=n->min.x && p.x<middle_x){
            if(p.y>=middle_y && p.y<=n->max.y) return 6;
            else if(p.y>=n->min.y && p.y<middle_y) return 7;
        }
      }
  	}

  	void subdivide(Octant* &n) {
	    CoordType middle_x = (n->min.x + n->max.x)/CoordType(2);
	    CoordType middle_y = (n->min.y + n->max.y)/CoordType(2);
	    CoordType middle_z = (n->min.z + n->max.z)/CoordType(2);

	    n->childs[0] = new Octant(middle_x, middle_y, n->min.z, n->max.x, n->max.y, middle_z);
	    n->childs[1] = new Octant(n->min.x, middle_y, n->min.z, middle_x, n->max.y, middle_z);
	    n->childs[2] = new Octant(n->min.x, n->min.y, n->min.z, middle_x, middle_y, middle_z);
	    n->childs[3] = new Octant(middle_x, n->min.y, n->min.z, n->max.x, middle_y, middle_z);

	    n->childs[4] = new Octant(middle_x, n->min.y, middle_z, n->max.x, middle_y, n->max.z);
	    n->childs[5] = new Octant(middle_x, middle_y, middle_z, n->max.x, n->max.y, n->max.z);
	    n->childs[6] = new Octant(n->min.x, middle_y, middle_z, middle_x, n->max.y, n->max.z);
	    n->childs[7] = new Octant(n->min.x, n->min.y, middle_z, middle_x, middle_y, n->max.z);
  	}
};

#endif
