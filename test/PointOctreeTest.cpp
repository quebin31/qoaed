#include "PointOctree.hpp"
#include "Tools.hpp"
#include <iostream>

int main(int argc, char** argv) {

  qoaed::PointOctree<int> po;
  po.insert(3, 2, 4, 1);
  po.insert(3, -2, 5, 2);
  po.insert(-1, -2, -5, 3);
  po.insert(12, 1, 2, 4);
  po.insert(3, 2, 1, 5);
  po.insert(10, 1, 3, 6);
  po.insert(6, 6, 6, 7);
  po.insert(1, 1, 1, 8);
  po.insert(1, 2, 3, 9);
  po.insert(1, 4, 3, 9);
  po.insert(1, 5, 3, 9);
  po.insert(1, 1, 3, 9);
  po.insert(1, 6, 3, 9);
  po.insert(1, 2, 2, 9);
  po.insert(1, 8, 3, 9);
  po.insert(1, 2, 4, 9);
  po.insert(1, 2, 5, 9);
  po.insert(1, 2, 7, 9);
  po.insert(1, 2, 8, 9);

  std::cout << "PointOctree on BFS: \n";
  po.visit_bfs([](auto& n){ std::cout << n.get_x() << ", " << n.get_y() << ", " << *n << std::endl; });

  std::cout << "Subtree ranged query (1,1,1) (10,10,10): \n";
  auto rs = po.ranged_query(qoaed::PointOctree<int>::Cube(1,1,1,10,10,10));
  rs.visit_bfs([](auto& n){ std::cout << n.get_x() << ", " << n.get_y() << ", " << *n << std::endl; });

  auto points = qoaed::tools::read_off(argv[1]);

  qoaed::PointOctree<char, double>* oc = new qoaed::PointOctree<char, double>;
  for (int ii = 0; ii < points.size(); ++ii) {
    oc->insert(points[ii].x, points[ii].y, points[ii].z, 'a');
  }

  oc->visit_bfs([](auto& n){ std::cout << n.get_x() << ' ' << n.get_y() << ' ' << n.get_z() << ' ' << *n << '\n'; });


  return 0;
}
