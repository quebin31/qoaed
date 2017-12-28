#include <iostream>
#include "PointQuadtree.hpp"

int main() {
  qoaed::PointQuadtree<std::string> pqt;

  pqt.insert(3, 2, "P: 3");
  pqt.insert(6, 7, "P: 2");
  pqt.insert(4, 5, "P: 9");
  pqt.insert(12, 2, "P: x");
  pqt.insert(11, 9, "P: y");

  std::cout << "Visit BFS on pqt:\n";
  pqt.visit_bfs([](auto& n){ std::cout << "x: " << n.get_x() << ", y: " << n.get_y() << ", val: " << *n << '\n'; });

  auto result = pqt.ranged_query(qoaed::PointQuadtree<std::string>::Rect(1, 1, 10, 10));
  auto result2 = pqt.ranged_query(qoaed::PointQuadtree<std::string>::Rect(6,6,10,10), [](auto& n){ *n = "Visited"; });

  std::cout << "Visit BFS on pqt:\n";
  pqt.visit_bfs([](auto& n){ std::cout << "x: " << n.get_x() << ", y: " << n.get_y() << ", val: " << *n << '\n'; });

  std::cout << "Visit BFS on result:\n";
  result.visit_bfs([](auto& n){ std::cout << "x: " << n.get_x() << ", y: " << n.get_y() << ", val: " << *n << '\n'; });

  auto nvis = pqt.find(4,5);
  std::cout << nvis.get_x() << ' ' << nvis.get_y() << ' ' << *nvis << std::endl;

  return 0;
}
