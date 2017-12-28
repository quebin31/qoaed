#include <iostream>
#include "PointQuadtree.hpp"

int main() {
  qoaed::PointQuadtree<std::string> pqt;

  pqt.insert(3, 2, "P: 3");
  pqt.insert(6, 7, "P: 2");
  pqt.insert(4, 5, "P: 9");
  pqt.insert(12, 2, "P: x");
  pqt.insert(11, 9, "P: y");

  auto result = pqt.ranged_query(qoaed::PointQuadtree<std::string>::Rect(1, 1, 10, 10));

  std::cout << "Visit BFS on pqt:\n";
  pqt.visit_bfs([](auto& n){ std::cout << "x: " << n.x << ", y: " << n.y << '\n'; });

  std::cout << "Visit BFS on result:\n";
  result.visit_bfs([](auto& n){ std::cout << "x: " << n.x << ", y: " << n.y << '\n'; });

  return 0;
}
