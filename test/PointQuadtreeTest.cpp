#include <iostream>
#include "PointQuadtree.hpp"

int main() {
  qoaed::PointQuadtree<std::string> pqt;

  pqt.insert(3, 2, "P: 3");
  pqt.insert(6, 7, "P: 2");
  pqt.insert(4, 5, "P: 9");
  pqt.insert(34, 2, "P: x");
  pqt.insert(14, 9, "P: y");

  auto result = pqt.ranged_query(qoaed::PointQuadtree<std::string>::Rect(1, 1, 10, 10));

  for (auto& x : result)
    std::cout << *x << ' ';
  std::cout << std::endl;


  return 0;
}
