#include "PointOctree.hpp"
#include <iostream>

int main() {
  qoaed::PointOctree<std::string> po;
  po.insert(3,2,4,"hola");
  po.insert(3,-2,5,"mundo");
  po.insert(-1,-2,-5,"jaja");

  po.visit_bfs([](auto& n){ std::cout << n.val << std::endl; });

  return 0;
}
