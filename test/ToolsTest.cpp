#include "Tools.hpp"
#include <iostream>

int main(int argc, char** argv) {
  if (argc < 2) return 1;

  auto points = qoaed::tools::read_off(argv[1]);

  for (auto& p : points) 
    std::cout << p.x << ' ' << p.y << ' ' << p.z << std::endl;

  return 0;
}
