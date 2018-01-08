#ifndef QOAED_TOOLS_HPP
#define QOAED_TOOLS_HPP 

#include <regex>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <utility>

#include "Point.hpp"

namespace qoaed {

namespace tools {

namespace constants {
  const double sqrt_of_2 = 1.414213562373;
}

enum flags {
  EXPECTING_PROPERTIES = 0x0001,
  EXPECTING_VERTICES   = 0x0002,
  READING_FINISHED     = 0x0004
};

void set_flag(int& c, flags flg) {
  c &= 0;
  c |= flg;
}

template <class T>
std::vector<Point3D<T>> read_off(const std::string& filename) {
  static_assert(
    tools::is_valid_coord_type<T>::value,
    "You passed an invalid type in read_off, valids are (inherited from Point3D)"
    "{ short, unsigned short, int, unsigned int, long, unsigned long, float, double }"
  );
  std::vector<Point3D<T>> points;

  std::ifstream file(filename);
  if (!file) 
    throw std::runtime_error("Failed to open file " + filename);

  std::string line;
  std::getline(file, line);

  int curr_flag = 0;

  //std::cout << "first line: " << line << std::endl;
  if (!std::regex_search(line, std::regex("OFF")))
    throw std::runtime_error("File " + filename + " is not a OFF file type");
  else 
    set_flag(curr_flag, EXPECTING_PROPERTIES);

  std::regex  numbers_regex("[+-]?([0-9]+\\.[0-9]+|[0-9]+)");
  std::smatch matchs;

  T   coord[3];
  int no_coord = 0;
  int no_line  = 1;

  while (file) {
    std::getline(file, line);
    no_line += 1;

    //std::cout << line << ", " ;
    //std::cout << "flag: " << curr_flag << std::endl;
    //std::cout << "expecting: " << no_coord << std::endl;
    // Skip comments
    if (std::regex_match(line, std::regex("^(#.*)$")) || line.empty())
      continue;

    if (!std::regex_search(line, numbers_regex)) {
      if (curr_flag & EXPECTING_PROPERTIES) 
        throw std::runtime_error("Expecting properties line, get unexpected line, error in line " + std::to_string(no_line));
      if (curr_flag & EXPECTING_VERTICES)
        throw std::runtime_error("Expecting vertices line, get unexpected line, error in line " + std::to_string(no_line));
      
      // TODO: Starting here are more cases to handle, but for now just interested in vertices
      if (curr_flag & READING_FINISHED)
        break;
    }

    if (curr_flag & EXPECTING_PROPERTIES) {
      if (!std::regex_search(line, matchs, std::regex("[0-9]+")))
        throw std::runtime_error(
          "Expecting properties line, get unexpected line, error in line " + 
          std::to_string(no_line) + 
          " hint: properties are not floating numbers, they are integers"
        );
      no_coord = std::stoi(matchs.str(0));
      set_flag(curr_flag, EXPECTING_VERTICES);
      points.reserve(no_coord);
      continue;
    }

    if (curr_flag & EXPECTING_VERTICES) {
      int no_numbers_in_curr_line = 0;
      while (std::regex_search(line, matchs, numbers_regex)) {
        if (no_numbers_in_curr_line >= 3) 
          throw std::runtime_error(
            "This line is not vertices line, error in line " + std::to_string(no_line) +
            " expected vertices line"
          );

        coord[no_numbers_in_curr_line] = static_cast<T>(std::stod(matchs.str(0)));
        no_numbers_in_curr_line += 1;
        line = matchs.suffix();
      }

      if (no_numbers_in_curr_line < 3)
        throw std::runtime_error(
          "Missing vertices in line " + std::to_string(no_line) + " got " + 
          std::to_string(no_numbers_in_curr_line) + " vertices"
        );

      points.emplace_back(coord[0], coord[1], coord[2]);
      no_coord -= 1;
      if (no_coord == 0)
        set_flag(curr_flag, READING_FINISHED);
    }
  }

  file.close();
  return points;
}

  
}

}

#endif
