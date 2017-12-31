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

enum flags {
  EXPECTING_PROPERTIES = 0x0001,
  EXPECTING_VERTEXES   = 0x0002,
  READING_FINISHED     = 0x0004
};

void set_flag(int& c, flags flg) {
  c &= 0;
  c |= flg;
}

// Read an off file and return the points
std::vector<Point3D<double>> read_off(const std::string& filename) {
  std::vector<Point3D<double>> points;

  std::ifstream file(filename);
  if (!file) 
    throw std::runtime_error("Failed to open file " + filename);

  std::string line;
  std::getline(file, line);

  int curr_flag;

  if (line != "OFF") 
    throw std::runtime_error("File " + filename + " is not a OFF file type");
  else 
    set_flag(curr_flag, EXPECTING_PROPERTIES);


  std::regex  line_point_regex("-?[[:digit:]]+\\.[[:digit:]]+");
  std::smatch matchs;

  double coord[3];
  int no_coord = 0;
  int no_line  = 1;

  while (file) {
    std::getline(file, line);
    no_line += 1;

    //std::cout << line << ", " ;
    //std::cout << "flag: " << curr_flag << std::endl;
    // Skip comments
    if (line[0] == '#' || line.empty())
      continue;

    if (std::regex_search(line, line_point_regex)) {
      if (curr_flag & EXPECTING_PROPERTIES) {
        throw std::runtime_error("Missing properties in OFF file: " + filename + 
            " error in line " + std::to_string(no_line));
      } else if (curr_flag & EXPECTING_VERTEXES) {
        int no_numbers = 0;
        while (std::regex_search(line, matchs, line_point_regex)) {
          coord[no_numbers] = std::stod(matchs.str(0));
          no_numbers += 1;
          line = matchs.suffix();
        }

        if (no_numbers < 3) 
          throw std::runtime_error("Missing coordenates in OFF file: " + filename +
              " error in line " + std::to_string(no_line));

        points.emplace_back(coord[0], coord[1], coord[2]);

        no_coord -= 1;
        //std::cout << "expecting: " << no_coord << std::endl;
        if (no_coord == 0)
          set_flag(curr_flag, READING_FINISHED);
      } else {
        throw std::runtime_error("Unexpected symbols in line " + std::to_string(no_line));
      }

    } else {

      if (!(curr_flag & EXPECTING_PROPERTIES || curr_flag & READING_FINISHED)) {
        throw std::runtime_error("Unexpected symbols in line " + std::to_string(no_line));
      } else if (!(curr_flag & READING_FINISHED)){
        if (!std::regex_search(line, matchs, std::regex("(\\d+|0)"))) 
          throw std::runtime_error("Unexpected symbols in line " + std::to_string(no_line));
        no_coord   = std::stoi(matchs.str(0));
        set_flag(curr_flag, EXPECTING_VERTEXES);
        points.reserve(no_coord);
      } else { break; }
    }
  }

  return points;
}

  
}

}

#endif
