#ifndef QOAED_EXCEPTIONS_H
#define QOAED_EXCEPTIONS_H

#include <exception>

namespace qoaed {

class DimensionsMissing : public std::exception {
public:
  const char* what() noexcept {
    return "Max dimesions for Quadtree are missing!";
  }
};

}

#endif
