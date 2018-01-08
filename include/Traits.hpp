#ifndef QOAED_TRAITS_HPP
#define QOAED_TRAITS_HPP 

#include <type_traits>

namespace qoaed {

namespace tools {

template <class T>
struct is_valid_coord_type : std::integral_constant<bool,
  std::is_same<int, T>::value     || 
  std::is_same<long, T>::value    || 
  std::is_same<short, T>::value   || 
  std::is_same<float, T>::value   || 
  std::is_same<double, T>::value  > {};

}

}

#endif
