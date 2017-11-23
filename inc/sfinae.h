#ifndef QOAED_SFINAE_H
#define QOAED_SFINAE_H

#include <utility>

namespace qoaed {

template <typename UnnamedType>
class container {
private:
  // We use std::declval to 'recreate' an object of 'UnnamedType'.
  // We use std::declval to also 'recreate' an object of type 'Param'.
  // We can use both of these recreated objects to test the validity!
  template <typename Param>
  constexpr auto testValidity(int) -> // int for precedence
  decltype(std::declval<UnnamedType>()(std::declval<Param>()), std::true_type()) {
    // If the substitution didn't fail return true_type
    return std::true_type();
  }

  template <typename Param>
  constexpr std::false_type testValidity(...) {
    // Sink hole returns false type
    return std::false_type();
  }

public:
  template <typename Param>
  constexpr auto operator()(const Param& p) {
    // The argument is forwarded to one of the two overloads.
    // The SFINAE on the 'true_type' will come into play to dispatch.
    // Once again, we use the int for the precedence.
    return testValidity<Param>(int());
  }
};

template <typename UnnamedType>
constexpr auto is_valid(const UnnamedType& t) {
  return container<UnnamedType>();
}

}

#endif
