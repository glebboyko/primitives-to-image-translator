#include "supply.hpp"

#include <float.h>

namespace PTIT {

float GetKCoefficient(const Segment& segment) {
  if (segment.GetB().x == segment.GetA().x) {
    return FLT_MAX;
  }
  return (static_cast<float>(segment.GetB().y - segment.GetA().y)) /
         (segment.GetB().x - segment.GetA().x);
}

}  // namespace PTIT