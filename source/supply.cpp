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

double DegToRad(double deg) { return deg * kPi / (kDegInCircle / 2); }
double RadToDeg(double rad) { return rad * (kDegInCircle / 2) / kPi; }

double TanToDeg(double tan) { return RadToDeg(atan(tan)); }

}  // namespace PTIT