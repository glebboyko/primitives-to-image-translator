#pragma once

#include "primitives.hpp"

namespace PTIT {

const double kPi = 3.1415926535;
const int kDegInCircle = 360;

float GetKCoefficient(const Segment& segment);

double DegToRad(double deg);
double RadToDeg(double rad);

}  // namespace PTIT