#include "primitives.hpp"

#include <float.h>

#include <cmath>
#include <list>
#include <optional>

#include "supply.hpp"

namespace PTIT {

Coord Coord::operator*(float coef) const {
  Coord ret = *this;
  ret.x *= coef;
  ret.y *= coef;
  return ret;
}

bool operator==(const Coord& first, const Coord& second) noexcept {
  return first.x == second.x && first.y == second.y;
}
bool operator!=(const Coord& first, const Coord& second) noexcept {
  return !(first == second);
}
bool operator<(const Coord& first, const Coord& second) noexcept {
  if (first.x != second.x) {
    return first.x < second.x;
  }
  return first.y < second.y;
}
bool operator<=(const Coord& first, const Coord& second) noexcept {
  return first == second || first < second;
}
bool operator>(const Coord& first, const Coord& second) noexcept {
  return !(first <= second);
}
bool operator>=(const Coord& first, const Coord& second) noexcept {
  return !(first < second);
}

double GetDistance(const Coord& first, const Coord& second) noexcept {
  double dx = second.x - first.x;
  double dy = second.y - first.y;

  return sqrt((dx * dx) + (dy * dy));
}

Segment::Segment(const Coord& a_point, const Coord& b_point)
    : a_point_(a_point), b_point_(b_point) {}

Coord& Segment::GetA() { return a_point_; }
Coord& Segment::GetB() { return b_point_; }

const Coord& Segment::GetA() const { return a_point_; }
const Coord& Segment::GetB() const { return b_point_; }

std::list<Coord> Segment::GetGraphic() const {
  Segment normalized(a_point_, b_point_);
  if (a_point_ > b_point_) {
    std::swap(normalized.a_point_, normalized.b_point_);
  }

  float k_coefficient = GetKCoefficient(*this);
  int b_coefficient = normalized.GetBCoefficient();

  std::list<Coord> graphic;

  if (k_coefficient == FLT_MAX) {
    for (int y = normalized.a_point_.y; y <= normalized.b_point_.y; ++y) {
      graphic.push_back({normalized.b_point_.x, y});
    }
  } else if (k_coefficient <= 1 && k_coefficient >= -1) {
    for (int x = normalized.a_point_.x; x <= normalized.b_point_.x; ++x) {
      graphic.push_back(
          {x, static_cast<int>(k_coefficient * x + b_coefficient)});
    }
  } else {
    int inc = k_coefficient >= 0 ? 1 : -1;
    for (int y = normalized.a_point_.y;
         inc > 0 ? y <= normalized.b_point_.y : y >= normalized.b_point_.y;
         y += inc) {
      graphic.push_back(
          {static_cast<int>(static_cast<float>(y) / k_coefficient -
                            static_cast<float>(b_coefficient) / k_coefficient),
           y});
    }
  }
  return graphic;
}

int Segment::GetBCoefficient() const {
  if (b_point_.x - a_point_.x == 0) {
    return 0;
  }
  return ((static_cast<int64_t>(b_point_.x) * a_point_.y) -
          (static_cast<int64_t>(a_point_.x) * b_point_.y)) /
         (b_point_.x - a_point_.x);
}

/*----------------------------- extract segments -----------------------------*/

}  // namespace PTIT
