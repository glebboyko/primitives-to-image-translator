#include "primitives.hpp"

#include <float.h>

#include <cmath>
#include <list>
#include <optional>

#include "supply.hpp"

namespace PTIT {

/*-------------------------------- coordinate --------------------------------*/
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

/*--------------------------------- segment ----------------------------------*/
Segment::Segment(const Coord& a_point, const Coord& b_point)
    : a_point_(a_point), b_point_(b_point) {}

Coord& Segment::GetA() { return a_point_; }
Coord& Segment::GetB() { return b_point_; }

const Coord& Segment::GetA() const { return a_point_; }
const Coord& Segment::GetB() const { return b_point_; }

double Segment::GetAngle() const {
  auto k_coef = GetKCoefficient(*this);
  double deg =
      k_coef == FLT_MAX ? static_cast<double>(kDegInCircle) / 4 : atan(k_coef);
  return a_point_ <= b_point_ ? deg
                              : (static_cast<double>(kDegInCircle) / 2) + deg;
}

void Segment::SetLen(int len) {
  auto center = GetCenter();
  if (a_point_.x == b_point_.x) {
    a_point_.y = center.y - len / 2;
    b_point_.y = center.y + (len / 2) + (len % 2);
    return;
  }

  double a_len_ratio = len / GetDistance(center, a_point_);
  double b_len_ratio = len / GetDistance(center, b_point_);

  a_point_ = {
      static_cast<int>(a_len_ratio * (a_point_.x - center.x) + center.x),
      static_cast<int>(a_len_ratio * (a_point_.y - center.y) + center.y)};

  b_point_ = {
      static_cast<int>(b_len_ratio * (b_point_.x - center.x) + center.x),
      static_cast<int>(b_len_ratio * (b_point_.y - center.y) + center.y)};
}

void Segment::SetAngle(double deg) { SetKCoef(tan(DegToRad(deg))); }

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

Coord Segment::GetCenter() const {
  return {(a_point_.x + b_point_.x) / 2, (a_point_.y + b_point_.y) / 2};
}

void Segment::SetKCoef(double new_k) {
  auto center = GetCenter();
  double c_len = GetDistance(a_point_, b_point_) / 2;

  double sin_n;
  double cos_n;

  if (new_k == FLT_MAX) {
    sin_n = 1;
    cos_n = 0;
  } else {
    double denominator = sqrt(1 + (new_k * new_k));

    sin_n = new_k / denominator;
    cos_n = 1 / denominator;
  }

  int delta_x = c_len * cos_n;
  int delta_y = c_len * sin_n;

  a_point_ = {center.x - delta_x, center.y - delta_y};
  b_point_ = {center.x + delta_x, center.y + delta_y};
}

std::list<Coord> Segment::GetArea(int radius) const {
  std::list<Coord> area;

  area.splice(area.cend(), Circe(a_point_, radius).GetGraphic());
  area.splice(area.cend(), Circe(b_point_, radius).GetGraphic());

  auto init_k = GetKCoefficient(*this);
  float norm_k;
  if (init_k == 0) {
    norm_k = FLT_MAX;
  } else if (init_k == FLT_MAX) {
    norm_k = 0;
  } else {
    norm_k = -(1 / init_k);
  }

  Segment from_a({a_point_, a_point_});
  from_a.SetLen(radius * 2);
  from_a.SetKCoef(norm_k);

  Segment from_b({b_point_, b_point_});
  from_b.SetLen(radius * 2);
  from_b.SetKCoef(norm_k);

  Segment upper_bound(from_a.b_point_, from_b.b_point_);
  Segment lower_bound(from_a.a_point_, from_b.a_point_);

  area.splice(area.cend(), upper_bound.GetGraphic());
  area.splice(area.cend(), lower_bound.GetGraphic());

  return FulfillArea(area);
}

/*--------------------------------- triangle ---------------------------------*/
Triangle::Triangle(const Coord& a_point, const Coord& b_point,
                   const Coord& c_point)
    : a_point_(a_point), b_point_(b_point), c_point_(c_point) {}

std::tuple<Coord, Coord, Coord> Triangle::GetPoints() const {
  return {a_point_, b_point_, c_point_};
}

std::list<Coord> Triangle::GetGraphic() const {
  std::list<Coord> graphic;

  graphic.splice(graphic.cend(), Segment(a_point_, b_point_).GetGraphic());
  graphic.splice(graphic.cend(), Segment(b_point_, c_point_).GetGraphic());
  graphic.splice(graphic.cend(), Segment(c_point_, a_point_).GetGraphic());

  return graphic;
}

/*---------------------------------- circle ----------------------------------*/
Circe::Circe(const PTIT::Coord& center, double radius)
    : center_(center), radius_(radius) {}

Coord& Circe::GetCenter() { return center_; }
const Coord& Circe::GetCenter() const { return center_; }
int& Circe::GetRadius() { return radius_; }
int Circe::GetRadius() const { return radius_; }

std::list<Coord> Circe::GetGraphic() const {
  // second quarter
  std::list<Coord> quarter_graphic;
  int64_t sqr_radius = radius_ * radius_;
  quarter_graphic.push_front({-radius_, 0});
  for (int x = -radius_ + 1; x <= 0; ++x) {
    int64_t sqr_x = x * x;
    int y = sqrt(sqr_radius - sqr_x);

    while (quarter_graphic.front().y < y - 1) {
      quarter_graphic.push_front({x, quarter_graphic.front().y + 1});
    }
    quarter_graphic.push_front({x, y});
  }

  // first quarter
  std::list<Coord> graphic;
  for (auto iter = std::next(quarter_graphic.begin());
       iter != quarter_graphic.end(); ++iter) {
    graphic.push_front({-iter->x, iter->y});
  }

  graphic.splice(graphic.cend(), std::move(quarter_graphic));

  // third and fourth quarter
  for (auto iter = std::next(graphic.rbegin());
       iter != std::prev(graphic.rend()); ++iter) {
    graphic.push_back({iter->x, -iter->y});
  }

  // moving to center
  std::for_each(graphic.begin(), graphic.end(), [this](Coord& coord) {
    coord = {coord.x + center_.x, coord.y + center_.y};
  });

  return graphic;
}

/*------------------------------ free functions ------------------------------*/
double GetDistance(const Coord& first, const Coord& second) noexcept {
  double dx = second.x - first.x;
  double dy = second.y - first.y;

  return sqrt((dx * dx) + (dy * dy));
}

std::list<Coord> FulfillArea(const std::list<Coord>& border) {
  std::vector<Coord> sorted_border;
  sorted_border.reserve(border.size());
  sorted_border.assign(border.begin(), border.end());

  std::sort(sorted_border.begin(), sorted_border.end(),
            [](const Coord& first, const Coord& second) {
              return first.y == second.y ? first.x < second.x
                                         : first.y < second.y;
            });

  std::list<Coord> area;
  area.assign(sorted_border.begin(), sorted_border.end());

  for (auto iter = std::next(area.begin()); iter != area.end(); ++iter) {
    if (std::prev(iter)->y != iter->y) {
      continue;
    }
    while (std::prev(iter)->x < iter->x - 1) {
      area.insert(iter, {std::prev(iter)->x + 1, iter->y});
    }
  }

  return area;
}

}  // namespace PTIT
