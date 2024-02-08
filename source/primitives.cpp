#include "primitives.hpp"

#include <float.h>

#include <cmath>
#include <cstdint>
#include <queue>

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

float GetKCoefficient(const Segment& segment) {
  if (segment.GetB().x == segment.GetA().x) {
    return FLT_MAX;
  }
  return (static_cast<float>(segment.GetB().y - segment.GetA().y)) /
         (segment.GetB().x - segment.GetA().x);
}

std::list<Coord> Segment::GetGraphic() const {
  Segment normalized(a_point_, b_point_);
  if (normalized.a_point_.x == normalized.b_point_.x &&
          normalized.a_point_.y > normalized.b_point_.y ||
      normalized.a_point_.x > normalized.b_point_.x) {
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

std::list<Coord> GetNeighbours(Coord point, int x_size, int y_size) {
  std::list<Coord> neighbours;
  for (int x = point.x - 1; x <= point.x + 1; ++x) {
    for (int y = point.y - 1; y <= point.y + 1; ++y) {
      if (x < 0 || y < 0 || x >= x_size || y >= y_size) {
        continue;
      }
      if (x == point.x && y == point.y) {
        continue;
      }
      neighbours.push_back({x, y});
    }
  }
  return neighbours;
}

const Coord& GetLeft(const Coord& a_point, const Coord& b_point) noexcept {
  if (a_point.x == b_point.x) {
    return a_point.y <= b_point.y ? a_point : b_point;
  }
  return a_point.x < b_point.x ? a_point : b_point;
}
const Coord& GetRight(const Coord& a_point, const Coord& b_point) noexcept {
  return &GetLeft(a_point, b_point) == &a_point ? b_point : a_point;
}

bool IsKBelongToRange(float k_coef, std::pair<float, float> k_range) noexcept {
  if (k_coef == k_range.second && k_coef == FLT_MAX) {
    return true;
  }
  return k_coef > k_range.first && k_coef < k_range.second;
}

std::list<Segment> GetNeighbourSegments(const Segment& segment) {
  std::list<Segment> neighbours;

  // a-left
  neighbours.push_back({{segment.GetA().x - 1, segment.GetA().y},
                        {segment.GetB().x + 1, segment.GetB().y}});
  // a-right
  neighbours.push_back({{segment.GetA().x + 1, segment.GetA().y},
                        {segment.GetB().x - 1, segment.GetB().y}});
  // a-up
  neighbours.push_back({{segment.GetA().x, segment.GetA().y + 1},
                        {segment.GetB().x, segment.GetB().y - 1}});
  // a-down
  neighbours.push_back({{segment.GetA().x, segment.GetA().y - 1},
                        {segment.GetB().x, segment.GetB().y + 1}});

  return neighbours;
}
std::pair<float, float> GetKRange(const Segment& segment) {
  if (segment.GetA() == segment.GetB()) {
    return {-FLT_MAX, FLT_MAX};
  }

  float k_min = GetKCoefficient(segment);
  float k_max = k_min;
  for (const auto& neighbour : GetNeighbourSegments(segment)) {
    float neighbour_k = GetKCoefficient(neighbour);
    k_min = std::min(k_min, neighbour_k);
    k_max = std::max(k_max, neighbour_k);
  }

  return {k_min, k_max};
}

std::list<Segment> BaseExtractPrimitives(
    std::vector<std::vector<std::pair<bool, int64_t>>>& bitmap) {
  if (bitmap.empty() || bitmap[0].empty()) {
    return {};
  }
  std::list<Segment> primitives;

  for (int x = 0; x < bitmap.size(); ++x) {
    for (int y = 0; y < bitmap[0].size(); ++y) {
      int64_t index = y * bitmap.size() + x;
      if (!bitmap[x][y].first) {
        continue;
      }

      Segment segment({x, y}, {x, y});
      std::queue<Coord> neighbours;
      std::pair<float, float> k_range = {-FLT_MAX, FLT_MAX};

      neighbours.push(segment.GetA());

      while (!neighbours.empty()) {
        Coord point = neighbours.front();
        neighbours.pop();

        if (!bitmap[point.x][point.y].first ||
            bitmap[point.x][point.y].second == index) {
          continue;
        }

        const Coord& base_point = GetDistance(point, segment.GetA()) >
                                          GetDistance(point, segment.GetB())
                                      ? segment.GetA()
                                      : segment.GetB();

        Segment new_segm(GetLeft(point, base_point),
                         GetRight(point, base_point));

        float segment_k = GetKCoefficient({new_segm.GetA(), new_segm.GetB()});
        if (IsKBelongToRange(segment_k, k_range)) {
          bitmap[point.x][point.y].second = index;
          for (const Coord& neighbour :
               GetNeighbours(point, bitmap.size(), bitmap[0].size())) {
            neighbours.push(neighbour);
          }
          segment = new_segm;
          k_range = GetKRange(segment);
        }
      }

      for (const auto& point : segment.GetGraphic()) {
        bitmap[point.x][point.y].first = false;
      }
      primitives.push_back(segment);
    }
  }
  return primitives;
}

}  // namespace PTIT
