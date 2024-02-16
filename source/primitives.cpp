#include "primitives.hpp"

#include <float.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <vector>

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

bool IsPointInPole(const Coord& point, int size_x, int size_y) {
  return point.x >= 0 && point.y >= 0 && point.x < size_x && point.y < size_y;
}

enum ConnectPattern { Vertical, Horizontal, DiagonalUp, DiagonalDown };

Coord GetPatternNeighbour(const Coord& point, ConnectPattern pattern) {
  switch (pattern) {
    case Vertical:
      return {point.x, point.y + 1};
    case Horizontal:
      return {point.x + 1, point.y};
    case DiagonalUp:
      return {point.x + 1, point.y + 1};
    case DiagonalDown:
      return {point.x + 1, point.y - 1};
  }
}

struct PatBaseSegm {
  BaseSegment base_segment;
  bool connections[2] = {false, false};
};

std::vector<PatBaseSegm> GetBaseSegments(
    std::vector<std::vector<ExtractPoint>>& bitmap, ConnectPattern pattern) {
  for (auto& abscissa : bitmap) {
    for (auto& [black, segm_ind] : abscissa) {
      segm_ind = -1;
    }
  }

  std::vector<PatBaseSegm> base_segments;
  for (int x = 0; x < bitmap.size(); ++x) {
    for (int y = 0; y < bitmap[0].size(); ++y) {
      auto& [black, segm_ind] = bitmap[x][y];

      if (!black) {
        continue;
      }

      if (segm_ind == -1) {
        segm_ind = base_segments.size();
        base_segments.push_back({.base_segment = {{x, y}}});
      }

      auto neighbour = GetPatternNeighbour({x, y}, pattern);
      if (!(IsPointInPole(neighbour, bitmap.size(), bitmap[0].size()) &&
            bitmap[neighbour.x][neighbour.y].black)) {
        continue;
      }
      base_segments[segm_ind].base_segment.push_back(neighbour);
      bitmap[neighbour.x][neighbour.y].segm_ind = segm_ind;
    }
  }

  return base_segments;
}

ConnectPattern GetAddPattern(int identifier,
                             ConnectPattern base_pattern) noexcept {
  if (base_pattern == Vertical || base_pattern == Horizontal) {
    if (identifier == 0) {
      return DiagonalDown;
    }
    return DiagonalUp;
  }
  return identifier == 0 ? Horizontal : Vertical;
}

Coord GetNeighbour(const BaseSegment& segment, ConnectPattern add_pattern,
                   ConnectPattern base_pattern) {
  switch (base_pattern) {
    case Vertical:
      if (add_pattern == DiagonalUp) {
        return GetPatternNeighbour(segment.back(), add_pattern);
      }
      return GetPatternNeighbour(segment.front(), add_pattern);

    case Horizontal:
      return GetPatternNeighbour(segment.back(), add_pattern);

    case DiagonalUp:
      return GetPatternNeighbour(segment.back(), add_pattern);

    case DiagonalDown:
      if (add_pattern == Vertical) {
        return GetPatternNeighbour(segment.front(), add_pattern);
      }
      return GetPatternNeighbour(segment.back(), add_pattern);
  }
}

std::list<int> Unite(int base_segm_ind, ConnectPattern add_pattern,
                     ConnectPattern base_pattern,
                     const std::vector<PatBaseSegm>& bases,
                     const std::vector<std::vector<ExtractPoint>>& bitmap) {
  std::list<int> united;
  united.push_back(base_segm_ind);
  int base_size = bases[base_segm_ind].base_segment.size();
  if (base_size == 1) {
    return {};
  }
  int add_size = base_size;

  while (true) {
    int last_segm_ind = united.back();

    auto neigh_coord = GetNeighbour(bases[last_segm_ind].base_segment,
                                    add_pattern, base_pattern);
    if (!IsPointInPole(neigh_coord, bitmap.size(), bitmap[0].size())) {
      break;
    }
    const auto& neigh_point = bitmap[neigh_coord.x][neigh_coord.y];
    if (neigh_point.segm_ind < 0) {
      break;
    }

    const auto& base_neigh = bases[neigh_point.segm_ind];
    int neigh_size = base_neigh.base_segment.size();

    if (base_neigh.base_segment.front() != neigh_coord ||
        neigh_size > base_size) {
      break;
    }

    if (neigh_size == add_size) {
      if (add_size == 1) {
        break;
      }
      united.push_back(neigh_point.segm_ind);
      continue;
    }
    if (united.size() == 1) {
      add_size = neigh_size;
      united.push_back(neigh_point.segm_ind);
      continue;
    }
    if (base_size == add_size) {
      united.push_back(neigh_point.segm_ind);
      break;
    }
    break;
  }

  if (united.size() == 1) {
    return {};
  }
  return united;
}

bool IsSegmnetCorrect(const BaseSegment& segment) {
  return segment == Segment(segment.front(), segment.back()).GetGraphic();
}

std::list<BaseSegment> GetPatternedSegments(
    std::vector<std::vector<ExtractPoint>>& bitmap, ConnectPattern pattern) {
  std::list<BaseSegment> united;
  auto bases = GetBaseSegments(bitmap, pattern);
  for (const auto& base : bases) {
    united.push_back(base.base_segment);
  }

  for (int index = 0; index < bases.size(); ++index) {
    for (int i = 0; i <= 1; ++i) {
      if (bases[index].connections[i]) {
        continue;
      }
      auto add_pattern = GetAddPattern(i, pattern);
      auto united_segm = Unite(index, add_pattern, pattern, bases, bitmap);
      if (united_segm.empty()) {
        continue;
      }
      BaseSegment united_segm_points;
      for (int united_segm_part_ind : united_segm) {
        for (const auto& point : bases[united_segm_part_ind].base_segment) {
          united_segm_points.push_back(point);
        }
      }

      if (!IsSegmnetCorrect(united_segm_points)) {
        continue;
      }

      for (auto iter = united_segm.begin();
           iter != std::prev(united_segm.end()); ++iter) {
        bases[*iter].connections[i] = true;
      }
      united.push_back(std::move(united_segm_points));
    }
  }

  return united;
}

bool AreNeighbours(const Coord& first, const Coord& second) noexcept {
  return abs(first.x - second.x) <= 1 && abs(first.y - second.y) <= 1;
}

std::list<Segment> BaseExtractPrimitives(
    std::vector<std::vector<ExtractPoint>>& bitmap) {
  auto set_compare = [](const BaseSegment& first, const BaseSegment& second) {
    return GetDistance(first.back(), first.front()) >
           GetDistance(second.back(), second.front());
  };
  std::multiset<BaseSegment, decltype(set_compare)> base_segments(set_compare);

  for (int i = 0; i < 4; ++i) {
    for (auto&& segment :
         GetPatternedSegments(bitmap, static_cast<ConnectPattern>(i))) {
      base_segments.insert(std::move(segment));
    }
  }

  std::list<Segment> segments;

  for (auto iter = base_segments.begin(); iter != base_segments.end(); ++iter) {
    std::list<BaseSegment> proc_segm_list;
    for (const auto& coord : *iter) {
      if (!bitmap[coord.x][coord.y].black) {
        continue;
      }

      if (!proc_segm_list.empty() &&
          AreNeighbours(proc_segm_list.back().back(), coord)) {
        proc_segm_list.back().push_back(coord);
      } else {
        proc_segm_list.push_back({coord});
      }
    }

    if (proc_segm_list.empty()) {
      continue;
    }
    if (proc_segm_list.size() > 1 ||
        proc_segm_list.front().size() != iter->size()) {
      for (auto&& proc_segm : proc_segm_list) {
        base_segments.insert(std::move(proc_segm));
      }
      continue;
    }

    if (!IsSegmnetCorrect(proc_segm_list.back())) {
      continue;
    }

    for (const auto& coord : proc_segm_list.back()) {
      bitmap[coord.x][coord.y].black = false;
    }
    segments.push_back(
        {proc_segm_list.back().front(), proc_segm_list.back().back()});
  }

  return segments;
}

}  // namespace PTIT
