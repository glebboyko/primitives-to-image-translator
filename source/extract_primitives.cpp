#include <algorithm>
#include <cmath>
#include <list>
#include <optional>
#include <queue>
#include <stack>

#include "primitives.hpp"
#include "supply.hpp"

namespace PTIT {

const double kDegAccuracy = 0.001;
using BaseSegment = std::list<Coord>;

enum EDeviation { Negative, Neutral, Positive };
using Deviation = std::pair<EDeviation, EDeviation>;

std::list<Coord> GetNeighbours(Coord point, int x_size = -1, int y_size = -1) {
  bool has_borders = x_size != -1;

  std::list<Coord> neighbours;

  for (int x = point.x - 1; x <= point.x + 1; ++x) {
    for (int y = point.y - 1; y <= point.y + 1; ++y) {
      if (has_borders && (x < 0 || y < 0 || x >= x_size || y >= y_size)) {
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

double TanToDeg(double tan) { return RadToDeg(atan(tan)); }

class KRange {
 public:
  KRange(bool empty = false) : is_empty_(empty){};
  KRange(double min_angle, double max_angle)
      : min_angle_(min_angle), max_angle_(max_angle) {
    if (max_angle_ < min_angle_) {
      std::swap(min_angle_, max_angle_);
    }
  }

  KRange(const KRange&) = default;
  KRange(KRange&&) = default;
  KRange& operator=(const KRange&) = default;
  KRange& operator=(KRange&&) = default;

  ~KRange() = default;

  bool InRange(double k_coef) const {
    if (is_empty_) {
      return false;
    }
    auto deg = TanToDeg(k_coef);

    return AreIntersect(*this, KRange(deg - kDegAccuracy, deg + kDegAccuracy));
  }
  void Intersect(const KRange& k_range) {
    if (is_empty_ || k_range.is_empty_) {
      is_empty_ = true;
      return;
    }

    min_angle_ = std::max(min_angle_, k_range.min_angle_);
    max_angle_ = std::min(max_angle_, k_range.max_angle_);

    if (min_angle_ > max_angle_) {
      is_empty_ = true;
    }
  }

 private:
  bool is_empty_ = false;
  double min_angle_ = -kDegInCircle / 4;
  double max_angle_ = kDegInCircle / 4;

  static bool AreIntersect(KRange first, KRange second) {
    if (first.min_angle_ == second.min_angle_) {
      return true;
    }
    if (first.min_angle_ > second.min_angle_) {
      std::swap(first, second);
    }
    return first.max_angle_ >= second.min_angle_;
  }
};

enum Movement { XMove, YMove, None };
std::pair<Deviation, Movement> GetConnectionType(const Coord& first,
                                                 const Coord& second) noexcept {
  Deviation dev = {Neutral, Neutral};

  if (first.x < second.x) {
    dev.first = Positive;
  } else if (first.x > second.x) {
    dev.first = Negative;
  }

  if (first.y < second.y) {
    dev.second = Positive;
  } else if (first.y > second.y) {
    dev.second = Negative;
  }

  Movement move = None;
  if (dev.first == Neutral && dev.second != Neutral) {
    move = YMove;
  } else if (dev.first != Neutral && dev.second == Neutral) {
    move = XMove;
  }

  return {dev, move};
}

bool CanBeConnected(const Coord& first, const Coord& second,
                    const Deviation& dev, Movement rest_move) noexcept {
  auto [local_dev, local_move] = GetConnectionType(first, second);
  auto deviation_check = [](EDeviation f_dev, EDeviation s_dev) {
    return f_dev == Neutral || s_dev == Neutral || f_dev == s_dev;
  };

  if (!deviation_check(dev.first, local_dev.first) ||
      !deviation_check(dev.second, local_dev.second)) {
    return false;
  }
  return rest_move == None || rest_move != local_move;
}

void UpdateConnection(Deviation& deviation, Movement& restr_move,
                      const Coord& first, const Coord& second) noexcept {
  auto [local_dev, local_move] = GetConnectionType(first, second);
  if (local_dev.first != Neutral) {
    deviation.first = local_dev.first;
  }
  if (local_dev.second != Neutral) {
    deviation.second = local_dev.second;
  }

  if (local_move != None) {
    restr_move = local_move == XMove ? YMove : XMove;
  }
}

KRange GetKRange(const Segment& segment) {
  if (segment.GetA() == segment.GetB()) {
    return KRange();
  }
  double min_angle = TanToDeg(GetKCoefficient(segment));
  double max_angle = min_angle;

  for (const auto& coord : GetNeighbours(segment.GetB())) {
    double segm_angle = TanToDeg(GetKCoefficient({segment.GetA(), coord}));
    min_angle = std::min(min_angle, segm_angle);
    max_angle = std::max(max_angle, segm_angle);
  }
  return KRange(min_angle, max_angle);
}

struct RecurseRet {
  std::list<BaseSegment> cont;
  std::list<BaseSegment> other;
};
struct InputData {
  Coord curr_point;
  KRange k_range;
  Deviation deviation = {Neutral, Neutral};
  Movement restr_move = None;
  Coord init_point = {0, 0};

  RecurseRet* parent_ret;
};

struct GlobalVars {
  bool is_cont = false;
  bool process_ret = false;
};

struct StackData {
  InputData input;
  RecurseRet my_ret;
  GlobalVars global_vars;
};

std::list<BaseSegment> BaseSegmentsGetter(
    std::vector<std::vector<bool>>& bitmap, const Coord& in_curr_point) {
  RecurseRet recurse_ret;

  std::stack<StackData> stack;
  stack.push({.input = {.curr_point = in_curr_point,
                        .k_range = KRange(true),
                        .parent_ret = &recurse_ret}});

  while (!stack.empty()) {
    auto& input = stack.top().input;
    auto& ret_cont = stack.top().my_ret;
    auto& vars = stack.top().global_vars;

    if (!vars.process_ret) {
      auto init_k = GetKCoefficient({input.init_point, input.curr_point});
      if (input.k_range.InRange(init_k)) {
        input.k_range = GetKRange({input.init_point, input.curr_point});
        vars.is_cont = true;
      } else {
        input.init_point = input.curr_point;
        input.k_range = KRange();
        input.deviation = {Neutral, Neutral};
        input.restr_move = None;
      }

      bitmap[input.curr_point.x][input.curr_point.y] = false;

      for (auto neighbour :
           GetNeighbours(input.curr_point, bitmap.size(), bitmap[0].size())) {
        if (!bitmap[neighbour.x][neighbour.y] ||
            !CanBeConnected(input.curr_point, neighbour, input.deviation,
                            input.restr_move)) {
          continue;
        }

        StackData local_data;
        local_data.input = {.curr_point = neighbour,
                            .k_range = input.k_range,
                            .init_point = input.init_point,
                            .parent_ret = &ret_cont};
        local_data.input.deviation = input.deviation;
        local_data.input.restr_move = input.restr_move;
        UpdateConnection(local_data.input.deviation,
                         local_data.input.restr_move, input.curr_point,
                         neighbour);
        stack.push(local_data);
      }

      vars.process_ret = true;
    } else {
      double cont_len = -1;
      auto cont_iter = ret_cont.cont.begin();

      for (auto iter = ret_cont.cont.begin(); iter != ret_cont.cont.end();
           ++iter) {
        auto segm_len =
            iter->empty() ? -1 : GetDistance(iter->front(), iter->back());
        if (segm_len > cont_len) {
          cont_len = segm_len;
          cont_iter = iter;
        }
      }
      if (cont_len >= 0) {
        auto cont = std::move(*cont_iter);
        ret_cont.cont.erase(cont_iter);
        cont.push_front(input.curr_point);
        if (vars.is_cont) {
          input.parent_ret->cont.push_back(std::move(cont));
        } else {
          input.parent_ret->other.push_back(std::move(cont));
        }
      } else if (vars.is_cont) {
        input.parent_ret->cont.push_back({input.curr_point});
      } else {
        input.parent_ret->other.push_back({input.curr_point});
      }

      for (auto&& cont : ret_cont.cont) {
        input.parent_ret->other.push_back(std::move(cont));
      }
      for (auto&& cont : ret_cont.other) {
        input.parent_ret->other.push_back(std::move(cont));
      }
      stack.pop();
    }
  }

  return recurse_ret.other;
}

struct BSCont {
  BaseSegment base_segment;
  Deviation deviation;
  Movement rest_move;
};

struct SCont {
  Segment segment;
  Deviation deviation;
  Movement rest_move;
};

bool CanBeConnected(const SCont& first, const SCont& second) {
  if (!GetKRange({first.segment.GetA(), first.segment.GetB()})
           .InRange(GetKCoefficient(
               {first.segment.GetA(), second.segment.GetB()})) ||
      !GetKRange({second.segment.GetB(), second.segment.GetA()})
           .InRange(GetKCoefficient(
               {second.segment.GetB(), first.segment.GetA()}))) {
    return false;
  }

  // checking pattern
  auto f_dev = first.deviation;
  auto s_dev = second.deviation;

  if (f_dev.first != Neutral && s_dev.first != Neutral &&
          f_dev.first != s_dev.first ||
      f_dev.second != Neutral && s_dev.second != Neutral &&
          f_dev.second != s_dev.second) {
    return false;
  }

  // checking movement
  return first.rest_move == None || second.rest_move == None ||
         first.rest_move == second.rest_move;
}

std::pair<Deviation, Movement> UniteConnections(const Deviation& f_dev,
                                                Movement f_move,
                                                const Deviation& s_dev,
                                                Movement s_move) {
  Deviation u_dev = f_dev;
  Movement u_move = f_move;

  if (s_dev.first != Neutral) {
    u_dev.first = s_dev.first;
  }
  if (s_dev.second != Neutral) {
    u_dev.second = s_dev.second;
  }

  if (s_move != None) {
    u_move = s_move;
  }

  return {u_dev, u_move};
}

Deviation ReverseDeviation(Deviation deviation) {
  if (deviation.first != Neutral) {
    deviation.first = deviation.first == Positive ? Negative : Positive;
  }
  if (deviation.second != Neutral) {
    deviation.second = deviation.second == Positive ? Negative : Positive;
  }

  return deviation;
}

bool UniteNeighbours(
    SCont cont, std::list<SCont>& segments,
    std::vector<std::vector<std::optional<std::list<SCont>::iterator>>>&
        bitmap) {
  auto& [segm, dev, restr_move] = cont;
  Coord conn_point = segm.GetB();
  auto iter = bitmap[conn_point.x][conn_point.y].value();

  for (auto neighbour :
       GetNeighbours(conn_point, bitmap.size(), bitmap[0].size())) {
    if (!bitmap[neighbour.x][neighbour.y].has_value() ||
        !CanBeConnected(conn_point, neighbour, dev, restr_move)) {
      continue;
    }

    auto neighbour_iter = bitmap[neighbour.x][neighbour.y].value();
    if (neighbour_iter == iter) {
      continue;
    }

    auto n_cont = *neighbour_iter;
    auto& [n_segm, n_dev, n_restr_move] = n_cont;

    if (n_segm.GetB() == neighbour) {
      std::swap(n_segm.GetB(), n_segm.GetA());
      n_dev = ReverseDeviation(n_dev);
    }

    if (!CanBeConnected(cont, n_cont)) {
      continue;
    }

    auto [u_dev, u_restr_move] =
        UniteConnections(dev, restr_move, n_dev, n_restr_move);

    segm.GetB() = n_segm.GetB();
    dev = u_dev;
    restr_move = u_restr_move;

    bitmap[neighbour.x][neighbour.y].reset();
    bitmap[conn_point.x][conn_point.y].reset();
    bitmap[segm.GetA().x][segm.GetA().y] = iter;
    bitmap[segm.GetB().x][segm.GetB().y] = iter;

    segments.erase(neighbour_iter);

    *iter = cont;
    return true;
  }

  return false;
}

std::list<Segment> BaseExtractPrimitives(
    std::vector<std::vector<bool>>& bitmap) {
  Coord size = {static_cast<int>(bitmap.size()),
                static_cast<int>(bitmap[0].size())};

  std::list<BSCont> raw_segments;

  // getting extracted raw segments
  for (int x = 0; x < size.x; ++x) {
    for (int y = 0; y < size.y; ++y) {
      if (!bitmap[x][y]) {
        continue;
      }

      auto base_segments = BaseSegmentsGetter(bitmap, {x, y});

      for (auto&& base : base_segments) {
        raw_segments.push_back({std::move(base), {Neutral, Neutral}, None});
      }
    }
  }

  // calculating deviation and restricted moves
  for (auto& [base, dev, restr_move] : raw_segments) {
    for (auto iter = std::next(base.begin()); iter != base.end(); ++iter) {
      if (dev.first != Neutral && dev.second != Neutral && restr_move != None) {
        break;
      }
      UpdateConnection(dev, restr_move, *std::prev(iter), *iter);
    }
  }

  std::list<SCont> processed_raws;

  using ConnBitmapType = std::optional<decltype(processed_raws)::iterator>;
  std::vector<std::vector<ConnBitmapType>> conn_bitmap(
      size.x, std::vector<ConnBitmapType>(size.y));

  for (const auto& [base, dev, restr_move] : raw_segments) {
    processed_raws.push_back(
        {Segment(base.front(), base.back()), dev, restr_move});
    auto a_point = processed_raws.back().segment.GetA();
    auto b_point = processed_raws.back().segment.GetB();
    conn_bitmap[a_point.x][a_point.y] = std::prev(processed_raws.end());
    conn_bitmap[b_point.x][b_point.y] = std::prev(processed_raws.end());
  }

  // connecting
  for (auto iter = processed_raws.begin(); iter != processed_raws.end();) {
    bool united = UniteNeighbours(*iter, processed_raws, conn_bitmap);

    auto cont = *iter;
    std::swap(cont.segment.GetA(), cont.segment.GetB());
    cont.deviation = ReverseDeviation(cont.deviation);

    united = united || UniteNeighbours(cont, processed_raws, conn_bitmap);

    if (!united) {
      ++iter;
    }
  }

  std::list<Segment> segments;
  for (auto segm : processed_raws) {
    segments.push_back(segm.segment);
  }

  return segments;
}

}  // namespace PTIT