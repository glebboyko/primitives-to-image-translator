#pragma once

#include <list>
#include <optional>

#include "concepts.hpp"

namespace PTIT {

struct Coord {
  int x;
  int y;

  Coord operator*(float coef) const;
};

bool operator==(const Coord&, const Coord&) noexcept;

double GetDistance(const Coord& first, const Coord& second) noexcept;

class Primitive {
 public:
  virtual std::list<Coord> GetGraphic() const = 0;
};

class Segment : public Primitive {
 public:
  Segment() = default;
  Segment(const Coord& a_point, const Coord& b_point);

  Coord& GetA();
  Coord& GetB();
  const Coord& GetA() const;
  const Coord& GetB() const;

  std::list<Coord> GetGraphic() const override;

 private:
  Coord a_point_;
  Coord b_point_;

  int GetBCoefficient() const;
};

using BaseSegment = std::list<Coord>;

struct ExtractPoint {
  bool black = false;
  int segm_ind = -1;
};

std::list<Segment> BaseExtractPrimitives(
    std::vector<std::vector<ExtractPoint>>& bitmap);

template <typename Container, typename Translator>
  requires ImageBitmap<Container> &&
           AvailabilityTranslator<Container, Translator>
std::list<Segment> ExtractPrimitives(const Container& container, int size_x,
                                     int size_y, Translator translator) {
  if (size_x == 0 || size_y == 0) {
    return {};
  }
  std::vector<std::vector<ExtractPoint>> converted_bitmap(
      size_x, std::vector<ExtractPoint>(size_y));
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      converted_bitmap[x][y] = {.black = translator(container[x][y])};
    }
  }

  return BaseExtractPrimitives(converted_bitmap);
}

}  // namespace PTIT