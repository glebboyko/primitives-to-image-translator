#pragma once

#include <list>

#include "concepts.hpp"

namespace PTIT {

struct Coord {
  int x;
  int y;

  Coord operator*(float coef) const;
};

bool operator==(const Coord&, const Coord&) noexcept;
bool operator!=(const Coord& first, const Coord& second) noexcept;
bool operator<(const Coord& first, const Coord& second) noexcept;
bool operator<=(const Coord& first, const Coord& second) noexcept;
bool operator>(const Coord& first, const Coord& second) noexcept;
bool operator>=(const Coord& first, const Coord& second) noexcept;

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

std::list<Coord> FulfillArea(const std::list<Coord>& border);

std::list<Segment> BaseExtractPrimitives(
    std::vector<std::vector<bool>>& bitmap);

template <typename Container, typename Translator>
  requires ImageBitmap<Container> &&
           AvailabilityTranslator<Container, Translator>
std::list<Segment> ExtractPrimitives(const Container& container, int size_x,
                                     int size_y, Translator translator) {
  std::vector<std::vector<bool>> converted_bitmap(size_x,
                                                  std::vector<bool>(size_y));
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      converted_bitmap[x][y] = static_cast<bool>(translator(container[x][y]));
    }
  }

  return BaseExtractPrimitives(converted_bitmap);
}

}  // namespace PTIT