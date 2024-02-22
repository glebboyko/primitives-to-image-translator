#pragma once

#include <list>
#include <tuple>

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
  double GetAngle() const;

  void SetLen(int len);
  void SetAngle(double deg);

  std::list<Coord> GetGraphic() const override;
  std::list<Coord> GetArea(int radius) const;

 private:
  Coord a_point_;
  Coord b_point_;

  int GetBCoefficient() const;
  Coord GetCenter() const;
  void SetKCoef(double new_k);
};

class Triangle : public Primitive {
 public:
  Triangle() = default;
  Triangle(const Coord& a_point, const Coord& b_point, const Coord& c_point);

  std::tuple<Coord, Coord, Coord> GetPoints() const;

  std::list<Coord> GetGraphic() const override;

 private:
  Coord a_point_;
  Coord b_point_;
  Coord c_point_;
};

class Circe : public Primitive {
 public:
  Circe() = default;
  Circe(const Coord& center, double radius);

  Coord& GetCenter();
  const Coord& GetCenter() const;
  int& GetRadius();
  int GetRadius() const;

  std::list<Coord> GetGraphic() const override;

 private:
  Coord center_;
  int radius_;
};

std::list<Coord> FulfillArea(const std::list<Coord>& border);

std::list<Segment> BaseExtractPrimitives(
    std::vector<std::vector<bool>>& bitmap);

template <typename Container, typename Translator>
  requires AvailabilityTranslator<Container, Translator>
std::list<Segment> ExtractPrimitives(const Container& container, int size_x,
                                     int size_y, Translator translator) {
  std::vector<std::vector<bool>> converted_bitmap(size_x,
                                                  std::vector<bool>(size_y));
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      converted_bitmap[x][y] = static_cast<bool>(translator(container, x, y));
    }
  }

  return BaseExtractPrimitives(converted_bitmap);
}

}  // namespace PTIT