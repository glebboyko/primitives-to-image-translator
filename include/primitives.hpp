#pragma once
#include <list>

namespace Primitives {

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

  float GetKCoefficient() const;
  int GetBCoefficient() const;
};

}  // namespace Primitives