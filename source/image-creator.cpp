#include "image-creator.hpp"

namespace PTIT {

bool operator==(const RGB& first, const RGB& second) noexcept {
  return first.red == second.red && first.green == second.green &&
         first.blue == second.blue;
}

}  // namespace PTIT