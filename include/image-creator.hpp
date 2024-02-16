#pragma once

#include <fstream>

#include "concepts.hpp"
#include "primitives.hpp"

namespace PTIT {

template <typename Container, typename Translator>
  requires ImageBitmap<Container> && UnifiedTranslator<Container, Translator>
void CreateImage(const char* image_file, const Container& image, ssize_t size_x,
                 ssize_t size_y, Translator translator) {
  std::ofstream img_file(image_file);
  if (!img_file.is_open()) {
    throw std::runtime_error("Cannot open file");
  }

  img_file << "P3"
           << "\n";
  img_file << size_x << " " << size_y << "\n";
  img_file << "255"
           << "\n";

  for (ssize_t y = size_y - 1; y >= 0; --y) {
    for (ssize_t x = 0; x < size_x; ++x) {
      const auto& [red, green, blue] = translator(image[x][y]);
      img_file << red << " " << green << " " << blue << "\n";
    }
  }

  img_file.close();
}

struct RGB {
  short red;
  short green;
  short blue;

  static const int kMaxColor = 255;
};

bool operator==(const RGB&, const RGB&) noexcept;

}  // namespace PTIT
