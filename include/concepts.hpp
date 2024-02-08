#pragma once

#include <concepts>

namespace PTIT {

template <typename Container>
concept ImageBitmap = requires(Container container) { container[0][0]; };

template <typename Container, typename Translator>
concept UnifiedTranslator =
    requires(Container container, Translator translator) {
      ImageBitmap<Container>;
      translator(container[0][0]);
    };

template <typename Container, typename Checker>
concept AvailabilityTranslator =
    requires(Container container, Checker checker) {
      ImageBitmap<Container>;
      UnifiedTranslator<Container, Checker>;
      static_cast<bool>(checker(container));
    };

}  // namespace PTIT