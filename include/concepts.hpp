#pragma once

#include <concepts>

namespace PTIT {

template <typename Container, typename Translator>
concept UnifiedTranslator =
    requires(Container container, Translator translator) {
      translator(container, std::declval<int>(), std::declval<int>());
    };

template <typename Container, typename Checker>
concept AvailabilityTranslator =
    requires(Container container, Checker checker) {
      UnifiedTranslator<Container, Checker>;
      static_cast<bool>(
          checker(container, std::declval<int>(), std::declval<int>()));
    };

template <typename Container, typename Translator>
concept RGBTranslator = requires(Container container, Translator translator) {
  UnifiedTranslator<Container, Translator>;
  translator(container, std::declval<int>(), std::declval<int>());
};

}  // namespace PTIT