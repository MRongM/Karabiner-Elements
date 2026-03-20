#pragma once

#include <pqrs/json.hpp>
#include <nlohmann/json.hpp>
#include <fmt/format.h>

namespace krbn {
namespace manipulator {
namespace manipulators {
namespace mouse_wheel_to_scroll {

// Options are intentionally minimal.
// Most tuning should be done via core complex_modifications parameters (see Settings UI).
struct options final {
  static constexpr double epsilon_default_value = 0.01; // stop condition

  options(void) : epsilon_(epsilon_default_value) {}

  double get_epsilon(void) const {
    return epsilon_;
  }

  void set_epsilon(double value) {
    epsilon_ = value;
  }

  void update(const nlohmann::json& json) {
    pqrs::json::requires_object(json, "json");

    for (const auto& [key, value] : json.items()) {
      if (key == "epsilon") {
        pqrs::json::requires_number(value, "`epsilon`");
        set_epsilon(value.get<double>());
      } else {
        throw pqrs::json::unmarshal_error(fmt::format("unknown key `{0}` in `{1}`", key, pqrs::json::dump_for_error_message(json)));
      }
    }
  }

private:
  double epsilon_;
};

} // namespace mouse_wheel_to_scroll
} // namespace manipulators
} // namespace manipulator
} // namespace krbn

