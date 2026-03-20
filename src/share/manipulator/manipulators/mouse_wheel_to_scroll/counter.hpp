#pragma once

#include "core_configuration/core_configuration.hpp"
#include "options.hpp"
#include "types/pointing_motion.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <pqrs/dispatcher.hpp>
#include <nod/nod.hpp>
#include <pqrs/osx/chrono.hpp>

namespace krbn {
namespace manipulator {
namespace manipulators {
namespace mouse_wheel_to_scroll {

class counter final : public pqrs::dispatcher::extra::dispatcher_client {
public:
  using wheel_to_scroll_event_t = pointing_motion;

  // Signals (invoked from dispatcher thread)
  nod::signal<void(const wheel_to_scroll_event_t&)> scroll_event_arrived;

  static constexpr int timer_interval_milliseconds = 20;

  counter(std::weak_ptr<pqrs::dispatcher::dispatcher> weak_dispatcher,
          pqrs::not_null_shared_ptr_t<const core_configuration::details::complex_modifications_parameters> parameters,
          const options& options)
      : dispatcher_client(weak_dispatcher),
        parameters_(parameters),
        options_(options),
        current_y_(0.0),
        current_x_(0.0),
        buffer_y_(0.0),
        buffer_x_(0.0),
        remainder_y_(0.0),
        remainder_x_(0.0),
        last_input_dir_y_(0),
        last_input_dir_x_(0),
        timer_(*this) {
  }

  ~counter(void) {
    detach_from_dispatcher([this] {
      timer_.stop();
    });
  }

  void async_reset(void) {
    enqueue_to_dispatcher([this] {
      current_y_ = 0.0;
      current_x_ = 0.0;
      buffer_y_ = 0.0;
      buffer_x_ = 0.0;
      remainder_y_ = 0.0;
      remainder_x_ = 0.0;
      last_input_dir_y_ = 0;
      last_input_dir_x_ = 0;
    });
  }

  void update(const pointing_motion& motion, pqrs::dispatcher::time_point /*time_point*/) {
    // Only deal with wheel deltas.
    auto dy = motion.get_vertical_wheel();
    auto dx = motion.get_horizontal_wheel();

    if (dy == 0 && dx == 0) {
      return;
    }

    enqueue_to_dispatcher([this, dy, dx] {
      const auto speed_rate = parameters_->make_mouse_wheel_to_scroll_speed_multiplier_rate();

      if (dy != 0) {
        const int dir = dy > 0 ? 1 : -1;
        if (last_input_dir_y_ == 0 || dir == last_input_dir_y_) {
          buffer_y_ += static_cast<double>(dy) * speed_rate;
        } else {
          // Direction changed: reset the ongoing curve to avoid weird mixing.
          buffer_y_ = static_cast<double>(dy) * speed_rate;
          current_y_ = 0.0;
          remainder_y_ = 0.0;
        }
        last_input_dir_y_ = dir;
      }

      if (dx != 0) {
        const int dir = dx > 0 ? 1 : -1;
        if (last_input_dir_x_ == 0 || dir == last_input_dir_x_) {
          buffer_x_ += static_cast<double>(dx) * speed_rate;
        } else {
          buffer_x_ = static_cast<double>(dx) * speed_rate;
          current_x_ = 0.0;
          remainder_x_ = 0.0;
        }
        last_input_dir_x_ = dir;
      }

      // Start emitting split deltas.
      if (!timer_running_) {
        timer_running_ = true;
        timer_.start(
            [this] {
              bool continue_timer = tick();
              if (!continue_timer) {
                timer_running_ = false;
                timer_.stop();
              }
            },
            std::chrono::milliseconds(timer_interval_milliseconds));
      }
    });
  }

private:
  static int extract_int_from_remainder(double& remainder) {
    if (remainder == 0.0) {
      return 0;
    }

    // Use floor/ceil to preserve sign for negative values.
    int truncated;
    if (remainder > 0.0) {
      truncated = static_cast<int>(std::floor(remainder));
      remainder -= static_cast<double>(truncated);
    } else {
      truncated = static_cast<int>(std::ceil(remainder));
      remainder -= static_cast<double>(truncated);
    }

    return truncated;
  }

  bool tick(void) {
    const double duration_ms = static_cast<double>(parameters_->get_mouse_wheel_to_scroll_duration_milliseconds());
    const double dt_ms = static_cast<double>(timer_interval_milliseconds);

    // Curve approach factor per tick.
    //  - duration_ms -> time constant for convergence
    //  - when duration_ms is small, we snap quickly
    double trans = 1.0;
    if (duration_ms > 0.0) {
      trans = 1.0 - std::exp(-dt_ms / duration_ms);
      trans = std::min(1.0, std::max(0.0, trans));
    }

    const double epsilon = options_.get_epsilon();

    const double frame_y = (buffer_y_ - current_y_) * trans;
    const double frame_x = (buffer_x_ - current_x_) * trans;

    current_y_ += frame_y;
    current_x_ += frame_x;

    remainder_y_ += frame_y;
    remainder_x_ += frame_x;

    const int out_y = extract_int_from_remainder(remainder_y_);
    const int out_x = extract_int_from_remainder(remainder_x_);

    if (out_y != 0 || out_x != 0) {
      auto y = out_y;
      auto x = out_x;
      enqueue_to_dispatcher([this, y, x] {
        auto motion = pointing_motion(0, 0, y, x);
        scroll_event_arrived(motion);
      });
    }

    const double residual_y = buffer_y_ - current_y_;
    const double residual_x = buffer_x_ - current_x_;

    // Stop based on curve convergence only.
    // Wheel outputs are integer-based, so a tiny fractional remainder might never
    // cross +/-1.0 to be emitted as an int. Dropping that leftover is acceptable.
    if (std::abs(residual_y) <= epsilon && std::abs(residual_x) <= epsilon) {
      // Flush any last integer parts from remainder (usually 0).
      const int final_y = extract_int_from_remainder(remainder_y_);
      const int final_x = extract_int_from_remainder(remainder_x_);
      if (final_y != 0 || final_x != 0) {
        auto y = final_y;
        auto x = final_x;
        enqueue_to_dispatcher([this, y, x] {
          auto motion = pointing_motion(0, 0, y, x);
          scroll_event_arrived(motion);
        });
      }

      buffer_y_ = 0.0;
      buffer_x_ = 0.0;
      current_y_ = 0.0;
      current_x_ = 0.0;
      remainder_y_ = 0.0;
      remainder_x_ = 0.0;
      last_input_dir_y_ = 0;
      last_input_dir_x_ = 0;
      return false;
    }

    return true;
  }

  pqrs::not_null_shared_ptr_t<const core_configuration::details::complex_modifications_parameters> parameters_;
  const options options_;

  // Mos-like buffer/current approach:
  // - buffer_* is the desired accumulated wheel delta target
  // - current_* moves towards buffer_* on each tick
  // - remainder_* stores fractional parts before converting into int wheel deltas
  double current_y_;
  double current_x_;
  double buffer_y_;
  double buffer_x_;
  double remainder_y_;
  double remainder_x_;

  int last_input_dir_y_;
  int last_input_dir_x_;

  pqrs::dispatcher::extra::timer timer_;
  bool timer_running_ = false;
};

} // namespace mouse_wheel_to_scroll
} // namespace manipulators
} // namespace manipulator
} // namespace krbn

