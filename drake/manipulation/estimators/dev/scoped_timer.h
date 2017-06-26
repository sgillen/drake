#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include <fmt/format.h>

#include "drake/common/unused.h"

namespace drake {
namespace manipulation {

// From: drake-distro:49e44b7:drake/common/test/measure_execution.h
using Clock = std::chrono::steady_clock;
using Duration = std::chrono::duration<double>;
using TimePoint = std::chrono::time_point<Clock, Duration>;

inline void sleep(double seconds) {
  int ms = std::round(seconds * 1000);
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline double wall_time() {
  return Duration(Clock::now().time_since_epoch()).count();
}

class Timer {
 public:
  Timer() {}
  inline bool is_active() const { return is_active_; }
  inline double start() {
    if (is_active_) {
      throw std::runtime_error("Timer already started");
    }
    start_ = Clock::now();
    is_active_ = true;
    return elapsed_; // Previously elapsed
  }
  inline double elapsed() const {
    if (is_active_) {
      return Duration(Clock::now() - start_).count();
    } else {
      return elapsed_;
    }
  }
  inline double stop() {
    if (!is_active_) {
      throw std::runtime_error("Time is not started");
    }
    elapsed_ = elapsed();
    is_active_ = false;
    return elapsed_;
  }
 private:
  TimePoint start_;
  double elapsed_{}; // When not active
  bool is_active_{};
};

class ScopedTimer {
 public:
  ScopedTimer(const std::string& message)
    : message_(message) {
    timer_.start();
  }
  void reset() {
    stop();
    timer_.start();
  }
  inline double elapsed() const { return timer_.elapsed(); }
  ~ScopedTimer() {
    stop();
  }
 private:
  void stop() {
    double elapsed = timer_.stop();
    drake::log()->info("[timer] {}: {} sec\n", message_, elapsed)
  }
  Timer timer_;
  std::string message_;
};

#ifndef DRAKE_NO_TIMING
#define DRAKE_SCOPE_TIME(name, message) \
  drake::manipulation::ScopedTimer timer_##name(message); \
  unused(timer_##name)
#else
#define DRAKE_SCOPE_TIME(name, message)
#endif

}  // namespace manipulation
}  // namespace drake
