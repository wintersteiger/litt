#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <iomanip>
#include <iostream>

#include <stdint.h>

#include <litt/pid.h>

using namespace litt;

static uint64_t current_time = 0;

class TestTime : public litt::Time {
public:
  TestTime() : litt::Time() {}
  virtual ~TestTime() = default;
  virtual uint64_t get_us() const override { return current_time; }
  virtual void sleep_us(uint64_t us) const override { current_time += us; }
};

static TestTime tt;

void run(PIDController<TestTime> &pid, size_t n, float &input) {
  std::cout << "  ---------  S/P: " << std::fixed << std::setprecision(3) << pid.setpoint << std::endl;

  for (size_t i = 0; i < n; i++) {
    float o = pid.update(input);

    std::cout << std::setw(11) << tt.get_us() << " " << std::setw(7) << std::fixed << std::setprecision(3) << input
              << " " << std::setw(7) << std::fixed << std::setprecision(3) << o << std::endl;

    tt.sleep_us(60e6); // 1 min

    if (o >= 0.0f)
      input += o / 200.0f; // Slow heat input

    input *= 0.99; // Heat loss
  }
}

TEST_CASE("Generate demo runs") {
  PIDController<TestTime> pid(21.0, {10.0, 1e-7, 1.0}, 0.0, 80.0, PIDController<TestTime>::Mode::AUTOMATIC);

  float input = 18.0;
  pid.set_automatic(true, input);

  pid.set_setpoint(21.0);
  run(pid, 60, input);
  pid.set_setpoint(22.0);
  run(pid, 60, input);
  pid.set_setpoint(20.0);
  run(pid, 60, input);
}