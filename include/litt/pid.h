#ifndef _LITT_PID_H_
#define _LITT_PID_H_

#include <math.h>
#include <stdint.h>

#include "ds.h"

namespace litt {

// Based on Brett Beauregard's excellent series of blog posts:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

struct Coefficients {
  Coefficients(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f) : kp(kp), ki(ki), kd(kd) {}
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
};

template <typename TimeType> class PIDController {
public:
  enum class Mode : uint8_t { AUTOMATIC, MANUAL };

  PIDController(float setpoint = 21.0f, const Coefficients &cs = Coefficients(), float min = 0.0f, float max = 0.0f,
                Mode mode = Mode::AUTOMATIC)
      : coefficients(cs), min(min), max(max), setpoint(setpoint), prev_input(setpoint), mode(mode) {}

  virtual ~PIDController() = default;

  float update(float input, bool new_derivative = true) {
    uint64_t now = time.get_us();

    if (is_automatic()) {
      float delta_t = (now - prev_time) / 1e6f;

      float error = setpoint - input;
      cum_err += coefficients.ki * (error * delta_t);
      cum_err = fmaxf(fminf(cum_err, max), -max);

      float delta_input = prev_time == 0 ? 0 : new_derivative ? (prev_input - input) * delta_t : prev_delta_input;

      output = coefficients.kp * error + cum_err + coefficients.kd * delta_input;
      output = fmaxf(fminf(output, max), min);

      prev_delta_input = delta_input;
    }

    if (input != prev_input || prev_time == 0)
      prev_time = now;
    prev_input = input;

    return output;
  }

  float update_unchanged() {
    // With infrequent temperature reports, we still want to update the
    // cumulative error once in a while. But, when we do that, we do not want to
    // change the derivative term.
    return update(prev_input, false);
  }

  void set_coefficients(const Coefficients &c) { coefficients = c; }

  void set_setpoint(float sp) {
    float bsp = fmaxf(fminf(sp, max), min);
    if (bsp != setpoint)
      cum_err = 0.0f;
    setpoint = bsp;
    if (prev_time != 0)
      update(prev_input);
  }

  void set_automatic(bool v, float input) {
    if (v && !is_automatic())
      initialize(input);
    mode = v ? Mode::AUTOMATIC : Mode::MANUAL;
  }

  void set_output_bounds(float nmin, float nmax) {
    min = nmax < nmin ? nmax : nmin;
    max = nmax < nmin ? nmin : nmax;
    output = fmaxf(fminf(output, max), min);
    cum_err = fmaxf(fminf(cum_err, max), -max);
  }

  Coefficients coefficients;
  float min = 0.0f, max = 100.0f;
  float setpoint = 21.0f;
  float output = 0.0f;
  float cum_err = 0.0f;
  float prev_input = 21.0f;
  float prev_delta_input = 0.0f;

  Mode get_mode() { return mode; }

  bool is_automatic() const { return mode == Mode::AUTOMATIC; }
  bool is_manual() const { return mode == Mode::MANUAL; }

  const char *mode_str() const {
    switch (mode) {
    case Mode::AUTOMATIC:
      return "auto";
    case Mode::MANUAL:
      return "manual";
    default:
      return "?";
    }
  }

protected:
  TimeType time;

  uint64_t prev_time = 0;
  Mode mode = Mode::AUTOMATIC;

  void initialize(float input) {
    prev_time = time.get_us();
    prev_input = input;
    cum_err = fmaxf(fminf(output, max), min);
  }
};

} // namespace litt

#endif // _LITT_PID_H_