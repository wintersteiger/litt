#ifndef _LITT_SCHEDULER_H_
#define _LITT_SCHEDULER_H_

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "ds.h"
#include "serialization.h"

namespace litt {
template <size_t MAX_TRANSITIONS, typename TimerType, typename RealTimeType> class Scheduler {
protected:
  static const constexpr uint32_t minutes_per_day = 60 * 24;
  static const constexpr uint32_t minutes_per_week = 7 * 24 * 60;

public:
  enum Mode : uint8_t { NONE = 0x00, HEAT = 0x01, COOL = 0x02, HEAT_AND_COOL = 0x04 };

  enum class Day : uint8_t {
    NODAY = 0x00,
    SUNDAY = 0x01,
    MONDAY = 0x02,
    TUESDAY = 0x04,
    WEDNESDAY = 0x08,
    THURSDAY = 0x10,
    FRIDAY = 0x20,
    SATURDAY = 0x40
    // , AWAY = 0x80 // TODO: Add support for away/vacation?
  };

  using Days = uint8_t;

#pragma pack(push, 1)
  struct ITransition {
    Mode mode = Mode::NONE;
    Days days = 0;
    uint16_t time = 0; // minutes since midnight
    float heat_setpoint = nanf("");
    float cool_setpoint = nanf("");
  };
#pragma pack(pop)

#pragma pack(push, 1)
  struct Configuration {
    ITransition transitions[MAX_TRANSITIONS];
    size_t num_transitions = 0;

    bool scheduling = false;

    bool serialize(uint8_t *buf, size_t size) const {
      using litt::serialize;

      if (!serialize(num_transitions, buf, size))
        return false;

      for (size_t i = 0; i < num_transitions; i++)
        if (!serialize(transitions[i], buf, size))
          return false;

      if (!serialize(scheduling, buf, size))
        return false;

      return true;
    }

    bool deserialize(const uint8_t *buf, size_t size) {
      using litt::deserialize;

      clear_schedule();

      if (!deserialize(num_transitions, buf, size))
        return false;

      for (size_t i = 0; i < num_transitions; i++)
        if (!deserialize(transitions[i], buf, size))
          return false;

      if (!deserialize(scheduling, buf, size))
        return false;

      return true;
    }

    size_t serialized_size() const {
      return sizeof(num_transitions) + num_transitions * sizeof(ITransition) + sizeof(scheduling);
    }

    void clear_schedule() {
      for (size_t i = 0; i < MAX_TRANSITIONS; i++)
        transitions[i] = {};
      num_transitions = 0;
    }
  };
#pragma pack(pop)

  Configuration configuration;

  Scheduler() : timer(timer_cb, this) {}

  virtual ~Scheduler() = default;

  bool start_scheduling() {
    if (configuration.scheduling && timer.is_running())
      return true;

    configuration.scheduling = true;

    if (next())
      on_scheduler_start();
    else
      configuration.scheduling = false;

    return configuration.scheduling;
  }

  bool stop_scheduling() {
    timer.stop();
    bool was_scheduling = configuration.scheduling;
    configuration.scheduling = false;
    if (was_scheduling)
      on_scheduler_stop();
    return true;
  }

  void clear_schedule() {
    stop_scheduling();
    configuration.clear_schedule();
  }

  bool have_schedule() { return configuration.num_transitions != 0; }

  struct Transition {
    uint16_t time = UINT16_MAX;
    float heat_setpoint = nanf("");
    float cool_setpoint = nanf("");
  };

  bool set_schedule(uint8_t days, Mode mode, const Transition *ts, size_t n) {
    if (n >= MAX_TRANSITIONS)
      return false;

    bool was_scheduling = configuration.scheduling;
    stop_scheduling();

    // Note: All schedule entries for the days in `days` are deleted and only the new schedule entries in `ts` are kept
    // for those days. Unclear whether this is the intended behavior for ZCL thermostat schedules.

    size_t num_deleted = 0;

    for (size_t i = 0; i < configuration.num_transitions; i++) {
      ITransition &t = configuration.transitions[i - num_deleted];
      t = configuration.transitions[i];
      t.days &= ~days;
      if (t.days == 0)
        num_deleted++;
    }
    for (size_t i = 0; i < num_deleted; i++) {
      auto &ti = configuration.transitions[configuration.num_transitions - 1 - i];
      ti.mode = Mode::NONE;
      ti.time = 0;
      ti.days = 0;
    }
    configuration.num_transitions -= num_deleted;

    for (size_t i = 0; i < n; i++)
      if (ts[i].time >= minutes_per_week)
        return false;

    for (size_t i = 0; i < n; i++) {
      if (configuration.num_transitions >= MAX_TRANSITIONS)
        return false;
      ITransition &t = configuration.transitions[configuration.num_transitions];
      const Transition &its = ts[i];
      t.mode = mode;
      t.days = days;
      t.time = its.time;
      if (mode & HEAT)
        t.heat_setpoint = its.heat_setpoint;
      if (mode & COOL)
        t.cool_setpoint = its.cool_setpoint;
      configuration.num_transitions++;
    }

    return was_scheduling ? start_scheduling() : true;
  }

  void remove_schedule(uint8_t days) {
    bool was_scheduling = configuration.scheduling;
    stop_scheduling();

    size_t j = 0;
    for (size_t i = 0; i < configuration.num_transitions; i++) {
      if ((configuration.transitions[i].days & days) == days)
        j++;
      if (i + j < configuration.num_transitions) {
        configuration.transitions[i] = configuration.transitions[i + j];
        configuration.transitions[i].days &= ~days;
      }
    }
    for (size_t i = 0; i < j; i++)
      configuration.transitions[configuration.num_transitions - 1 - i].mode = Mode::NONE;
    configuration.num_transitions -= j;

    if (was_scheduling)
      start_scheduling();
  }

  bool is_scheduling() const { return configuration.scheduling; }

  virtual void on_scheduler_start() {}
  virtual void on_scheduler_stop() {}
  virtual void on_scheduled_heat_setpoint_change(float temperature) {}
  virtual void on_scheduled_cool_setpoint_change(float temperature) {}

  Transition current_schedule_transition() const {
    Transition r;
    auto t = current_itransition();
    if (t.mode != Mode::NONE) {
      r.time = t.time;
      r.heat_setpoint = t.heat_setpoint;
      r.cool_setpoint = t.cool_setpoint;
    }
    return r;
  }

  uint16_t minutes_until_next_scheduler_transition(Day day, uint32_t time) const {
    size_t ri = SIZE_MAX;
    uint16_t r = 0;
    bool strictly_greater = true;

    if (day != Day::NODAY) {
      Day cday = day;

      for (uint8_t d = 0; d < 7; d++) {
        for (size_t i = 0; i < configuration.num_transitions; i++) {
          const auto &ti = configuration.transitions[i];
          if (static_cast<uint8_t>(cday) & ti.days) {
            if (((strictly_greater && ti.time > time) || (!strictly_greater && ti.time >= time)) &&
                (ri == SIZE_MAX || ti.time < configuration.transitions[ri].time)) {
              ri = i;
            }
          }
        }

        if (ri != SIZE_MAX) {
          r += configuration.transitions[ri].time - time;
          break;
        } else {
          cday = (cday == Day::SATURDAY) ? Day::SUNDAY : static_cast<Day>(static_cast<uint8_t>(cday) << 1);
          r += minutes_per_day - time;
          time = 0;
          strictly_greater = false;
        }
      }
    }

    return ri == SIZE_MAX ? UINT16_MAX : r;
  }

  uint16_t minutes_until_next_scheduler_transition() const {
    uint8_t weekday;
    uint16_t minutes;
    if (!real_time.get_weekday_minutes(weekday, minutes) || weekday > 6)
      return UINT16_MAX;

    return minutes_until_next_scheduler_transition(static_cast<Day>(1 << weekday), minutes);
  }

protected:
  RealTimeType real_time;
  TimerType timer;

  static bool timer_cb(Timer *timer, void *obj) { return static_cast<Scheduler *>(obj)->on_timer_ftick(); }

  virtual bool on_timer_ftick() {
    timer.stop();
    return next();
  }

  const ITransition &current_itransition() const {
    static ITransition r;

    uint8_t weekday;
    uint16_t minutes;
    if (!real_time.get_weekday_minutes(weekday, minutes) || weekday > 6)
      return r;

    return get_preceding_schedule_transition(static_cast<Day>(1 << weekday), minutes);
  }

  const ITransition &get_preceding_schedule_transition(Day day, uint32_t time) const {
    static ITransition default_r;
    size_t ri = SIZE_MAX;

    if (day != Day::NODAY) {
      Day cday = day;

      for (uint8_t d = 0; d < 7; d++) {
        for (size_t i = 0; i < configuration.num_transitions; i++) {
          const auto &ti = configuration.transitions[i];
          if ((static_cast<uint8_t>(day) & ti.days) != 0) {
            if (ti.time <= time && (ri == SIZE_MAX || ti.time >= configuration.transitions[ri].time))
              ri = i;
          }
        }

        if (ri != SIZE_MAX)
          break;
        else {
          cday = (cday == Day::SUNDAY) ? Day::SATURDAY : static_cast<Day>(static_cast<uint8_t>(cday) >> 1);
          time = minutes_per_day - 1;
        }
      }
    }

    return ri == SIZE_MAX ? default_r : configuration.transitions[ri];
  }

  bool next() {
    if (!configuration.scheduling || configuration.num_transitions == 0)
      return false;

    uint8_t weekday;
    uint16_t minutes;
    if (!real_time.get_weekday_minutes(weekday, minutes) || weekday > 6)
      return false;

    const auto &tc = get_preceding_schedule_transition(static_cast<Day>(1 << weekday), minutes);

    if (tc.mode == Mode::NONE)
      return false;

    if ((tc.mode == Mode::HEAT || tc.mode == Mode::HEAT_AND_COOL) && !isnan(tc.heat_setpoint))
      on_scheduled_heat_setpoint_change(tc.heat_setpoint);

    if ((tc.mode == Mode::COOL || tc.mode == Mode::HEAT_AND_COOL) && !isnan(tc.cool_setpoint))
      on_scheduled_cool_setpoint_change(tc.cool_setpoint);

    uint16_t t = minutes_until_next_scheduler_transition(static_cast<Day>(1 << weekday), minutes);

    if (t >= minutes_per_week)
      return false;
    else {
      if (t == 0)
        t = 1;

      timer.start(t * 60.0f * 1e6f);
      return true;
    }
  }
};

} // namespace litt

#endif // _LITT_SCHEDULER_H_
