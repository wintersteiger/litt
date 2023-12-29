#ifndef _LITT_RTA_H_
#define _LITT_RTA_H_

#include <stddef.h>
#include <stdint.h>

namespace litt {

template <size_t NUM_ROOMS, typename TimeType> class RoomTemperatureApproximation {
public:
  RoomTemperatureApproximation(TimeType &time) : time(time) {}
  virtual ~RoomTemperatureApproximation() = default;
  virtual void initialize(float setpoint) = 0;
  virtual void set(uint8_t room_id, float t) = 0;
  virtual float get(uint8_t room_id) const = 0;
  virtual uint64_t last_time(uint8_t room_id) const = 0;

  struct Datapoint {
    uint64_t time;
    float temperature;
  };

protected:
  TimeType &time;
};

template <size_t NUM_ROOMS, size_t NUM_POINTS, typename TimeType>
class Extrapolation : public RoomTemperatureApproximation<NUM_ROOMS, TimeType> {
public:
  using Base = RoomTemperatureApproximation<NUM_ROOMS, TimeType>;

  Extrapolation(TimeType &time, float setpoint) : Base(time) { initialize(setpoint); }

  virtual ~Extrapolation() = default;

  virtual void initialize(float setpoint) override {
    for (size_t i = 0; i < NUM_ROOMS; i++) {
      for (size_t j = 0; j < NUM_POINTS; j++)
        datapoints[i].data[j] = {0, setpoint};
    }
  }

  virtual void set(uint8_t room_id, float t) override {
    if (room_id >= NUM_ROOMS)
      return;
    auto &dps = datapoints[room_id].data;
    if (dps[0].time == 0) {
      for (size_t i = 0; i < NUM_POINTS; i++)
        dps[i] = {Base::time.get_us(), t};
    } else {
      for (size_t i = 0; i < NUM_POINTS - 1; i++) {
        dps[i].time = dps[i + 1].time;
        dps[i].temperature = dps[i + 1].temperature;
      }
      dps[NUM_POINTS - 1].time = Base::time.get_us();
      dps[NUM_POINTS - 1].temperature = t;
    }
  }

  float get(uint8_t room_id) const override {
    if (room_id >= NUM_ROOMS)
      return 0.0f;
    uint64_t now = Base::time.get_us();
    const auto &dps = datapoints[room_id].data;
    if (dps[NUM_POINTS - 2].time == dps[NUM_POINTS - 1].time)
      return dps[NUM_POINTS - 1].temperature;
    float slope = (dps[NUM_POINTS - 2].temperature - dps[NUM_POINTS - 1].temperature) /
                  (dps[NUM_POINTS - 2].time - dps[NUM_POINTS - 1].time);
    uint64_t dt = now - dps[NUM_POINTS - 1].time;
    return dps[NUM_POINTS - 1].temperature + (dt * slope) / 2.0f; // Probably too aggressive?
  }

  virtual uint64_t last_time(uint8_t room_id) const override {
    if (room_id >= NUM_ROOMS)
      return 0;
    return datapoints[room_id].data[NUM_POINTS - 1].time;
  }

protected:
  struct Datapoints {
    typename Base::Datapoint data[NUM_POINTS];
  };

  Datapoints datapoints[NUM_ROOMS] = {0};
};

template <size_t NUM_ROOMS, typename TimeType>
class LastValue : public RoomTemperatureApproximation<NUM_ROOMS, TimeType> {
public:
  using Base = RoomTemperatureApproximation<NUM_ROOMS, TimeType>;

  LastValue(TimeType &time, float setpoint) : Base(time) { initialize(setpoint); }

  virtual ~LastValue() = default;

  virtual void initialize(float setpoint) override {
    for (size_t i = 0; i < NUM_ROOMS; i++)
      datapoint[i] = {0, setpoint};
  }

  virtual void set(uint8_t room_id, float t) override {
    if (room_id >= NUM_ROOMS)
      return;
    auto &dp = datapoint[room_id];
    dp.time = Base::time.get_us();
    dp.temperature = t;
  }

  float get(uint8_t room_id) const override {
    if (room_id >= NUM_ROOMS)
      return 0.0f;
    return datapoint[room_id].temperature;
  }

  virtual uint64_t last_time(uint8_t room_id) const override {
    if (room_id >= NUM_ROOMS)
      return 0;
    return datapoint[room_id].time;
  }

protected:
  typename Base::Datapoint datapoint[NUM_ROOMS] = {};
};

template <size_t NUM_ROOMS, size_t NUM_POINTS, typename TimeType>
class Average : public RoomTemperatureApproximation<NUM_ROOMS, TimeType> {
public:
  using Base = RoomTemperatureApproximation<NUM_ROOMS, TimeType>;

  Average(TimeType &time, float setpoint) : Base(time) { initialize(setpoint); }

  virtual ~Average() = default;

  virtual void initialize(float setpoint) override {
    for (size_t i = 0; i < NUM_ROOMS; i++)
      for (size_t j = 0; j < NUM_POINTS; j++)
        datapoints[i].data[j] = {0, setpoint};
  }

  virtual void set(uint8_t room_id, float t) override {
    if (room_id >= NUM_ROOMS)
      return;
    auto &dps = datapoints[room_id].data;
    if (dps[0].time == 0) {
      for (size_t i = 0; i < NUM_POINTS; i++)
        dps[i] = {Base::time.get_us(), t};
    } else {
      dps[next_write_index[room_id]] = {Base::time.get_us(), t};
      next_write_index[room_id] = (next_write_index[room_id] + 1) % NUM_POINTS;
    }
  }

  float get(uint8_t room_id) const override {
    if (room_id >= NUM_ROOMS)
      return 0.0f;
    double r = 0.0f;
    auto &dps = datapoints[room_id].data;
    for (size_t i = 0; i < NUM_POINTS; i++)
      r += dps[i].temperature;
    return r / (double)NUM_POINTS;
  }

  virtual uint64_t last_time(uint8_t room_id) const override {
    if (room_id >= NUM_ROOMS)
      return 0;
    auto inx = (next_write_index[room_id] - 1) % NUM_POINTS;
    return datapoints[room_id].data[inx].time;
  }

protected:
  struct Datapoints {
    typename Base::Datapoint data[NUM_POINTS];
  };

  Datapoints datapoints[NUM_ROOMS] = {};
  size_t next_write_index[NUM_ROOMS] = {};
};
} // namespace litt

#endif // _LITT_RTA_H_
