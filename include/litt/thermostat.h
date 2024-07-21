#ifndef _LITT_THERMOSTAT_H_
#define _LITT_THERMOSTAT_H_

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "ch_interface.h"
#include "commands.h"
#include "ds.h"
#include "pid.h"
#include "rta.h"
#include "serialization.h"

#if __cplusplus >= 201103L && LITT_HAVE_STL
#include <type_traits>
#endif

namespace litt {

template <size_t NUM_ROOMS, class TimeType, typename TimerType> class Thermostat {
#if __cplusplus >= 201103L && LITT_HAVE_STL
  static_assert(std::is_base_of<litt::Time, TimeType>::value, "TimeType must derive from litt::Time");
  static_assert(std::is_base_of<litt::Timer, TimerType>::value, "TimerType must derive from litt::Timer");
#endif

protected:
  class FlowSetpointUpdater {
  public:
    FlowSetpointUpdater(Thermostat &t, bool approximate) : t(t), approximate(approximate) {
      before = t.flow_setpoint();
    }

    virtual ~FlowSetpointUpdater() {
      float after = t.flow_setpoint();
      if (after != before)
        t.on_flow_setpoint_change(before, after, approximate);
    }

  protected:
    Thermostat &t;
    float before;
    bool approximate;
  };

public:
  enum class Mode : uint8_t { OFF = 0, AUTOMATIC, MANUAL, INVALID };
  enum class MixingFunction : uint8_t { FIRST = 0, MAX, AVERAGE, INVALID };

#pragma pack(push, 1)
  struct Configuration {
    // Minimum flow setpoint [C]
    float min_flow_setpoint = 30.0f;

    // Tiny cycle classification threshold
    float tiny_cycle_minutes = 1.0f;

    // Short cycle classification threshold
    float short_cycle_minutes = 8.0f;

    // Spike protection
    bool spike_protection = true;

    // Spike protection deadband
    float spike_protection_deadband = 10.0f;

    // Operating mode
    Mode mode = Mode::OFF;

    // Mixing function
    MixingFunction mixing_function = MixingFunction::FIRST;

    // A reference temperature for selection of the weather compensation curve [C].
    // (Ideal calls this the "room temperature setpoint", others use an index number.)
    float weather_compensation_ref_temp = 21.0f;

    // The heat loss constant for your house.
    // This is (U-Value [W/((m^2)K)] * Area [m^2] / Radiator output [W/K]) * dT [K].
    float heat_loss_constant = 5.5f;

    // Radiator exponent, e.g. 1.3 - 1.4 for normal radiators (see specifications).
    // (Depends on the ratio between radiation and convection?)
    // (Roughly, aluminium closer to 1.4, steel closer to 1.3.)
    float radiator_exponent = 1.3f;

    bool serialize(uint8_t *&buf, size_t &sz) const {
      using litt::serialize;
      return serialize(min_flow_setpoint, buf, sz) && serialize(tiny_cycle_minutes, buf, sz) &&
             serialize(short_cycle_minutes, buf, sz) && serialize(spike_protection, buf, sz) &&
             serialize(spike_protection_deadband, buf, sz) && serialize(mode, buf, sz) &&
             serialize(mixing_function, buf, sz) && serialize(weather_compensation_ref_temp, buf, sz) &&
             serialize(heat_loss_constant, buf, sz) && serialize(radiator_exponent, buf, sz);
    }

    bool deserialize(const uint8_t *&buf, size_t &sz) {
      using litt::deserialize;
      return deserialize(min_flow_setpoint, buf, sz) && deserialize(tiny_cycle_minutes, buf, sz) &&
             deserialize(short_cycle_minutes, buf, sz) && deserialize(spike_protection, buf, sz) &&
             deserialize(spike_protection_deadband, buf, sz) && deserialize(mode, buf, sz) &&
             deserialize(mixing_function, buf, sz) && deserialize(weather_compensation_ref_temp, buf, sz) &&
             deserialize(heat_loss_constant, buf, sz) && deserialize(radiator_exponent, buf, sz);
    }

    size_t serialized_size() const {
      return sizeof(min_flow_setpoint) + sizeof(tiny_cycle_minutes) + sizeof(short_cycle_minutes) +
             sizeof(spike_protection) + sizeof(spike_protection_deadband) + sizeof(mode) + sizeof(mixing_function) +
             sizeof(weather_compensation_ref_temp) + sizeof(heat_loss_constant) + sizeof(radiator_exponent);
    }
  };
#pragma pack(pop)

#pragma pack(push, 1)
  struct Statistics {
    uint64_t tiny_cycles = 0, short_cycles = 0, normal_cycles = 0;
    uint32_t num_spike_protected = 0, num_tiny_cycle_protected = 0;

    bool serialize(uint8_t *&buf, size_t &sz) const {
      using litt::serialize;
      return serialize(tiny_cycles, buf, sz) && serialize(short_cycles, buf, sz) && serialize(normal_cycles, buf, sz) &&
             serialize(num_spike_protected, buf, sz) && serialize(num_tiny_cycle_protected, buf, sz);
    }

    bool deserialize(const uint8_t *&buf, size_t &sz) {
      using litt::deserialize;
      bool r = deserialize(tiny_cycles, buf, sz) && deserialize(short_cycles, buf, sz) &&
               deserialize(normal_cycles, buf, sz) && deserialize(num_spike_protected, buf, sz) &&
               deserialize(num_tiny_cycle_protected, buf, sz);
      if (!r)
        *this = Statistics();
      return r;
    }

    size_t serialized_size() const {
      return sizeof(tiny_cycles) + sizeof(short_cycles) + sizeof(normal_cycles) + sizeof(num_spike_protected) +
             sizeof(num_tiny_cycle_protected);
    }
  };
#pragma pack(pop)

  Configuration configuration;
  Statistics statistics;

  Thermostat(CentralHeatingInterface &chif) : chif(chif), tiny_cycle_protect_timer(tiny_cycle_protect_timer_cb, this) {}

  virtual ~Thermostat() {}

  bool is_off() const { return configuration.mode == Mode::OFF; }
  bool is_manual() const { return configuration.mode == Mode::MANUAL; }
  bool is_automatic() const { return configuration.mode == Mode::AUTOMATIC; }

  /// @brief String representation of an operating mode.
  /// @param mode mode to convert to a string
  /// @return a string
  static const char *mode_string(Mode mode) {
    switch (mode) {
    case Mode::OFF:
      return "off";
    case Mode::AUTOMATIC:
      return "automatic";
    case Mode::MANUAL:
      return "manual";
    default:
      return "unknown";
    }
  }

  /// @brief String representation of the current operating mode.
  /// @return a string
  const char *get_mode_string() const { return mode_string(configuration.mode); }

  /// @brief Sets the operating mode of the thermostat.
  /// @param m new operating mode
  void set_mode(Mode m) {
    if (m >= Mode::INVALID)
      m = Mode::OFF;
    Mode old_m = configuration.mode;
    configuration.mode = m;
    on_mode_change(old_m, configuration.mode);
  }

  /// @brief Called when the operating mode of the thermostat changes.
  /// @param from previous operating mode
  /// @param to new operating mode
  /// @note The new mode is not necessarily different from the previous one (e.g. during initialization).
  virtual void on_mode_change(Mode from, Mode to) {
    configuration.mode = to;

    switch (to) {
    case Mode::OFF:
      if (chif.enabled())
        chif.disable();
      disengage_protection();
      break;
    case Mode::MANUAL:
      chif.enable();
      disengage_protection();
      break;
    default:
      break;
    }

    // Flow setpoint may have to be set even if it didn't change (e.g. off -> automatic, where it is
    // tracked/remembered, but not passed to the CH interface).
    auto fsp = flow_setpoint();
    on_flow_setpoint_change(fsp, fsp, false);
  }

  virtual void set_max_flow_setpoint_bounds(float min, float max) {}

  /// @brief Called when the boiler ignites or extinguishes the flame.
  /// @param on true if the flame is now on, false otherwise
  /// @note The flame may be reported to be on even if it is already known to be on.
  virtual void on_flame_change(bool on) {
    if (on) {
      flame_on_time = time.get_us();

      // If the boiler ignites while spike protection is active (e.g. due to DHW demand), we can't do anything about
      // it. We just reset the protection and let it kick back in later, if necessary.
      disengage_spike_protect();

      // Engage tiny cycle protection, i.e. don't lower the setpoint for a while.
      engage_tiny_cycle_protect();
    } else {
      if (flame_on_time != 0) {
        auto now = time.get_us();
        auto delta_t = (now - flame_on_time) / (60.0f * 1e6f);
        if (delta_t < configuration.tiny_cycle_minutes)
          statistics.tiny_cycles++;
        else if (delta_t < configuration.short_cycle_minutes)
          statistics.short_cycles++;
        else
          statistics.normal_cycles++;
        flame_on_time = 0;
      }

      disengage_tiny_cycle_protect();
    }
  }

  virtual float flow_setpoint() {
    if (is_manual())
      return manual_setpoint;
    else
      return chif.flow_setpoint();
  }

  /// @brief Called when the flow setpoint changes
  /// @param from previous flow setpoint
  /// @param to new setpoint
  /// @param approximated true if the setpoint was approximated (e.g. interpolated)
  /// @note The new flow setpoint is not necessarily different from the previous one.
  virtual void on_flow_setpoint_change(float from, float to, bool approximated) {
    switch (configuration.mode) {
    case Mode::OFF: {
      if (chif.enabled())
        chif.disable();
      break;
    }
    case Mode::MANUAL:
      chif.set_flow_setpoint(manual_setpoint);
      break;
    case Mode::AUTOMATIC: {
      float flow = chif.flow_temperature();
      check_spike_protect(flow, flow, from, to);
      if (!tiny_cycle_protect_active || to > from)
        chif.set_flow_setpoint(to);
      break;
    }
    case Mode::INVALID:
      break;
    }
  }

  /// @brief To be called when the flow temperature changes
  /// @param from previous flow temperature
  /// @param to new flow temperature
  /// @note The new flow temperature is not necessarily different from the previous one.
  // TODO: This should be part of the boiler interface? Maybe both?
  virtual void on_flow_temperature_change(float from, float to) {
    if (!is_off() && !is_manual()) {
      float setpoint = flow_setpoint();
      check_spike_protect(from, to, setpoint, setpoint);
    }
  }

  /// @brief Called when spike protection is activated or deactivated.
  /// @param on true if spike protection is now active, false otherwise
  virtual void on_spike_protect(bool on) {}

  /// @brief Predicate indicating whether spike protection is active.
  /// @return true if spike protection is active, false otherwise
  bool is_spike_protect_active() const { return spike_protect_active; }

  /// @brief Called when tiny cycle protection is activated or deactivated.
  /// @param on true if tiny cycle protection is now active, false otherwise
  virtual void on_tiny_cycle_protect(bool on) {}

  /// @brief Sets the flow setpoint and changes the mode to manual.
  /// @param temperature
  /// @return true if successful, false otherwise
  virtual bool set_flow_setpoint(float temperature) {
    if (temperature < 0.0f || temperature > 100.0f || isnan(temperature))
      return false;
    manual_setpoint = temperature;
    if (configuration.mode != Mode::MANUAL)
      set_mode(Mode::MANUAL);
    else
      on_flow_setpoint_change(manual_setpoint, manual_setpoint, false);
    return true;
  }

  /// @brief Called to execute a flow setpoint change command.
  /// @param user_data user-defined data associated with the command
  /// @param temperature new flow setpoint
  /// @return true if the command was executed successfully, false otherwise
  virtual bool on_set_flow_setpoint_cmd(uint16_t user_data, float temperature) {
    return set_flow_setpoint(temperature);
  }

  /// @brief Called to execute a thermostat mode change command.
  /// @param user_data user-defined data associated with the command
  /// @param temperature new flow setpoint
  /// @return true if the command was executed successfully, false otherwise
  virtual bool on_set_thermostat_mode_cmd(uint16_t user_data, Mode mode) {
    set_mode(mode);
    return true;
  }

  /// @brief Set the mixing function.
  /// @param function the mixing function
  /// @return true if the command was executed successfully, false otherwise
  virtual bool set_mixing_function(MixingFunction function) {
    if (function >= MixingFunction::INVALID)
      return false;
    FlowSetpointUpdater u(*this, false);
    auto before = configuration.mixing_function;
    if (before != function)
      on_mixing_function_change(before, function);
    configuration.mixing_function = function;
    return true;
  }

  /// @brief Called when the mixing function changes.
  /// @param from previous mixing function
  /// @param to new mixing function
  virtual void on_mixing_function_change(MixingFunction from, MixingFunction to) {}

  /// @brief Called to execute a mixing function change command.
  /// @param user_data user-defined data associated with the command
  /// @param function index of the mixing function
  /// @return true if the command was executed successfully, false otherwise
  virtual bool on_set_mixing_function_cmd(uint16_t user_data, uint8_t function) {
    return set_mixing_function(static_cast<MixingFunction>(function));
  }

  static const char *mixing_function_name(MixingFunction function) {
    const char *function_names[] = {"first", "max", "average"};
    uint8_t mfi = static_cast<uint8_t>(function);
    return function >= MixingFunction::INVALID ? "invalid" : function_names[mfi];
  }

  /// @brief Called when the outside air temperature changes.
  /// @param from the previous outside air temperature.
  /// @param to the new outside air temperature.
  virtual void on_outside_air_temperature_change(float from, float to) {
    FlowSetpointUpdater u(*this, false);
    outside_air_temperature = to;
  }

  /// @brief Computes the weather-compensated flow setpoint.
  /// @return The weather-compensated flow setpoint.
  virtual float weather_compensated_flow_setpoint() const {
    const float c = configuration.heat_loss_constant;
    const float rt = configuration.weather_compensation_ref_temp;
    const float dt = rt - outside_air_temperature;
    const float exp = 1.0f / configuration.radiator_exponent;
    const float r = pow(c, exp) * pow(dt, exp) + 5 + rt; // +5 = dT/2. Remove?
    return isnormal(r) ? r : 0.0f;
  }

  /// @brief Executes a command.
  /// @param f command frame
  /// @return true if the command was executed successfully, false otherwise
  virtual bool execute(const CommandFrame &f) {
    auto cmd_id = f.command_id;
    switch (cmd_id) {
    case CommandID::SET_FLOW_SETPOINT: {
      float fsp = OpenTherm::to_f88((uint16_t)(f.payload & 0x0000FFFF));
      return on_set_flow_setpoint_cmd(f.user_data, fsp);
    }
    case CommandID::SET_THERMOSTAT_MODE:
      return on_set_thermostat_mode_cmd(f.user_data, static_cast<Mode>(f.payload));
    case CommandID::SET_MIXING_FUNCTION:
      return on_set_mixing_function_cmd(f.user_data, f.payload & 0xFF);
    default:
      return false;
    }
  }

  /// @brief Disengages all protections
  void disengage_protection() {
    disengage_spike_protect();
    disengage_tiny_cycle_protect();
  }

protected:
  TimeType time;
  CentralHeatingInterface &chif;

  float manual_setpoint = 30.0f;

  float outside_air_temperature = nanf("");

  uint64_t flame_on_time = 0;

  bool spike_protect_active = false;
  bool tiny_cycle_protect_active = false;

  void engage_spike_proctect() {
    chif.disable();
    if (!spike_protect_active) {
      spike_protect_active = true;
      statistics.num_spike_protected++;
      on_spike_protect(true);
    }
  }

  void disengage_spike_protect() {
    if (spike_protect_active) {
      spike_protect_active = false;
      on_spike_protect(false);
    }
  }

  void engage_tiny_cycle_protect() {
    if (!tiny_cycle_protect_active) {
      tiny_cycle_protect_active = true;
      tiny_cycle_protect_timer.start(configuration.short_cycle_minutes * 60.0f * 1e6f);
      statistics.num_tiny_cycle_protected++;
      on_tiny_cycle_protect(true);
    }
  }

  void disengage_tiny_cycle_protect() {
    if (tiny_cycle_protect_active) {
      tiny_cycle_protect_timer.stop();
      tiny_cycle_protect_active = false;
      on_tiny_cycle_protect(false);
    }
  }

  void check_spike_protect(float from_flow, float to_flow, float from_setpoint, float to_setpoint) {
    if (!configuration.spike_protection || is_off() || is_manual())
      return;

    if (!spike_protect_active) {
      // LOG_DBG("%d %0.2f %0.2f %0.2f %0.2f %d", chif.flame(), from_flow, to_flow, from_setpoint,
      //         to_setpoint, (from_setpoint < from_flow && to_setpoint >= to_flow));

      // Note: chif.flame() may not be up to date yet; to_flow > from_flow also indicates that the flame is on. If so,
      // we do not want to trigger spike protection.
      if (!(chif.flame() || (from_flow != 0.0f && to_flow > from_flow)) &&
          ((from_setpoint < from_flow && to_setpoint >= to_flow) ||
           (to_setpoint >= to_flow && to_flow > to_setpoint - configuration.spike_protection_deadband)))
        engage_spike_proctect();
    } else if (to_flow <= to_setpoint - configuration.spike_protection_deadband ||
               to_flow >= to_setpoint + configuration.spike_protection_deadband) {
      // LOG_DBG("release: %d %0.2f %0.2f %0.2f %0.2f %d %d", chif.flame(), from_flow, to_flow,
      //         from_setpoint, to_setpoint, (to_flow <= to_setpoint - settings.spike_protect_deadband),
      //         (to_flow >= to_setpoint + settings.spike_protect_deadband));
      disengage_spike_protect();
    }

    if (!spike_protect_active) {
      if (configuration.min_flow_setpoint <= to_setpoint && to_setpoint <= chif.max_flow_setpoint())
        chif.enable();
      else
        chif.disable();
    }
  }

  TimerType tiny_cycle_protect_timer;

  static bool tiny_cycle_protect_timer_cb(Timer *timer, void *obj) {
    return static_cast<Thermostat *>(obj)->on_tiny_cycle_protect_ftick();
  }

  virtual bool on_tiny_cycle_protect_ftick() {
    FlowSetpointUpdater u(*this, false);
    disengage_tiny_cycle_protect();
    return true;
  }
};

template <size_t NUM_PIDS, class TimeType, typename TimerType>
class PIDDrivenThermostat : public Thermostat<NUM_PIDS, TimeType, TimerType> {
#if __cplusplus >= 201103L && LITT_HAVE_STL
  static_assert(std::is_base_of<litt::Time, TimeType>::value, "TimeType must derive from litt::Time");
  static_assert(std::is_base_of<litt::Timer, TimerType>::value, "TimerType must derive from litt::Timer");
#endif
public:
  using RTATT = RoomTemperatureApproximation<NUM_PIDS, TimeType>;
  using Base = Thermostat<NUM_PIDS, TimeType, TimerType>;

  using Base::configuration;
  using typename Base::FlowSetpointUpdater;
  using typename Base::Mode;

  PIDDrivenThermostat(CentralHeatingInterface &chif, float const (&weights)[NUM_PIDS], float default_setpoint = 21.0f)
      : Base(chif), pid_timer(10e6, 10e6, on_pid_timer_cb, nullptr, this), rta(time, default_setpoint) {
    for (size_t i = 0; i < NUM_PIDS; i++) {
      pids[i] = PIDController<TimeType>(default_setpoint, {8.0f, 0.01875f, 0.01f}, 0.0f, 100.0f);
      this->weights[i] = weights[i];
    }
  }

  virtual ~PIDDrivenThermostat() = default;

  virtual float flow_setpoint() override {
    if (Base::is_automatic()) {
      switch (configuration.mixing_function) {
      case Base::MixingFunction::MAX:
        return flow_setpoint_from_pids_max();
      case Base::MixingFunction::AVERAGE:
        return flow_setpoint_from_pids_avg();
      case Base::MixingFunction::FIRST:
      default:
        return flow_setpoint_from_pid(0);
      }
    } else
      return Base::flow_setpoint();
  }

  virtual void on_mode_change(Mode from, Mode to) override {
    typename Base::FlowSetpointUpdater u(*this, to == Mode::MANUAL);
    Base::on_mode_change(from, to);

    switch (to) {
    case Mode::OFF:
      for (size_t i = 0; i < NUM_PIDS; i++)
        pids[i].set_automatic(false, 0.0f);
      break;
    case Mode::MANUAL: {
      for (size_t i = 0; i < NUM_PIDS; i++)
        pids[i].set_automatic(false, flow_setpoint());
      break;
    }
    case Mode::AUTOMATIC:
    default:
      break;
    }
  }

  virtual void set_max_flow_setpoint_bounds(float min, float max) override {
    Base::set_max_flow_setpoint_bounds(min, max);
    for (size_t i = 0; i < NUM_PIDS; i++)
      pids[i].set_output_bounds(min, Base::chif.max_flow_setpoint());
  }

  virtual float room_setpoint(uint8_t id) const {
    if (id >= NUM_PIDS)
      return 0.0f;
    return pids[id].setpoint;
  }

  float room_temperature(uint8_t id) const { return rta.get(id); }

  uint64_t room_temperature_time(uint8_t id) const {
    if (id >= NUM_PIDS)
      return 0;
    return rta.last_time(id);
  }

  void set_pid_coefficients(uint8_t id, const Coefficients &cs) {
    if (id < NUM_PIDS)
      pids[id].set_coefficients(cs);
  }

  float pid_weight(uint8_t id) const {
    if (id >= NUM_PIDS)
      return 0.0f;
    return weights[id];
  }

  const char *pid_mode(uint8_t id) const {
    if (id >= NUM_PIDS)
      return "?";
    return pids[id].mode_str();
  }

  bool report_temperature(uint8_t id, float temperature) {
    if (id >= NUM_PIDS)
      return false;
    if (!Base::is_off() && !Base::is_manual()) {
      typename Base::FlowSetpointUpdater u(*this, false);
      rta.set(id, temperature);
      pids[id].update(room_temperature(id));
      pid_timer.restart(10e6);
    } else
      rta.set(id, temperature);
    return true;
  }

  virtual bool on_temperature_report_cmd(uint16_t user_data, uint8_t id, float temperature) {
    return report_temperature(id, temperature);
  }

  bool report_setpoint(uint8_t id, float temperature) {
    if (id >= NUM_PIDS)
      return false;
    pids[id].set_setpoint(temperature);
    return true;
  }

  virtual bool on_set_pid_setpoint_cmd(uint16_t user_data, uint8_t id, float temperature) {
    return report_setpoint(id, temperature);
  }

  virtual bool on_set_pid_kp_cmd(uint16_t user_data, uint8_t id, float value) {
    if (id >= NUM_PIDS)
      return false;
    auto cs = pids[id].coefficients;
    cs.kp = value;
    pids[id].set_coefficients(cs);
    return true;
  }

  virtual bool on_set_pid_ki_cmd(uint16_t user_data, uint8_t id, float value) {
    if (id >= NUM_PIDS)
      return false;
    auto cs = pids[id].coefficients;
    cs.ki = value;
    pids[id].set_coefficients(cs);
    return true;
  }

  virtual bool on_set_pid_kd_cmd(uint16_t user_data, uint8_t id, float value) {
    if (id >= NUM_PIDS)
      return false;
    auto cs = pids[id].coefficients;
    cs.kd = value;
    pids[id].set_coefficients(cs);
    return true;
  }

  virtual bool on_set_pid_cum_err_cmd(uint16_t user_data, uint8_t id, float value) {
    if (id >= NUM_PIDS)
      return false;
    pids[id].cum_err = value;
    return true;
  }

  virtual bool on_set_pid_weight_cmd(uint16_t user_data, uint8_t id, float value) {
    if (id >= NUM_PIDS)
      return false;
    weights[id] = value;
    return true;
  }

  virtual bool on_set_pid_automatic_cmd(uint16_t user_data, uint8_t id, float room_setpoint) {
    if (id >= NUM_PIDS)
      return false;
    typename Base::FlowSetpointUpdater u(*this, true);
    pids[id].set_setpoint(room_setpoint);
    pids[id].set_automatic(true, room_temperature(id));
    return true;
  }

  virtual bool execute(const CommandFrame &f) override {
    auto cmd_id = f.command_id;
    switch (cmd_id) {
    case CommandID::SET_PID_SETPOINT:
    case CommandID::SET_PID_KP:
    case CommandID::SET_PID_KI:
    case CommandID::SET_PID_KD:
    case CommandID::SET_PID_CUM_ERR:
    case CommandID::SET_PID_AUTOMATIC: {
      uint32_t p = f.payload;
      uint8_t id = p >> 16;
      int16_t k16 = p & 0x0000FFFF;
      float value = OpenTherm::to_f88(k16);
      switch (cmd_id) {
      case CommandID::TEMPERATURE_REPORT:
        return on_temperature_report_cmd(f.user_data, id, value);
      case CommandID::SET_PID_SETPOINT:
        return on_set_pid_setpoint_cmd(f.user_data, id, value);
      case CommandID::SET_PID_KP:
        return on_set_pid_kp_cmd(f.user_data, id, value);
      case CommandID::SET_PID_KI:
        return on_set_pid_ki_cmd(f.user_data, id, value);
      case CommandID::SET_PID_KD:
        return on_set_pid_kd_cmd(f.user_data, id, value);
      case CommandID::SET_PID_CUM_ERR:
        return on_set_pid_cum_err_cmd(f.user_data, id, value);
      case CommandID::SET_PID_WEIGHT:
        return on_set_pid_weight_cmd(f.user_data, id, value);
      case CommandID::SET_PID_AUTOMATIC:
        return on_set_pid_automatic_cmd(f.user_data, id, value);
      default:
        return false;
      }
    }
    default:
      return Base::execute(f);
    }
  }

protected:
  PIDController<TimeType> pids[NUM_PIDS];
  float weights[NUM_PIDS];
  TimerType pid_timer;

  using Base::time;

  // Extrapolation<2> rta;
  LastValue<NUM_PIDS, TimeType> rta;
  // Average<5> rta;

  static bool on_pid_timer_cb(Timer *timer, void *obj) {
    return static_cast<PIDDrivenThermostat *>(obj)->on_pid_timer(timer);
  };

  bool on_pid_timer(Timer *) {
    if (!Base::is_manual()) {
      FlowSetpointUpdater u(*this, true);
      for (size_t i = 0; i < NUM_PIDS; i++)
        pids[i].update_unchanged();
    }
    return true;
  };

  float flow_setpoint_from_pid(size_t id) const {
    if (id >= NUM_PIDS)
      return 0.0f;
    return pids[id].output;
  }

  float flow_setpoint_from_pids_max() const {
    float r = 0.0f;
    for (size_t i = 0; i < NUM_PIDS; i++)
      r = fmax(r, pids[i].output);
    return r;
  }

  float flow_setpoint_from_pids_avg() const {
    float r = 0.0f;
    size_t n = NUM_PIDS;
    for (size_t i = 0; i < NUM_PIDS; i++) {
      auto &wi = weights[i];
      if (wi == 0.0f)
        n--;
      else
        r += wi * pids[i].output;
    }
    return r / (float)n;
  }
};

template <size_t NUM_DEMANDS, class TimeType, typename TimerType>
class DemandDrivenThermostat : public Thermostat<NUM_DEMANDS, TimeType, TimerType> {
#if __cplusplus >= 201103L && LITT_HAVE_STL
  static_assert(std::is_base_of<litt::Time, TimeType>::value, "TimeType must derive from litt::Time");
  static_assert(std::is_base_of<litt::Timer, TimerType>::value, "TimerType must derive from litt::Timer");
#endif
public:
  using Base = Thermostat<NUM_DEMANDS, TimeType, TimerType>;

  using typename Base::FlowSetpointUpdater;
  using typename Base::MixingFunction;
  using typename Base::Mode;

  class DemandUpdater : public FlowSetpointUpdater {
  public:
    DemandUpdater(DemandDrivenThermostat &t, bool approximated = false) : FlowSetpointUpdater(t, approximated), t(t) {
      before = t.mixed_demand();
    }

    virtual ~DemandUpdater() {
      float after = t.mixed_demand();
      if (after != before)
        t.on_mixed_demand_change(before, after);
    }

  protected:
    DemandDrivenThermostat &t;
    float before;
  };

  DemandDrivenThermostat(CentralHeatingInterface &chif) : Base(chif) {}

  virtual ~DemandDrivenThermostat() = default;

  virtual void on_mode_change(Mode from, Mode to) override {
    DemandUpdater u(*this);
    Base::on_mode_change(from, to);
  }

  // 0.0f <= d <= 1.0f
  virtual void on_mixed_demand_change(float from, float to) {}

  virtual float flow_setpoint() override {
    if (Base::is_automatic()) {
      const float max_flow_setpoint = Base::chif.max_flow_setpoint();
      const float wcsp = Base::weather_compensated_flow_setpoint();
      const float t = mixed_demand();
      const float range = max_flow_setpoint - wcsp;
      const float r = wcsp + (t * range);
      return fminf(r, max_flow_setpoint);
    } else
      return Base::flow_setpoint();
  }

  float get_demand(uint8_t id) const {
    if (id >= NUM_DEMANDS)
      return 0.0f;
    return demands[id].level;
  }

  virtual bool execute(const CommandFrame &f) override {
    switch (f.command_id) {
    case CommandID::DEMAND: {
      uint32_t p = f.payload;
      uint8_t id = p >> 16;
      uint16_t k16 = p & 0x0000FFFF;
      float value = OpenTherm::to_f88(k16);
      return on_demand_report_cmd(f.user_data, id, value);
    }
    default: {
      DemandUpdater u(*this);
      return Base::execute(f);
    }
    }
  }

  void delete_demander(uint8_t id) {
    if (id < max_id_seen) {
      for (size_t i = id; i < NUM_DEMANDS && i < max_id_seen; i++)
        demands[i] = demands[i + 1];
      demands[max_id_seen].level = 0.0f;
      demands[max_id_seen].time = 0;
      max_id_seen--;
    }
  }

protected:
  using Base::configuration;
  using Base::time;
  uint8_t max_id_seen = 0;

  struct Demand {
    float level = 0.0f;
    uint64_t time = 0;
  };

  Demand demands[NUM_DEMANDS];

  bool report_demand(uint8_t id, float demand) {
    if (id >= NUM_DEMANDS || isnan(demand) || demand < 0.0f || demand > 1.0f)
      return false;

    DemandUpdater du(*this);

    demands[id].level = demand;
    demands[id].time = time.get_us();

    if (id > max_id_seen)
      max_id_seen = id;

    return true;
  }

  virtual bool on_demand_report_cmd(uint16_t user_data, uint8_t id, float demand) { return report_demand(id, demand); }

  float mixed_demand() const {
    switch (configuration.mixing_function) {
    case Base::MixingFunction::AVERAGE: {
      double sum = 0.0;
      size_t n = 0;
      for (size_t i = 0; i < NUM_DEMANDS && i <= max_id_seen; i++) {
        const Demand &d = demands[i];
        if (d.time != 0 && !isnan(d.level)) {
          sum += d.level;
          n++;
        }
      }
      if (n == 0)
        return 0.0f;
      else {
        float r = sum / n;
        r = fmax(0.0f, fmin(r, 1.0f));
        return r;
      }
    }
    case Base::MixingFunction::MAX: {
      double r = 0.0;
      for (size_t i = 0; i < NUM_DEMANDS && i <= max_id_seen; i++) {
        const Demand &d = demands[i];
        if (d.time != 0 && !isnan(d.level))
          r = fmax(r, d.level);
      }
      r = fmax(0.0f, fmin(r, 1.0f));
      return r;
    }
    case Base::MixingFunction::FIRST:
    default:
      return fmax(0.0f, fmin(demands[0].level, 1.0f));
    }
  }
};

} // namespace litt

#endif // _LITT_THERMOSTAT_H_
