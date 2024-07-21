#ifndef _LITT_OPENTHERM_APPLICATION_H_
#define _LITT_OPENTHERM_APPLICATION_H_

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "transport.h"

// OpenTherm 2.2 application layer

namespace litt {
namespace OpenTherm {

template <typename T> static float to_f88(const T &v) { return v / 256.0f; }

static uint16_t from_f88(float f) { return f * 256.0f; }

enum RWSpec : uint8_t { ReadOnly, WriteOnly, ReadWrite };

enum IDType : uint8_t { flag8_flag8, F88, flag8_u8, u8_u8, s8_s8, u16, s16, special_u8, flag8_ };

struct DataID;

class IDIndex {
public:
  IDIndex() = default;
  virtual ~IDIndex() = default;

  virtual void register_id(DataID *id) = 0;

  virtual const DataID *find(uint8_t id) const = 0;
  virtual DataID *find(uint8_t id) = 0;
};

struct DataID {
  DataID(uint8_t nr, RWSpec, const char *, IDType, const char *, IDIndex *index, OpenTherm::TransportBase *t = nullptr,
         uint16_t default_value = 0)
      : nr(nr), value(default_value) {
    if (index)
      index->register_id(this);
  }

  const uint8_t nr;
  uint16_t value = 0;

  DataID &operator=(int v) { return operator=(static_cast<uint16_t>(v)); }

  DataID &operator=(uint16_t v) {
    value = v;
    return *this;
  }

  DataID &operator=(float v) {
    using namespace OpenTherm;
    value = from_f88(v);
    return *this;
  }
};

struct DataIDWithMeta : public DataID {
  DataIDWithMeta(uint8_t nr, RWSpec msg, const char *data_object, IDType type, const char *description, IDIndex *index,
                 OpenTherm::TransportBase *t = nullptr, uint16_t default_value = 0)
      : DataID(nr, msg, data_object, type, description, index, t, default_value), msg(msg), data_object(data_object),
        type(type), description(description) {}

  DataIDWithMeta &operator=(int v) { return static_cast<DataIDWithMeta &>(DataID::operator=(v)); }

  DataIDWithMeta &operator=(uint16_t v) { return static_cast<DataIDWithMeta &>(DataID::operator=(v)); }

  DataIDWithMeta &operator=(float v) { return static_cast<DataIDWithMeta &>(DataID::operator=(v)); }

  RWSpec msg;
  const char *data_object;
  IDType type;
  const char *description;

  operator const char *() const {
    using namespace OpenTherm;

    static char buf[128];
    switch (type) {
    case flag8_flag8:
      snprintf(buf, sizeof(buf), "%02x/%02x", value >> 8, value & 0x00FF);
      break;
    case F88: {
      float f = to_f88(value);
      snprintf(buf, sizeof(buf), "%.2f", f);
      break;
    }
    case flag8_u8:
      snprintf(buf, sizeof(buf), "%02x/%u", value >> 8, value & 0x00FF);
      break;
    case u8_u8:
      snprintf(buf, sizeof(buf), "%u/%u", value >> 8, value & 0x00FF);
      break;
    case s8_s8:
      snprintf(buf, sizeof(buf), "%d/%d", (int8_t)(value >> 8), (int8_t)value);
      break;
    case u16:
      snprintf(buf, sizeof(buf), "%u", value);
      break;
    case s16:
      snprintf(buf, sizeof(buf), "%d", (int16_t)value);
      break;
    case special_u8:
      snprintf(buf, sizeof(buf), "XXX/%u", value & 0x00FF);
      break;
    case flag8_:
      snprintf(buf, sizeof(buf), "%02x", value >> 8);
      break;
    default:
      snprintf(buf, sizeof(buf), "unknown type");
      break;
    }
    return buf;
  }
};

template <typename DataIDType, typename IndexType> class DataIDs : public IndexType {
public:
  using DID = DataIDType;

  DataIDs() : IndexType() {}

  virtual ~DataIDs() = default;

  // Ids 0 .. 127 are reserved for OpenTherm pre-defined information, while id’s from 128 .. 255 can be used by
  // manufacturers (members of the association) for test & diagnostic purposes only.

  // TODO: initial values?

  // clang-format off
  DID status = DID(0, ReadOnly, "Status", flag8_flag8, "Master and Slave Status flags", this);
  DID tset = DID(1, WriteOnly, "T_set", F88, "Control setpoint, i.e. CH water temperature setpoint (\xF8""C)", this);
  DID mconfig_mmemberid = DID(2, WriteOnly, "M-Config / M-MemberIDcode", flag8_u8, "Master Configuration Flags / Master MemberID Code", this);
  DID sconfig_smemberid = DID(3, ReadOnly, "S-Config / S-MemberIDcode", flag8_u8, "Slave Configuration Flags / Slave MemberID Code", this);
  DID command = DID(4, WriteOnly, "Command", u8_u8, "Remote Command", this);
  DID asf_flags = DID(5, ReadOnly, "ASF-flags / OEM-fault-code", flag8_u8, "Application-specific fault flags and OEM fault code", this);
  DID rbp_flags = DID(6, ReadOnly, "RBP-flags", flag8_flag8, "Remote boiler parameter transfer-enable & read/write flags", this);
  DID cooling_protocol = DID(7, WriteOnly, "Cooling-control", F88, "Cooling control signal (%)", this);
  DID tsetch2 = DID(8, WriteOnly, "T_setCH2", F88, "Control setpoint for 2nd CH circuit (\xF8""C)", this);
  DID troverride = DID(9, ReadOnly, "T_rOverride", F88, "Remote override room setpoint", this);
  DID tsp = DID(10, ReadOnly, "# TSP", u8_u8, "Number of Transparent-Slave-Parameters supported by slave", this);
  DID tsp_index_value = DID(11, ReadWrite, "TSP-index / TSP-value", u8_u8, "Index number / Value of referred-to transparent slave parameter", this);
  DID fhb_size = DID(12, ReadOnly, "FHB-size", u8_u8, "Size of Fault-History-Buffer supported by slave", this);
  DID fhb_index_value = DID(13, ReadOnly, "FHB-index / FHB-value", u8_u8, "Index number / Value of referred-to fault-history buffer entry", this);
  DID max_rel_mod_level_setting = DID(14, WriteOnly, "Max-rel-mod-level-setting", F88, "Maximum relative modulation level setting (%)", this);
  DID max_capacity_min_mod_level = DID(15, ReadOnly, "Max-Capacity / Min-Mod-Level", u8_u8, "Maximum boiler capacity (kW) / Minimum boiler modulation level (%)", this);
  DID trset = DID(16, WriteOnly, "Tr_set", F88, "Room Setpoint (\xF8""C)", this);
  DID rel_mod_level = DID(17, ReadOnly, "Rel.-mod-level", F88, "Relative Modulation Level (%)", this);
  DID ch_pressure = DID(18, ReadOnly, "CH-pressure", F88, "Water pressure in CH circuit (bar)", this);
  DID dhw_flow_rate = DID(19, ReadOnly, "DHW-flow-rate", F88, "Water flow rate in DHW circuit (litres/minute)", this);
  DID day_time = DID(20, ReadWrite, "Day-Time", special_u8, "Day of Week and Time of Day", this);
  DID date = DID(21, ReadWrite, "Date", u8_u8, "Calendar date", this);
  DID year = DID(22, ReadWrite, "Year", u16, "Calendar year", this);
  DID trsetch2 = DID(23, WriteOnly, "TrsetCH2", F88, "Room Setpoint for 2nd CH circuit (\xF8""C)", this);
  DID tr = DID(24, WriteOnly, "T_r", F88, "Room temperature (\xF8""C)", this);
  DID tboiler = DID(25, ReadOnly, "T_boiler", F88, "Boiler flow water temperature (\xF8""C)", this);
  DID tdhw = DID(26, ReadOnly, "Tdhw", F88, "DHW temperature (\xF8""C)", this);
  DID toutside = DID(27, ReadOnly, "T_outside", F88, "Outside temperature (\xF8""C)", this);
  DID tret = DID(28, ReadOnly, "T_ret", F88, "Return water temperature (\xF8""C)", this);
  DID tstorage = DID(29, ReadOnly, "T_storage", F88, "Solar storage temperature (\xF8""C)", this);
  DID tcollector = DID(30, ReadOnly, "T_collector", F88, "Solar collector temperature (\xF8""C)", this);
  DID tflowch2 = DID(31, ReadOnly, "TflowCH2", F88, "Flow water temperature CH2 circuit (\xF8""C)", this);
  DID tdhw2 = DID(32, ReadOnly, "Tdhw2", F88, "Domestic hot water temperature 2 (\xF8""C)", this);
  DID texhaust = DID(33, ReadOnly, "Texhaust", s16, "Boiler exhaust temperature (\xF8""C)", this);
  DID tdhwset_ub_lb = DID(48, ReadOnly, "T_dhwSet UB/LB", s8_s8, "DHW setpoint upper & lower bounds for adjustment (\xF8""C)", this);
  DID max_tset_ub_lb = DID(49, ReadOnly, "MaxT_set UB/LB", s8_s8, "Max CH water setpoint upper & lower bounds for adjustment (\xF8""C)", this, nullptr, 0xFFFF);
  DID hcratio_ub_lb = DID(50, ReadOnly , "Hcratio UB/LB", s8_s8, "OTC heat curve ratio upper & lower bounds for adjustment", this);
  DID tdhwset = DID(56, ReadWrite, "T_dhwSet", F88, "DHW setpoint (\xF8""C) (Remote parameter 1)", this);
  DID maxtset = DID(57, ReadWrite, "MaxT_set", F88, "Max CH water setpoint (\xF8""C) (Remote parameters 2)", this, nullptr, 100);
  DID hcratio = DID(58, ReadWrite, "Hcratio", F88, "OTC heat curve ratio (\xF8""C) (Remote parameter 3)", this);
  DID remote_override = DID(100, ReadOnly, "Remote override function", flag8_, "Function of manual and program changes in master and remote room setpoint", this);
  DID oem_diagnostic_code = DID(115, ReadOnly, "OEM diagnostic code", u16, "OEM-specific diagnostic/service code", this);
  DID burner_starts = DID(116, ReadWrite, "Burner starts", u16, "Number of starts burner", this);
  DID ch_pump_starts = DID(117, ReadWrite, "CH pump starts", u16, "Number of starts CH pump", this);
  DID dhw_pump_valve_starts = DID(118, ReadWrite, "DHW pump/valve starts", u16, "Number of starts DHW pump/valve", this);
  DID dhw_burner_starts = DID(119, ReadWrite, "DHW burner starts", u16, "Number of starts burner during DHW mode", this);
  DID burner_ophours = DID(120, ReadWrite, "Burner operation hours", u16, "Number of hours that burner is in operation (i.e. flame on)", this);
  DID ch_pump_ophours = DID(121, ReadWrite, "CH pump operation hours", u16, "Number of hours that CH pump has been running", this);
  DID dhw_pump_valve_ophours = DID(122, ReadWrite, "DHW pump/valve operation hours", u16, "Number of hours that DHW pump has been running or DHW valve has been opened", this);
  DID dhw_burner_ophours = DID(123, ReadWrite, "DHW burner operation hours", u16, "Number of hours that burner is in operation during DHW mode", this);
  DID ot_version_master = DID(124, WriteOnly , "OpenTherm version Master", F88, "The implemented version of the OpenTherm Protocol Specification in the master", this);
  DID ot_version_slave = DID(125, ReadOnly, "OpenTherm version Slave", F88, "The implemented version of the OpenTherm Protocol Specification in the slave", this);
  DID master_version = DID(126, WriteOnly, "Master-version", u8_u8, "Master product version number and type", this);
  DID slave_version = DID(127, ReadOnly, "Slave-version", u8_u8, "Slave product version number and type", this);

  // Not in 2.2:
  DID theatex = DID(34, ReadOnly, "T_heatex", F88, "Boiler heat exchanger temperature (\xF8""C)", this);
  DID fan_speed = DID(35, ReadWrite, "Fan speed", u8_u8, "Boiler fan speed setpoint and actual value (%)", this);
  DID burner_flame_current = DID(36, ReadOnly, "Burner flame current", s16, "Electrical current through burner flame (\xE6""A)", this);
  DID tr2 = DID(37, WriteOnly, "T_r2", F88, "Room temperature for 2nd CH circuit (\xF8""C)", this);
  DID humidity = DID(38, ReadWrite, "Humidity", s16, "Relative Humidity (%)", this);
  DID operating_mode = DID(99, ReadWrite, "Operating mode", u16, "Operating mode HC1/HC2/DHW", this);
  DID burner_nonstarts = DID(113, ReadWrite, "Burner non-starts", u16, "Number of unsuccessful burner starts", this);
  DID flame_low = DID(114, ReadWrite, "Flame low", u16, "Number of times flame signal was too low", this);
  // clang-format on
};

class Application {
public:
  Application(OpenTherm::TransportBase &transport) : transport(transport) {}

  virtual ~Application() = default;

  virtual void run() = 0;

  virtual DataID *find(uint8_t id) = 0;

  virtual const DataID *find(uint8_t id) const = 0;

  virtual bool process(const Frame &f) {
    switch (f.msg_type()) {
    case ReadData:
      return on_read(f.id(), f.value());
    case WriteData:
      return on_write(f.id(), f.value());
    case InvalidData:
      return on_invalid_data(f.id(), f.value());
    case ReadACK:
      on_read_ack(f.id(), f.value());
      return true;
    case WriteACK:
      on_write_ack(f.id(), f.value());
      return true;
    case DataInvalid:
      on_data_invalid(f.id(), f.value());
      return true;
    case UnknownDataID:
      on_unknown_data_id(f.id(), f.value());
      return true;
    }

    return false;
  }

protected:
  OpenTherm::TransportBase &transport;

  // Master to Slave
  virtual bool on_read(uint8_t data_id, uint16_t data_value = 0x0000) {
    const DataID *id = find(data_id);
    if (id != NULL) {
      // return DATA-INVALID(DATA-ID, DATA-VALUE) if the data ID is recognised
      // but the data requested is not available or invalid. DATA-VALUE can be
      // 0x0000 in this case.
      return transport.tx(Frame(ReadACK, data_id, id->value)) != NoRequestID;
    } else
      return transport.tx(Frame(UnknownDataID, data_id, data_value)) != NoRequestID;
  }

  virtual bool on_write(uint8_t data_id, uint16_t data_value) {
    DataID *id = find(data_id);
    if (id != NULL) {
      id->value = data_value;
      return transport.tx(Frame(WriteACK, data_id, data_value)) != NoRequestID;
    } else
      return transport.tx(Frame(UnknownDataID, data_id, data_value)) != NoRequestID;
  }

  virtual bool on_invalid_data(uint8_t data_id, uint16_t data_value) {
    const DataID *id = find(data_id);
    if (id != NULL)
      return transport.tx(Frame(DataInvalid, data_id, data_value)) != NoRequestID;
    else
      return transport.tx(Frame(UnknownDataID, data_id, data_value)) != NoRequestID;
  }

  // Slave to Master
  virtual void on_read_ack(uint8_t data_id, uint16_t data_value) {
    DataID *id = find(data_id);
    if (id)
      id->value = data_value;
  }

  virtual void on_write_ack(uint8_t data_id, uint16_t data_value) {
    DataID *id = find(data_id);
    if (id)
      id->value = data_value;
  }

  virtual void on_data_invalid(uint8_t data_id, uint16_t data_value) {}

  virtual void on_unknown_data_id(uint8_t data_id, uint16_t data_value) {}

  static bool sprocess(Application &app, const Frame &f) { return app.process(f); }
};

class SmallIDIndex : public IDIndex {
protected:
  static constexpr const size_t max_entries = 61;
  DataID *index[max_entries] = {0};
  size_t num_entries = 0;

public:
  SmallIDIndex() = default;

  virtual ~SmallIDIndex() = default;

  virtual void register_id(DataID *id) override {
    if (num_entries >= max_entries)
      return;
    index[num_entries] = id;
    num_entries++;
  }

  virtual const DataID *find(uint8_t id) const override {
    for (const auto d : index)
      if (d && d->nr == id)
        return d;
    return nullptr;
  }

  virtual DataID *find(uint8_t id) override {
    for (auto d : index)
      if (d && d->nr == id)
        return d;
    return nullptr;
  }
};

using SmallIDs = DataIDs<DataID, SmallIDIndex>;

class SmallApplication : public Application, public SmallIDs {
public:
  SmallApplication(OpenTherm::TransportBase &transport) : Application(transport), SmallIDs() {}

  virtual ~SmallApplication() = default;

  virtual void run() override = 0;

  virtual DataID *find(uint8_t id) override { return SmallIDs::find(id); }

  virtual const DataID *find(uint8_t id) const override { return SmallIDs::find(id); }
};

class RichIDIndex : public IDIndex {
protected:
  DataIDWithMeta *index[256] = {0};

public:
  RichIDIndex() = default;

  virtual ~RichIDIndex() = default;

  virtual void register_id(DataID *id) override { index[id->nr] = static_cast<DataIDWithMeta *>(id); }

  virtual DataIDWithMeta *find(uint8_t id) override { return index[id]; }

  virtual const DataIDWithMeta *find(uint8_t id) const override { return index[id]; }
};

using RichIDs = DataIDs<DataIDWithMeta, RichIDIndex>;

class RichApplication : public Application, public DataIDs<DataIDWithMeta, RichIDIndex> {
public:
  /// @brief Constructor
  /// @param transport a transport layer implementation
  RichApplication(OpenTherm::TransportBase &transport) : Application(transport), RichIDs() {}

  /// @brief Destructor.
  virtual ~RichApplication() = default;

  /// @brief Main entry point for an OpenTherm (rich) application.
  virtual void run() override = 0;

  /// @brief Called when a fault is indicated.
  virtual void on_fault_indication() = 0;

  /// @brief Called when a diagnostic event is indicated.
  virtual void on_diagnostic_indication() {}

  /// @brief Called when the flame changes.
  /// @param on true if the flame is now on, false otherwise
  virtual void on_flame_change(bool on) {}

  /// @brief Find a data ID.
  /// @param nr number of the data ID to find
  /// @return pointer to the corresponding DataIDWithMeta object, or nullptr if not found
  virtual DataIDWithMeta *find(uint8_t nr) override { return RichIDs::find(nr); }

  /// @brief Find a data ID.
  /// @param nr number of the data ID to find
  /// @return pointer to the corresponding DataIDWithMeta object, or nullptr if not found
  virtual const DataIDWithMeta *find(uint8_t nr) const override { return RichIDs::find(nr); }

  struct FlowSetpointBounds {
    int8_t lower_bound;
    int8_t upper_bound;
  };

  /// @brief Called when a change in master status is detected.
  /// @param from previous status
  /// @param to new status
  virtual void on_master_status_change(uint8_t from, uint8_t to) {}

  /// @brief Called when a change in slave status is detected.
  /// @param from previous status
  /// @param to new status
  virtual void on_slave_status_change(uint8_t from, uint8_t to) {}

  /// @brief Called when a change in maximum flow setpoint bounds is detected.
  /// @param from previous bounds
  /// @param to new bounds
  virtual void on_max_flow_setpoint_bounds_change(const FlowSetpointBounds &from, const FlowSetpointBounds &to) {}

  /// @brief Called when the maximum flow setpoint changes.
  /// @param from previous temperature
  /// @param to new temperature
  virtual void on_max_flow_setpoint_change(float from, float to) {}

  /// @brief Called when a change in flow temperature is detected.
  /// @param from previous flow temperature
  /// @param to new flow temperature
  virtual void on_flow_temperature_change(float from, float to) {}

  /// @brief Called when a change in return temperature is detected.
  /// @param from previous return temperature
  /// @param to new return temperature
  virtual void on_return_temperature_change(float from, float to) {}

  /// @brief Called when a change in outside temperature is detected.
  /// @param from previous outside temperature
  /// @param to new outside temperature
  virtual void on_outside_air_temperature_change(float from, float to) {}

  virtual void on_read_ack(uint8_t data_id, uint16_t data_value) override {
    using namespace OpenTherm;

    auto before = find(data_id);
    uint16_t before_value = before->value;
    bool changed = before && before_value != data_value;

    switch (data_id) {
    case 0:
      if (changed) {
        if ((data_value & 0x0001) != 0)
          on_fault_indication();

        if ((data_value & 0x0040) != 0)
          on_diagnostic_indication();

        uint8_t master_before = before_value >> 8;
        uint8_t master_after = data_value >> 8;
        if (master_before != master_after)
          on_master_status_change(master_before, master_after);

        uint8_t slave_before = before_value & 0x00FF;
        uint8_t slave_after = data_value & 0x00FF;
        if (slave_before != slave_after)
          on_slave_status_change(slave_before, slave_after);

        bool flame_was = before_value & 0x08;
        bool flame_is = data_value & 0x08;
        if (flame_was != flame_is)
          on_flame_change(flame_is);
      }
      break;
    case 49:
      if (changed) {
        int8_t ub_before = before_value >> 8;
        int8_t lb_before = before_value & 0xFF;
        int8_t ub = data_value >> 8;
        int8_t lb = data_value & 0xFF;
        on_max_flow_setpoint_bounds_change({lb_before, ub_before}, {lb, ub});
      }
      break;
    case 25:
      if (changed)
        on_flow_temperature_change(to_f88(before_value), to_f88(data_value));
      break;
    case 27:
      if (changed)
        on_outside_air_temperature_change(to_f88(before_value), to_f88(data_value));
      break;
    case 28:
      if (changed)
        on_return_temperature_change(to_f88(before_value), to_f88(data_value));
      break;
    case 57:
      if (changed)
        on_max_flow_setpoint_change(to_f88(before_value), to_f88(data_value));
      break;
    default:;
    }

    Application::on_read_ack(data_id, data_value);
  }
};

} // namespace OpenTherm
} // namespace litt

#endif // _LITT_OPENTHERM_APPLICATION_H_
