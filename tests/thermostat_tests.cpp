#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <litt/thermostat.h>

#include "test_ds.h"

using namespace litt;

static TestTime tt;

typedef litt::Thermostat<1, TestTime, TestTimer> MyThermostat;

class TestApp : public MyThermostat {
public:
  TestApp() : MyThermostat(configuration, chif, MyThermostat::MixingFunction::FIRST) {
    set_mode(MyThermostat::Mode::OFF);
  }

  ~TestApp() = default;

  Configuration configuration = {.min_flow_setpoint = 30.0f};
  TestCHIF chif;

  virtual void on_flow_setpoint_change(float from, float to, bool approximated) override {
    std::cout << "flow setpoint: " << from << " -> " << to << std::endl;
    MyThermostat::on_flow_setpoint_change(from, to, approximated);
  }

  virtual void on_flow_temperature_change(float from, float to) override {
    std::cout << "flow temperature: " << from << " -> " << to << std::endl;
    MyThermostat::on_flow_temperature_change(from, to);
  }
};

TEST_CASE("Initial state") {
  TestApp a;
  REQUIRE(a.chif.enabled() == false);
  REQUIRE(a.get_mode() == MyThermostat::Mode::OFF);
}

TEST_CASE("Basic mode change") {
  TestApp a;
  REQUIRE(a.chif.enabled() == false);
  a.set_mode(MyThermostat::Mode::AUTOMATIC);
  REQUIRE(a.get_mode() == MyThermostat::Mode::AUTOMATIC);
}

TEST_CASE("Initial flow setpoint") {
  TestApp a;
  REQUIRE(a.flow_setpoint() >= a.configuration.min_flow_setpoint);
}

TEST_CASE("Flow setpoint change") {
  TestApp a;
  a.set_mode(MyThermostat::Mode::AUTOMATIC);
  REQUIRE(a.flow_setpoint() >= a.configuration.min_flow_setpoint);
  REQUIRE(a.execute(CommandFrame(CommandID::SET_FLOW_SETPOINT, 0, 80.0f)));
  REQUIRE(a.flow_setpoint() == 80.0f);
  REQUIRE(a.get_mode() == MyThermostat::Mode::MANUAL);
  REQUIRE(a.execute(CommandFrame(CommandID::SET_FLOW_SETPOINT, 0, 30.0f)));
  REQUIRE(a.flow_setpoint() == 30.0f);
  REQUIRE(a.get_mode() == MyThermostat::Mode::MANUAL);
}

TEST_CASE("Basic flow temperature change") {
  TestApp a;
  a.set_mode(MyThermostat::Mode::AUTOMATIC);
  REQUIRE(a.get_mode() == MyThermostat::Mode::AUTOMATIC);
  REQUIRE(a.chif.flow_setpoint() >= a.configuration.min_flow_setpoint);
  a.chif.set_flow_temperature(40.0f);
  a.on_flow_temperature_change(24.0f, 40.0f);
  REQUIRE(a.chif.flow_temperature() == 40.0f);
}

TEST_CASE("Mode auto -> off") {
  TestApp a;
  a.set_mode(MyThermostat::Mode::AUTOMATIC);
  REQUIRE(a.get_mode() == MyThermostat::Mode::AUTOMATIC);
  REQUIRE(a.chif.flow_setpoint() >= a.configuration.min_flow_setpoint);
  a.set_mode(MyThermostat::Mode::OFF);
  REQUIRE(a.chif.flow_setpoint() == a.configuration.min_flow_setpoint);
}

TEST_CASE("Mode off -> auto") {
  TestApp a;
  REQUIRE(a.get_mode() == MyThermostat::Mode::OFF);
  REQUIRE(a.chif.flow_setpoint() == a.configuration.min_flow_setpoint);
  REQUIRE(!a.chif.enabled());

  a.set_max_flow_setpoint_bounds(30.0f, 80.0f);
  a.chif.set_flow_temperature(40.0f);
  REQUIRE(a.execute(CommandFrame(CommandID::SET_FLOW_SETPOINT, 0, 80.0f)));

  REQUIRE(a.flow_setpoint() == 80.0f);
  a.set_mode(MyThermostat::Mode::AUTOMATIC);
  REQUIRE(a.chif.flow_setpoint() == 80.0f);
}

typedef litt::DemandDrivenThermostat<1, TestTime, TestTimer> MyDemandThermostat;

class DemandTestApp : public MyDemandThermostat {
public:
  DemandTestApp() : MyDemandThermostat(configuration, chif, MyThermostat::MixingFunction::FIRST) {
    set_mode(MyThermostat::Mode::OFF);
  }

  ~DemandTestApp() = default;

  Configuration configuration = {.min_flow_setpoint = 30.0f};
  TestCHIF chif;

  virtual void on_flow_setpoint_change(float from, float to, bool approximated) override {
    std::cout << "flow setpoint: " << from << " -> " << to << std::endl;
    MyThermostat::on_flow_setpoint_change(from, to, approximated);
  }

  virtual void on_flow_temperature_change(float from, float to) override {
    std::cout << "flow temperature: " << from << " -> " << to << std::endl;
    MyThermostat::on_flow_temperature_change(from, to);
  }
};

TEST_CASE("Mode off -> auto (demand driven)") {
  DemandTestApp a;
  REQUIRE(a.get_mode() == MyDemandThermostat::Mode::OFF);
  REQUIRE(a.chif.flow_setpoint() == a.configuration.min_flow_setpoint);
  REQUIRE(!a.chif.enabled());

  a.set_max_flow_setpoint_bounds(30.0f, 80.0f);
  a.chif.set_flow_temperature(40.0f);

  REQUIRE(a.execute(CommandFrame(CommandID::DEMAND, 0, 0, 1.0f)));
  REQUIRE(a.chif.flow_setpoint() == a.configuration.min_flow_setpoint);
  REQUIRE(!a.chif.enabled());

  a.set_mode(MyDemandThermostat::Mode::AUTOMATIC);
  a.chif.set_flow_temperature(42.0f);
  REQUIRE(a.chif.flow_setpoint() == 80.0f);
  REQUIRE(a.chif.enabled());
}

TEST_CASE("Trigger spike protection (demand driven)") {
  DemandTestApp a;
  REQUIRE(a.get_mode() == MyDemandThermostat::Mode::OFF);
  REQUIRE(a.chif.flow_setpoint() == a.configuration.min_flow_setpoint);
  REQUIRE(!a.chif.enabled());

  a.set_max_flow_setpoint_bounds(30.0f, 80.0f);
  a.chif.set_flow_temperature(38.0f);

  REQUIRE(a.execute(CommandFrame(CommandID::DEMAND, 0, 0, 0.5f)));
  REQUIRE(a.chif.flow_setpoint() == a.configuration.min_flow_setpoint);
  REQUIRE(!a.chif.enabled());

  a.set_mode(MyDemandThermostat::Mode::AUTOMATIC);
  a.chif.set_flow_temperature(39.0f);
  a.on_flow_temperature_change(38.0f, 39.0f);
  REQUIRE(a.is_spike_protect_active());
  REQUIRE(a.chif.flow_setpoint() == 40.0f);
  REQUIRE(!a.chif.enabled());

  REQUIRE(a.execute(CommandFrame(CommandID::DEMAND, 0, 0, 1.0f)));
  REQUIRE(a.chif.flow_setpoint() == 80.0f);
  REQUIRE(a.chif.enabled());
}
