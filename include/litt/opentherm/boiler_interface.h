#ifndef _LITT_OPENTHERM_BOILER_INTERFACE_H_
#define _LITT_OPENTHERM_BOILER_INTERFACE_H_

#include <math.h>

#include "application.h"
#include "transport.h"

#include <litt/ch_interface.h>
#include <litt/dhw_interface.h>

namespace litt {
namespace OpenTherm {

template <typename TransportType> class BoilerInterface {
public:
  BoilerInterface(TransportType &transport, Application &app)
      : ch1(true, transport, app), ch2(false, transport, app), dhw(transport), transport(transport) {}

  virtual ~BoilerInterface() = default;

  class CHIF : public litt::CentralHeatingInterface {
  public:
    CHIF(bool first, TransportType &transport, Application &app) : first(first), transport(transport), app(app) {}
    virtual ~CHIF() = default;

    virtual void enable() override {
      if (first)
        transport.set_ch(true);
      else
        transport.set_ch2(true);
    }

    virtual void disable() override {
      if (first)
        transport.set_ch(false);
      else
        transport.set_ch2(false);
    }

    virtual bool enabled() override { return first ? transport.ch_enabled() : transport.ch2_enabled(); }

    virtual bool flame() const override {
      auto id = app.find(0);
      return id ? id->value & 0x08 : false;
    }

    virtual bool set_flow_setpoint(float temperature) override {
      if (isnan(temperature) || isinf(temperature))
        return false;

      if (temperature > 80.0f) {
        // Just in case, we consider a flow setpoint > 80.0C too high.
        temperature = 80.0f;
      }

      return transport.tx(Frame(WriteData, 1, temperature)) != NoRequestID;
    }

    virtual float flow_setpoint() const override {
      auto id = app.find(1);
      return id ? to_f88(id->value) : 0.0f;
    }

    virtual float flow_temperature() const override {
      auto id = app.find(first ? 25 : 31);
      return id ? to_f88(id->value) : 0.0f;
    }

  protected:
    bool first;
    TransportType &transport;
    Application &app;
  };

  class DHWIF : public litt::DomesticHotWaterInterface {
  public:
    DHWIF(TransportType &transport) : transport(transport) {}
    virtual ~DHWIF() = default;

    virtual void enable() override { transport.set_dhw(true); }
    virtual void disable() override { transport.set_dhw(false); }
    virtual bool enabled() override { return transport.dhw_enabled(); }

  protected:
    TransportType &transport;
  };

  virtual void otc_enable() const {
    if (!transport.otc_active())
      transport.set_otc(true);
  }

  virtual void otc_disable() const { transport.set_otc(false); }

  virtual bool otc_enabled() const { return transport.otc_active(); }

  CHIF ch1;
  CHIF ch2;
  DHWIF dhw;

protected:
  TransportType &transport;
};

} // namespace OpenTherm
} // namespace litt

#endif // _LITT_OPENTHERM_BOILER_INTERFACE_H_
