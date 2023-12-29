#ifndef _CH_INTERFACE_H_
#define _CH_INTERFACE_H_

namespace litt {

class CentralHeatingInterface {
public:
  CentralHeatingInterface() = default;
  virtual ~CentralHeatingInterface() = default;

  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual bool enabled() = 0;

  operator bool() { return enabled(); }

  virtual bool flame() const = 0;

  virtual float flow_setpoint() const = 0;
  virtual float flow_temperature() const = 0;
  virtual bool set_flow_setpoint(float temperature) = 0;
};

} // namespace litt

#endif // _CH_INTERFACE_H_
