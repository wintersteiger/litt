#ifndef _DHW_INTERFACE_H_
#define _DHW_INTERFACE_H_

namespace litt {

class DomesticHotWaterInterface {
public:
  DomesticHotWaterInterface() = default;
  virtual ~DomesticHotWaterInterface() = default;

  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual bool enabled() = 0;

  operator bool() { return enabled(); }
};

} // namespace litt

#endif // _DHW_INTERFACE_H_
