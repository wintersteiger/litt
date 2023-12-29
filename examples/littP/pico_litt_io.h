#ifndef _PICO_IO_H_
#define _PICO_IO_H_

#include <hardware/pio.h>

#include <litt/opentherm/transport.h>

#include "opentherm.pio.h"

typedef litt::Pins<unsigned> PicoPins;

class PicoIO : public litt::OpenTherm::IO<PicoPins> {
public:
  PicoIO(const PicoPins &pins) : litt::OpenTherm::IO<PicoPins>(pins) {

    sm_tx = pio_claim_unused_sm(pio, true);

    if (pins.tx_inverted) {
      uint offset_tx = pio_add_program(pio, &opentherm_tx_inv_program);
      opentherm_tx_inv_init(pio, sm_tx, offset_tx, pins.tx);
    } else {
      uint offset_tx = pio_add_program(pio, &opentherm_tx_program);
      opentherm_tx_init(pio, sm_tx, offset_tx, pins.tx);
    }

    sm_rx = pio_claim_unused_sm(pio, true);
    uint offset_rx = pio_add_program(pio, &opentherm_rx_program);
    opentherm_rx_init(pio, sm_rx, offset_rx, pins.rx);
  }

  virtual ~PicoIO() = default;

  virtual void put(const litt::OpenTherm::Frame &f) override {
    pio_sm_put(pio0, sm_tx, (uint32_t)f);
  }

  using IO::put;

  virtual litt::OpenTherm::Frame get_blocking() override {
    return pio_sm_get_blocking(pio, sm_rx);
  }

  virtual bool get_timeout(litt::OpenTherm::Frame &f, uint64_t timeout_us) override {
    // return pio_sm_get_blocking(pio, sm_rx);

    auto deadline = time_us_64() + timeout_us;
    do {
      auto l = pio_sm_get_rx_fifo_level(pio, sm_rx);
      if (l > 0) {
        auto r = pio_sm_get(pio, sm_rx);
        f = r;
        return true;
      }
      sleep_us(1000);
    }
    while(time_us_64() <= deadline);
    return false;
  }

  virtual void log(const char *fmt, ...) override {
    va_list args;
    va_start(args, fmt);
    ::vllog(fmt, args);
    va_end(args);
  }

protected:
  PIO pio = pio0;
  unsigned sm_tx, sm_rx;

  float clk_div() const { return pio0->sm[sm_tx].clkdiv / 1e6; }
};

#endif // _PICO_IO_H_
