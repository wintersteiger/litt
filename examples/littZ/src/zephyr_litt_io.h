#ifndef _ZEPHYR_LITT_IO_H_
#define _ZEPHYR_LITT_IO_H_

#include <zephyr/drivers/gpio.h>

#include <litt/opentherm/transport.h>

typedef litt::Pins<struct gpio_dt_spec> ZephyrPins;

class ZephyrIO : public litt::OpenTherm::IO<ZephyrPins> {
public:
  ZephyrIO(const ZephyrPins &pins_)
      : litt::OpenTherm::IO<ZephyrPins>(pins_), tx_timer(500, 500, tx_timer_ftick, tx_timer_fstop, this), in_queue(16),
        rx_callback_info({.io = this}) {
    const ZephyrPins &p = pins_;

    if (!device_is_ready(p.rx.port))
      LOG_ERR("X RX port not ready");
    else if (!device_is_ready(p.tx.port))
      LOG_ERR("X TX port not ready");

    int err;
    if ((err = gpio_pin_configure_dt(&p.rx, GPIO_INPUT)) < 0)
      LOG_ERR("X Error %d: RX gpio_pin_configure failed.", err);
    if ((err = gpio_pin_configure_dt(&p.tx, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW)) < 0)
      LOG_ERR("X Error %d: TX gpio_pin_configure failed.", err);
    if ((err = gpio_pin_set_dt(&p.tx, p.tx_inverted ? 0 : 1)) < 0)
      LOG_ERR("X Error %d: TX gpio_pin_set_dt failed.", err);

    if (p.owned) {
      gpio_init_callback(&rx_callback_info.data, rx_callback, BIT(p.rx.pin));
      if ((err = gpio_add_callback(p.rx.port, &rx_callback_info.data)) < 0)
        LOG_ERR("X Error %d: gpio_add_callback failed.", err);
    }
  }

  virtual ~ZephyrIO() = default;

  virtual void put(const litt::OpenTherm::Frame &f) override {
    bits_to_send = (1ull << 63) | ((uint64_t)f) << 31 | (1ull << 30);
    num_transitions_remaining = 68;
    tx_timer.start();
  }

  using IO::put;

  virtual litt::OpenTherm::Frame get_blocking() override { return in_queue.remove(); }

  virtual bool get_timeout(litt::OpenTherm::Frame &f, uint64_t timeout_us) override {
    return in_queue.remove_timeout(f, timeout_us);
  }

  virtual void log(const char *fmt, ...) override {
    va_list args;
    va_start(args, fmt);
    log2_generic(LOG_LEVEL_WRN, fmt, args);
    va_end(args);
  }

  // Without pin ownership, we rely on other ISRs to call isr().
  void isr() {
    if (!pins.owned)
      rx_callback();
  }

  void cease() { tx_timer.stop(true); }

protected:
  ZephyrTimer tx_timer;

  uint64_t bits_to_send = 0;
  uint8_t num_transitions_remaining = 0;

  static bool tx_timer_ftick(litt::Timer *timer, void *iop) {
    auto io = static_cast<ZephyrIO *>(iop);
    return io->tx_timer_ftick();
  }

  bool tx_timer_ftick() {
    if (num_transitions_remaining == 0) {
      tx_timer.stop();
      gpio_pin_set_dt(&pins.tx, pins.tx_inverted ? 1 : 0);
      rx.state = IDLE;
      if (pins.owned)
        gpio_pin_interrupt_configure_dt(&pins.rx, GPIO_INT_EDGE_BOTH);
    } else {
      bool first_half = (num_transitions_remaining % 2) == 0;
      bool bit = (bits_to_send & 0x8000000000000000) != 0;
      if (pins.tx_inverted)
        bit = !bit;
      gpio_pin_set_dt(&pins.tx, first_half ? bit : !bit);

      num_transitions_remaining--;
      if (!first_half)
        bits_to_send = bits_to_send << 1;
    }
    return true;
  }

  static bool tx_timer_fstop(litt::Timer *timer, void *iop) {
    auto io = static_cast<ZephyrIO *>(iop);
    return io->tx_timer_fstop();
  }

  bool tx_timer_fstop() {
    bits_to_send = 0;
    num_transitions_remaining = 0;

    // Leave OT line high to help avoid triggering 100% OpenTherm/Lite duty cycle in case of crashes.
    // Note sure this is actually helps.
    gpio_pin_set_dt(&pins.tx, pins.tx_inverted ? 0 : 1);

    return true;
  }

  ZephyrQueue<litt::OpenTherm::Frame> in_queue;

  struct RXCallbackInfo {
    struct gpio_callback data;
    ZephyrIO *io;
  } rx_callback_info;

  enum RXState { IDLE, START, DATA, STOP };

  struct {
    volatile RXState state = IDLE;
    volatile uint32_t frame = 0;
    volatile int64_t prev_time = 0;
  } rx;

  static void rx_callback(const struct device *, struct gpio_callback *cb, uint32_t) {
    RXCallbackInfo *ci = CONTAINER_OF(cb, RXCallbackInfo, data);
    ci->io->rx_callback();
  }

  void rx_callback() {
    bool rising = gpio_pin_get_dt(&pins.rx) == 1;
    int64_t time = k_uptime_ticks();
    int64_t delta;

    if (time < rx.prev_time)
      delta = k_ticks_to_us_near64((ULONG_MAX - rx.prev_time) + time);
    else
      delta = k_ticks_to_us_near64(time - rx.prev_time);

    switch (rx.state) {
    case IDLE:
      if (rising) {
        rx.state = START;
        rx.prev_time = time;
      }
      break;
    case START:
      if (delta > 750) {
        // LOG_WRN("rx_callback: frame dropped in START: %lldus", delta);
        rx.state = IDLE;
      } else if (!rising) {
        rx.frame = 0x01;
        rx.state = DATA;
      } else
        rx.state = IDLE;
      rx.prev_time = time;
      break;
    case DATA:
      if (delta > 1500) {
        // LOG_WRN("rx_callback: frame dropped in DATA: %08x", rx.frame);
        rx.state = IDLE;
        rx.prev_time = time;
      } else if (delta >= 750) {
        bool is_last = (rx.frame & 0x80000000) != 0;
        rx.frame = (rx.frame << 1) | (rising ? 0 : 1);
        if (is_last)
          rx.state = STOP;
        rx.prev_time = time;
      }
      break;
    case STOP:
      if (delta > 1500 || (delta > 750 && rising) || (delta <= 750 && !rising)) {
        // LOG_WRN("rx_callback: frame dropped in STOP: %08x", rx.frame);
      } else {
        if (!in_queue.try_add(rx.frame))
          // LOG_ERR("rx_callback: queue add failed");
          if (pins.owned)
            gpio_pin_interrupt_configure_dt(&pins.rx, GPIO_INT_DISABLE);
      }
      rx.state = IDLE;
      rx.prev_time = time;
      break;
    default:
      LOG_WRN("unknown RX state: %d", rx.state);
      rx.state = IDLE;
      rx.prev_time = time;
    }
  }
};

#endif // _ZEPHYR_LITT_IO_H_
