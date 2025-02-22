#ifndef _ZEPHYR_LITT_IO_H_
#define _ZEPHYR_LITT_IO_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/nordic-nrf-gpio.h>

#include <litt/opentherm/transport.h>

typedef litt::Pins<struct gpio_dt_spec> ZephyrPins;

// Note: TX timer has significant jitter, so this is not super reliable.

class ZephyrIO : public litt::OpenTherm::IO<ZephyrPins> {
public:
  ZephyrIO(const ZephyrPins &pins_, void (*rx_fblink)(bool) = nullptr, void (*tx_fblink)(bool) = nullptr)
      : litt::OpenTherm::IO<ZephyrPins>(pins_),
        tx_timer(500, 500, tx_timer_ftick, tx_timer_fstop, this),
        in_queue(16), rx_callback_info({.io = this}), rx_fblink(rx_fblink), tx_fblink(tx_fblink) {
    const ZephyrPins &p = pins_;

    if (!gpio_is_ready_dt(&p.rx))
      LOG_ERR("X RX port not ready");
    else if (!gpio_is_ready_dt(&p.tx))
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
      if ((err = gpio_add_callback_dt(&p.rx, &rx_callback_info.data)) < 0)
        LOG_ERR("X Error %d: gpio_add_callback failed.", err);
    }
  }

  virtual ~ZephyrIO() = default;

  virtual void put(const litt::OpenTherm::Frame &f) override {
    if (tx_fblink)
      tx_fblink(true);
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
    log_generic(LOG_LEVEL_WRN, fmt, args);
    va_end(args);
  }

  // Without pin ownership, we rely on other ISRs to call isr().
  void isr() {
    if (!pins.owned)
      rx_callback();
  }

  void cease() {
    tx_timer.stop(true);
  }

  void set_cur_req_id(uint64_t id) { cur_req_id = id; }

  uint64_t cur_req_id = 0;

// protected:
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
      rx.state = IDLE;
      rx.rising = false;
      rx.num_irqs = 0;
      if (pins.owned)
        gpio_pin_interrupt_configure_dt(&pins.rx, GPIO_INT_EDGE_BOTH);
      gpio_pin_set_dt(&pins.tx, pins.tx_inverted ? 1 : 0);
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

    if (tx_fblink)
      tx_fblink(false);

    return true;
  }

  ZephyrQueue<litt::OpenTherm::Frame> in_queue;

  struct RXCallbackInfo {
    struct gpio_callback data;
    ZephyrIO *io;
  } rx_callback_info;

  void (*rx_fblink)(bool);
  void (*tx_fblink)(bool);

  enum RXState { IDLE, START, DATA, STOP };

  RXState rx_state() const { return rx.state; }

  struct {
    volatile RXState state = IDLE;
    volatile uint32_t frame = 0;
    volatile int64_t prev_time = 0;
    volatile bool rising = false;
    volatile uint64_t num_irqs = 0;
  } rx;

  static void rx_callback(const struct device *, struct gpio_callback *cb, uint32_t) {
    RXCallbackInfo *ci = CONTAINER_OF(cb, RXCallbackInfo, data);
    ci->io->rx_callback();
  }

  void rx_callback() {
    rx.rising = !rx.rising;
    bool rising = rx.rising;
    int64_t time = k_uptime_ticks();
    int64_t delta;

    rx.num_irqs++;

    auto stop = [this]() {
      if (pins.owned)
        gpio_pin_interrupt_configure_dt(&pins.rx, GPIO_INT_DISABLE);
    };

    if (time < rx.prev_time) {
      delta = k_ticks_to_us_near64((INT64_MAX - rx.prev_time) + time);
      LOG_INF("time < prev_time: prev=%lld cur=%lld delta=%lld", rx.prev_time, time, delta);
    } else
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
        LOG_INF("RX ABORT 1 %lld (delta: %lld)", cur_req_id, delta);
        stop();
      } else if (!rising) {
        rx.frame = 0x01;
        rx.state = DATA;
      } else {
        LOG_INF("RX ABORT 2 %lld", cur_req_id);
        stop();
      }
      rx.prev_time = time;
      break;
    case DATA:
      if (delta > 1550) {
        LOG_INF("RX ABORT 3 %lld (delta: %lld)", cur_req_id, delta);
        stop();
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
      if (delta > 1550 || (delta > 750 && rising) || (delta <= 750 && !rising))
        LOG_INF("RX ABORT 4 %lld (delta: %lld)", cur_req_id, delta);
      else {
        if (!in_queue.try_add(rx.frame))
          LOG_WRN("RX QUEUE ADD FAILED");
      }
      stop();
      rx.prev_time = time;
      break;
    default:
      LOG_WRN("unknown RX state: %d", rx.state);
      rx.state = IDLE;
      rx.prev_time = time;
    }

    if (rx_fblink) {
      switch (rx.state) {
      case START:
        rx_fblink(true);
        break;
      case DATA:
        break;
      default:
        rx_fblink(false);
      }
    }
  }
};

#endif // _ZEPHYR_LITT_IO_H_
