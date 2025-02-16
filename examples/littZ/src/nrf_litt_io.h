#ifndef _NRF_LITT_IO_H_
#define _NRF_LITT_IO_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/nordic-nrf-gpio.h>

#include <nrfx_pwm.h>

#include <litt/opentherm/transport.h>

typedef litt::Pins<struct gpio_dt_spec> ZephyrPins;

#define PWM_INST_IDX 1
#define PWM_COUNTER_TOP 1000

class NRFIO : public litt::OpenTherm::IO<ZephyrPins> {
public:
  NRFIO(const ZephyrPins &pins_, void (*rx_fblink)(bool) = nullptr, void (*tx_fblink)(bool) = nullptr)
      : litt::OpenTherm::IO<ZephyrPins>(pins_), in_queue(16), rx_callback_info({.io = this}), rx_fblink(rx_fblink),
        tx_fblink(tx_fblink) {
    const ZephyrPins &p = pins_;

    if (!gpio_is_ready_dt(&p.rx))
      LOG_ERR("X RX port not ready");
    else if (!gpio_is_ready_dt(&p.tx))
      LOG_ERR("X TX port not ready");

    int err;
    if ((err = gpio_pin_configure_dt(&p.rx, GPIO_INPUT)) < 0)
      LOG_ERR("X Error %d: RX gpio_pin_configure failed.", err);

    if (p.owned) {
      gpio_init_callback(&rx_callback_info.data, rx_callback, BIT(p.rx.pin));
      if ((err = gpio_add_callback_dt(&p.rx, &rx_callback_info.data)) < 0)
        LOG_ERR("X Error %d: gpio_add_callback failed.", err);
    }

    bool tx_inverted = (p.tx.dt_flags & GPIO_ACTIVE_LOW) != 0;

    for (size_t i = 0; i < 34; i++)
      tx_seq_values[i].channel_0 = PWM_COUNTER_TOP / 2;

    if (!tx_inverted)
      tx_seq_values[0].channel_0 = tx_seq_values[33].channel_0 = (PWM_COUNTER_TOP / 2) | 0x8000;

    uint8_t tx_pin = 0;
    // TODO: Figure out how to get the port number from the port device
    const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    tx_pin = NRF_PIN_PORT_TO_PIN_NUMBER(p.tx.pin, p.tx.port == gpio0_dev ? 0ul : 1ul);

    pwm = NRFX_PWM_INSTANCE(PWM_INST_IDX);

    pwm_config = {.output_pins =
                      {
                          tx_pin,
                          NRF_PWM_PIN_NOT_CONNECTED,
                          NRF_PWM_PIN_NOT_CONNECTED,
                          NRF_PWM_PIN_NOT_CONNECTED,
                      },
                  .pin_inverted =
                      {
                          tx_inverted,
                          false,
                          false,
                          false,
                      },
                  .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
                  .base_clock = NRF_PWM_CLK_1MHz,
                  .count_mode = NRF_PWM_MODE_UP,
                  .top_value = PWM_COUNTER_TOP,
                  .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
                  .step_mode = NRF_PWM_STEP_AUTO,
                  .skip_gpio_cfg = false};

    tx_seq = {.values = {.p_individual = &tx_seq_values[0]},
              .length = NRF_PWM_VALUES_LENGTH(tx_seq_values),
              .repeats = 0,
              .end_delay = 0};

    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_INST_IDX)), 0, NRFX_PWM_INST_HANDLER_GET(PWM_INST_IDX), 0, 0);

    if (nrfx_pwm_init(&pwm, &pwm_config, &tx_stop, this) != NRFX_SUCCESS)
      LOG_ERR("Error initializing PWM port");
  }

  static void tx_stop(nrfx_pwm_evt_type_t event, void *p) {
    auto io = static_cast<NRFIO *>(p);
    io->tx_stop();
  }

  virtual ~NRFIO() = default;

  virtual void put(const litt::OpenTherm::Frame &f) override {
    if (tx_fblink)
      tx_fblink(true);

    uint32_t fu32 = f;
    for (size_t i = 0; i < 32; i++) {
      if (fu32 & (1 << (31 - i)))
        tx_seq_values[i + 1].channel_0 |= 0x8000;
      else
        tx_seq_values[i + 1].channel_0 &= ~0x8000;
    }

    nrfx_pwm_simple_playback(&pwm, &tx_seq, 1, NRFX_PWM_FLAG_STOP);
  }

  void tx_stop() {
    if (tx_fblink)
      tx_fblink(false);

    rx.state = IDLE;
    rx.rising = false;
    rx.num_irqs = 0;
    if (pins.owned)
      gpio_pin_interrupt_configure_dt(&pins.rx, GPIO_INT_EDGE_BOTH);
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

  void cease() {}

  void set_cur_req_id(uint64_t id) { cur_req_id = id; }

  uint64_t cur_req_id = 0;

  // protected:
  nrfx_pwm_t pwm;
  nrfx_pwm_config_t pwm_config;
  nrf_pwm_values_individual_t tx_seq_values[34];
  nrf_pwm_sequence_t tx_seq;

  ZephyrQueue<litt::OpenTherm::Frame> in_queue;

  struct RXCallbackInfo {
    struct gpio_callback data;
    NRFIO *io;
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

#endif // _NRF_LITT_IO_H_
