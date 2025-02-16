#ifndef _ARDUINO_LITT_IO_H_
#define _ARDUINO_LITT_IO_H_

#include <litt/opentherm/transport.h>

class ArduinoIO;
extern ArduinoIO *io;

typedef litt::Pins<int> ArduinoPins;

class ArduinoIO : public litt::OpenTherm::IO<ArduinoPins> {
public:
  ArduinoIO(const ArduinoPins &pins, void (*rx_fblink)(bool) = nullptr, void (*tx_fblink)(bool) = nullptr)
      : litt::OpenTherm::IO<ArduinoPins>(pins), rx_state(IDLE), rx_fblink(rx_fblink), tx_fblink(tx_fblink) {
    if (pins.owned) {
      pinMode(pins.rx, INPUT_PULLUP);
      pinMode(pins.tx, OUTPUT);
    }
    ::io = this;
  }

  virtual ~ArduinoIO() = default;

  virtual void log(const char *fmt, ...) override {
    va_list args;
    va_start(args, fmt);
    ::vllog(fmt, args);
    va_end(args);
  }

  using IO::put;

  void send_bit(bool v) const {
    digitalWrite(pins.tx, pins.tx_inverted ^ v ? HIGH : LOW);
    delayMicroseconds(487);
    digitalWrite(pins.tx, pins.tx_inverted ^ v ? LOW : HIGH);
    delayMicroseconds(487);
  }

  virtual void put(const litt::OpenTherm::Frame &f) override {
    uint32_t x = (uint32_t)f;
    if (tx_fblink)
      tx_fblink(true);
    send_bit(true);
    for (size_t i = 0; i < 32; i++) {
      send_bit((x & 0x80000000) != 0);
      x <<= 1;
    }
    send_bit(true);
    if (tx_fblink)
      tx_fblink(false);
  }

  virtual litt::OpenTherm::Frame get_blocking() override {
    waiting_task = xTaskGetCurrentTaskHandle();
    if (pins.owned)
      attachInterrupt(digitalPinToInterrupt(pins.rx), internal_isr, CHANGE);
    while (rx_state != COMPLETE && rx_state != ERROR)
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (pins.owned)
      detachInterrupt(digitalPinToInterrupt(pins.rx));
    rx_state = IDLE;
    return litt::OpenTherm::Frame(frame);
  }

  virtual bool get_timeout(litt::OpenTherm::Frame &f, uint64_t timeout_us) override {
    waiting_task = xTaskGetCurrentTaskHandle();
    if (pins.owned)
      attachInterrupt(digitalPinToInterrupt(pins.rx), internal_isr, CHANGE);
    while (rx_state != COMPLETE && rx_state != ERROR)
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_us) / 1000);
    if (pins.owned)
      detachInterrupt(digitalPinToInterrupt(pins.rx));
    rx_state = IDLE;
    return false;
  }

  // Without pin ownership, we rely on other ISRs to call isr().
  void isr() { rx_callback(this); }

protected:
  enum RXState { IDLE, START, DATA, STOP, COMPLETE, ERROR };
  volatile RXState rx_state = IDLE;
  volatile uint32_t frame = 0;
  volatile uint32_t prev_time = 0;
  volatile TaskHandle_t waiting_task;
  void (*rx_fblink)(bool);
  void (*tx_fblink)(bool);

  static void internal_isr() { rx_callback(::io); }

  static void rx_callback(ArduinoIO *io) {
    bool rising = digitalRead(io->pins.rx);
    uint32_t time = micros();
    int32_t delta = time - io->prev_time;

    if (delta < 0)
      delta += LONG_MAX;

    switch (io->rx_state) {
    case IDLE:
      if (rising) {
        io->rx_state = START;
        io->prev_time = time;
      }
      break;
    case START:
      if (delta > 750) {
        llog("D START");
        io->rx_state = ERROR;
      } else if (!rising) {
        io->frame = 0x01;
        io->rx_state = DATA;
      } else
        io->rx_state = IDLE;
      io->prev_time = time;
      break;
    case DATA:
      if (delta > 1750) {
        llog("D DATA: %08x", io->frame);
        io->rx_state = ERROR;
        io->prev_time = time;
      } else if (delta >= 750) {
        bool is_last = (io->frame & 0x80000000) != 0;
        io->frame = (io->frame << 1) | (rising ? 0 : 1);
        if (is_last)
          io->rx_state = STOP;
        io->prev_time = time;
      }
      break;
    case STOP:
      if (delta > 1500 || (delta > 750 && rising) || (delta <= 750 && !rising)) {
        llog("D STOP: %08x", io->frame);
        io->rx_state = ERROR;
      } else
        io->rx_state = COMPLETE;
      io->prev_time = time;
      break;
    case COMPLETE:
    case ERROR:
      break;
    default:
      llog("unknown RX state: %d", io->rx_state);
      io->rx_state = ERROR;
      io->prev_time = time;
    }
    if (io->rx_fblink) {
      switch (io->rx_state) {
      case START:
        io->rx_fblink(true);
        break;
      case DATA:
        break;
      default:
        io->rx_fblink(false);
      }
    }

    if (io->rx_state == ERROR || io->rx_state == COMPLETE) {
      if (io->waiting_task)
        vTaskNotifyGiveFromISR(io->waiting_task, NULL);
    }
  }
};

#endif // _ARDUINO_LITT_IO_H_
