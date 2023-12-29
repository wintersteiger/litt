#ifndef _ARDUINO_LITT_DS_H_
#define _ARDUINO_LITT_DS_H_

#include <limits.h>

#include "FreeRTOSConfig.h"

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <timers.h>


extern void llog(const char *, ...);
extern void vllog(const char *fmt, va_list args);

class ArduinoTimer : public litt::Timer {
public:
  ArduinoTimer() : ArduinoTimer(0, 0, nullptr, nullptr, nullptr) {}

  ArduinoTimer(callback_t ftick, void *data = nullptr)
      : ArduinoTimer(0, 0, ftick, nullptr, data) {}

  ArduinoTimer(callback_t ftick, callback_t fstop, void *data = nullptr)
      : ArduinoTimer(0, 0, ftick, fstop, data) {}

  ArduinoTimer(uint64_t delay_us, uint64_t period_us, callback_t ftick,
               callback_t fstop = nullptr, void *data = nullptr)
      : litt::Timer(delay_us, period_us, ftick, fstop, data) {
    if (!(t = xTimerCreateStatic("timer", pdMS_TO_TICKS(period_us) / 1000,
                                 pdTRUE, this, pcb, &state)))
      llog("xTimerCreate failed");
  }

  virtual ~ArduinoTimer() = default;

  virtual void start(uint64_t delay_us = 0) override {
    if (xTimerStart(t, 0) == pdFAIL)
      llog("xTimerStart failed");
  }

  virtual void stop(bool run_fstop = true) override {
    if (xTimerStop(t, 0) == pdFAIL)
      llog("xTimerStop failed");
  }

protected:
  TimerHandle_t t;
  StaticTimer_t state;

  static void pcb(TimerHandle_t xTimer) {
    ArduinoTimer *at = static_cast<ArduinoTimer *>(pvTimerGetTimerID(xTimer));
    return at->ftick(at, at->data);
  };
};

class ArduinoSemaphore : public litt::BinarySemaphore {
public:
  ArduinoSemaphore() : litt::BinarySemaphore() {
    if (!(s = xSemaphoreCreateBinaryStatic(&state)))
      llog("xSemaphoreCreateCounting failed");
    xSemaphoreGive(s);
  }
  virtual ~ArduinoSemaphore() = default;

  virtual void acquire_blocking() override { xSemaphoreTake(s, portMAX_DELAY); }
  virtual bool acquire_timeout(uint64_t us) override {
    return xSemaphoreTake(s, pdMS_TO_TICKS(us) / 1000);
  }
  virtual bool release() override { xSemaphoreGive(s); }
  virtual bool try_acquire() override { return xSemaphoreTake(s, 0); }

protected:
  SemaphoreHandle_t s;
  StaticSemaphore_t state;
};

class ArduinoTime : public litt::Time {
public:
  ArduinoTime() = default;
  virtual ~ArduinoTime() = default;

  virtual uint64_t get_us() const override { return micros(); }
  virtual void sleep_us(uint64_t us) const override {
    vTaskDelay(pdMS_TO_TICKS(us) / 1000);
  }
};

template <typename T> class ArduinoQueue : public litt::Queue<T> {
public:
  static const constexpr size_t queue_size = 2;

  ArduinoQueue(size_t size = queue_size) : litt::Queue<T>(size) {
    if (!(q = xQueueCreateStatic(size, sizeof(T), storage, &state)))
      llog("xQueueCreate failed");
  }
  virtual ~ArduinoQueue() { vQueueDelete(q); }

  virtual bool full() const override { return xQueueIsQueueFullFromISR(q); }
  virtual bool empty() const override { return xQueueIsQueueEmptyFromISR(q); }
  virtual void add(const T &data) override {
    xQueueSend(q, &data, portMAX_DELAY);
  }
  virtual bool try_add(const T &data) override {
    return xQueueSend(q, &data, 0);
  }
  virtual T remove() override {
    T r;
    xQueueReceive(q, &r, portMAX_DELAY);
    return r;
  }
  virtual bool try_remove(T &t) override { return xQueueReceive(q, &t, 0); }
  virtual size_t level() const override { return uxQueueMessagesWaiting(q); }

protected:
  QueueHandle_t q;
  StaticQueue_t state;
  uint8_t storage[queue_size * sizeof(T)];
};

#endif // _ARDUINO_LITT_DS_H_
