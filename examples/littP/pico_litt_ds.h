#ifndef _PICO_OPENTHERM_DS_H_
#define _PICO_OPENTHERM_DS_H_

#include "pico/mutex.h"
#include <hardware/rtc.h>
#include <hardware/timer.h>
#include <pico/critical_section.h>
#include <pico/util/queue.h>

#include <litt/ds.h>

extern void vllog(const char *fmt, va_list args);
extern void llog(const char *fmt, ...);

class PicoTimer : public litt::Timer {
public:
  PicoTimer() : PicoTimer(0, 0, nullptr, nullptr, nullptr) {}

  PicoTimer(callback_t ftick, void *data = nullptr) : PicoTimer(0, 0, ftick, nullptr, data) {}

  PicoTimer(callback_t ftick, callback_t fstop, void *data = nullptr) : PicoTimer(0, 0, ftick, fstop, data) {}

  PicoTimer(uint64_t delay_us, uint64_t period_us, callback_t ftick, callback_t fstop = nullptr, void *data = nullptr)
      : litt::Timer(delay_us, period_us, ftick, fstop, data) {
    if (!alarm_pool)
      alarm_pool = alarm_pool_create(2, 16);
  }

  virtual ~PicoTimer() {
    if (running)
      stop(true);
  }

  virtual void start(uint64_t delay_us = 0) override {
    stop(false);

    this->delay_us = delay_us;
    running = true;

    if (delay_us == 0) {
      if (!alarm_pool_add_repeating_timer_us(alarm_pool, -period_us, pcb, this, &t))
        llog("No alarm slots available!");
    } else {
      alarm_id = alarm_pool_add_alarm_in_us(alarm_pool, delay_us, acb, this, true);
    }
  }

  virtual void stop(bool run_fstop = true) override {
    running = false;
    cancel_repeating_timer(&t);
    if (alarm_id != -1) {
      alarm_pool_cancel_alarm(alarm_pool, alarm_id);
      alarm_id = -1;
    }
    if (run_fstop && fstop)
      fstop(this, data);
  }

protected:
  bool running = false;
  static alarm_pool_t *alarm_pool;
  repeating_timer_t t = {0};
  volatile alarm_id_t alarm_id = -1;

  static bool pcb(repeating_timer_t *rt) {
    PicoTimer *pt = static_cast<PicoTimer *>(rt->user_data);
    return pt->ftick(pt, pt->data);
  };

  static int64_t acb(alarm_id_t id, void *data) {
    PicoTimer *pt = static_cast<PicoTimer *>(data);
    if (pt) {
      if (pt->alarm_id != id)
        return 0;
      alarm_pool_cancel_alarm(alarm_pool, id);
      pt->alarm_id = -1;
      if (pt->period_us != 0)
        alarm_pool_add_repeating_timer_us(alarm_pool, -pt->period_us, pcb, pt, &pt->t);
      else
        pt->running = false;
      if (pt->ftick)
        pt->ftick(pt, pt->data);
    }
    return 0;
  }
};

class PicoSemaphore : public litt::BinarySemaphore {
public:
  PicoSemaphore() : litt::BinarySemaphore() { sem_init(&sem, 1, 1); }
  virtual ~PicoSemaphore() = default;
  virtual void acquire_blocking() override { sem_acquire_blocking(&sem); }
  virtual bool try_acquire() override { return sem_try_acquire(&sem); }
  virtual bool release() override { return sem_release(&sem); }
  virtual bool acquire_timeout(uint64_t us) override { return sem_acquire_timeout_us(&sem, us); }

protected:
  semaphore_t sem;
};

template <typename T> class PicoQueue : public litt::Queue<T> {
public:
  PicoQueue(uint size) : litt::Queue<T>(size) { queue_init(&q, sizeof(T), size); }
  virtual ~PicoQueue() { queue_free(&q); };
  virtual bool full() const override { return queue_is_full(const_cast<queue_t *>(&q)); }
  virtual bool empty() const override { return queue_is_empty(const_cast<queue_t *>(&q)); }
  virtual void add(const T &data) override { queue_add_blocking(&q, &data); }
  virtual bool try_add(const T &data) override { return queue_try_add(&q, &data); }
  virtual T remove() override {
    T r;
    queue_remove_blocking(&q, &r);
    return r;
  }
  virtual bool try_remove(T &t) override { return queue_try_remove(&q, &t); }
  virtual size_t level() const override { return queue_get_level(const_cast<queue_t *>(&q)); }

  virtual bool remove_timeout(T &t, uint64_t timeout_us) override {
    auto deadline = time_us_64() + timeout_us;
    do {
      if (queue_try_remove(&q, &t))
        return true;
      sleep_us(1000);
    } while (time_us_64() <= deadline);
    return false;
  }

  void abort() {}

protected:
  queue_t q;
};

class PicoTime : public litt::Time {
public:
  PicoTime() : litt::Time() {}
  virtual ~PicoTime() = default;
  virtual uint64_t get_us() const override { return time_us_64(); }
  virtual void sleep_us(uint64_t us) const override { ::busy_wait_us(us); }
};

class PicoRealTime : public litt::RealTime {
public:
  PicoRealTime() : litt::RealTime() {}
  virtual ~PicoRealTime() = default;

  virtual bool get_weekday_minutes(uint8_t &weekday, uint16_t &minutes) const override {
    if (!is_available())
      return false;
    datetime_t dt;
    rtc_get_datetime(&dt);
    weekday = dt.dotw;
    minutes = 60 * dt.hour + dt.min;
    return true;
  }
};

class PicoMutex : public litt::Mutex {
public:
  PicoMutex() : litt::Mutex() { mutex_init(&mtx); }

  virtual ~PicoMutex() {}

  virtual bool lock() override { return mutex_try_enter(&mtx, NULL); }

  virtual void unlock() override { mutex_exit(&mtx); }

  virtual bool lock_timeout(uint64_t timeout_us) override { return mutex_enter_timeout_us(&mtx, timeout_us); }

protected:
  mutex_t mtx;
};

#endif // _PICO_OPENTHERM_DS_H_
