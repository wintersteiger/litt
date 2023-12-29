// CM Wintersteiger, 2022

#ifndef _LITT_DS_H
#define _LITT_DS_H

#include <stddef.h>
#include <stdint.h>

namespace litt {

template <typename T = unsigned> struct Pins {
  T rx;
  T tx;
  bool owned;
  bool tx_inverted;
};

class BinarySemaphore {
public:
  BinarySemaphore() {}
  virtual ~BinarySemaphore() = default;

  virtual void acquire_blocking() = 0;
  virtual bool acquire_timeout(uint64_t us) = 0;
  virtual bool release() = 0;
  virtual bool try_acquire() = 0;
};

template <typename T> class Queue {
public:
  Queue(size_t size) {}
  virtual ~Queue() = default;
  virtual bool full() const = 0;
  virtual bool empty() const = 0;
  virtual void add(const T &data) = 0;
  virtual bool try_add(const T &data) = 0;
  virtual T remove() = 0;
  virtual bool try_remove(T &t) = 0;
  virtual bool remove_timeout(T &t, uint64_t timeout_us) = 0;
  virtual size_t level() const = 0;
};

class Timer {
public:
  typedef bool (*callback_t)(Timer *, void *);

  Timer() = default;

  Timer(callback_t ftick, void *data = nullptr) : Timer(0, 0, ftick, nullptr, data) {}

  Timer(callback_t ftick, callback_t fstop, void *data = nullptr) : Timer(0, 0, ftick, fstop, data) {}

  Timer(uint64_t delay_us, uint64_t period_us, callback_t ftick, callback_t fstop = nullptr, void *data = nullptr)
      : delay_us(delay_us), period_us(period_us), ftick(ftick), fstop(fstop), data(data) {}

  ~Timer() = default;

  virtual void start(uint64_t delay_us = 0) = 0;

  virtual void stop(bool run_fstop = true) = 0;

  virtual bool tick() {
    if (!ftick)
      return false;
    return ftick(this, data);
  }

  virtual void restart(uint64_t delay_us = 0) {
    stop(false);
    start(delay_us);
  }

  Timer &operator=(Timer &) = delete;

protected:
  uint64_t delay_us = 0;
  uint64_t period_us = 0;
  bool (*ftick)(Timer *, void *) = nullptr;
  bool (*fstop)(Timer *, void *) = nullptr;
  void *data = nullptr;
};

class Time {
public:
  Time() = default;
  virtual ~Time() = default;
  virtual uint64_t get_us() const = 0;
  virtual void sleep_us(uint64_t) const = 0;
};

class RealTime {
public:
  RealTime() = default;
  virtual ~RealTime() = default;

  void set_available(bool on) { available = on; }
  bool is_available() const { return available; }

  // Note: weekday == 0 means Sunday.
  virtual bool get_weekday_minutes(uint8_t &weekday, uint16_t &minutes) const = 0;

protected:
  bool available = false;
};

class Mutex {
public:
  Mutex() = default;
  virtual ~Mutex() = default;

  virtual bool lock() = 0;
  virtual void unlock() = 0;
  virtual bool lock_timeout(uint64_t timeout_us) = 0;
};

class LockGuard {
public:
  LockGuard(Mutex &mtx) : mtx(mtx) { mtx.lock(); }

  LockGuard(LockGuard&) = delete;
  LockGuard(const LockGuard&) = delete;
  LockGuard(LockGuard&&) = delete;

  virtual ~LockGuard() { mtx.unlock(); }

protected:
  Mutex &mtx;
};

} // namespace litt

#endif // _LITT_DS_H