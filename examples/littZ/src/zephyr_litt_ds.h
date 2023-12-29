#ifndef _ZEPHYR_LITT_DS_H_
#define _ZEPHYR_LITT_DS_H_

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/mutex.h>
#include <zephyr/sys/time_units.h>

#include <litt/ds.h>

class ZephyrTimer : public litt::Timer {
public:
  ZephyrTimer() : ZephyrTimer(0, 0, nullptr, nullptr, nullptr) {}

  ZephyrTimer(callback_t ftick, void *data = nullptr) : ZephyrTimer(0, 0, ftick, nullptr, data) {}

  ZephyrTimer(callback_t ftick, callback_t fstop, void *data = nullptr) : ZephyrTimer(0, 0, ftick, fstop, data) {}

  ZephyrTimer(uint64_t delay_us, uint64_t period_us, callback_t ftick, callback_t fstop = nullptr, void *data = nullptr)
      : litt::Timer(delay_us, period_us, ftick, fstop, data) {
    k_timer_init(&timer, sftick, sfstop);
    k_timer_user_data_set(&timer, this);
  }

  virtual ~ZephyrTimer() {
    if (running)
      stop(true);
  }

  virtual void start(uint64_t delay_us = 0) override {
    stop(false);
    this->delay_us = delay_us;
    if (delay_us == 0)
      k_timer_start(&timer, K_USEC(period_us), K_USEC(period_us));
    else
      k_timer_start(&timer, K_USEC(delay_us), K_USEC(period_us));
    running = true;
  }

  virtual void stop(bool run_fstop = true) override {
    this->run_fstop = run_fstop;
    k_timer_stop(&timer);
    running = false;
  }

protected:
  bool running = false;
  bool run_fstop = true;

  struct k_timer timer;

  static void sftick(k_timer *rt) {
    ZephyrTimer *pt = static_cast<ZephyrTimer *>(k_timer_user_data_get(rt));
    if (pt && pt->ftick)
      pt->ftick(pt, pt->data);
  };

  static void sfstop(k_timer *rt) {
    ZephyrTimer *pt = static_cast<ZephyrTimer *>(k_timer_user_data_get(rt));
    if (pt && pt->run_fstop && pt->fstop)
      pt->fstop(pt, pt->data);
  };
};

class ZephyrSemaphore : public litt::BinarySemaphore {
public:
  ZephyrSemaphore() : litt::BinarySemaphore() { k_sem_init(&sem, 1, 1); }
  virtual ~ZephyrSemaphore() = default;
  virtual void acquire_blocking() override { k_sem_take(&sem, K_FOREVER); }
  virtual bool try_acquire() override { return k_sem_take(&sem, K_NO_WAIT) == 0; }
  virtual bool release() override {
    k_sem_give(&sem);
    return true;
  }
  virtual bool acquire_timeout(uint64_t us) override { return k_sem_take(&sem, K_USEC(us)) == 0; }

protected:
  struct k_sem sem;
};

template <typename T> class ZephyrQueue : public litt::Queue<T> {
public:
  ZephyrQueue(size_t size) : litt::Queue<T>(size) {
    int r;
    if ((r = k_msgq_alloc_init(&q, sizeof(T), size)) != 0)
      LOG_ERR("k_msgq_alloc_init failed (error %d)", r);
  }
  virtual ~ZephyrQueue() { k_msgq_cleanup(&q); };
  virtual bool full() const override { return k_msgq_num_free_get(const_cast<k_msgq *>(&q)) == 0; }
  virtual bool empty() const override { return k_msgq_num_used_get(const_cast<k_msgq *>(&q)) == 0; }
  virtual void add(const T &data) override { k_msgq_put(&q, &data, K_FOREVER); }
  virtual bool try_add(const T &data) override { return k_msgq_put(&q, &data, K_NO_WAIT) == 0; }
  virtual T remove() override {
    T r;
    k_msgq_get(&q, &r, K_FOREVER);
    return r;
  }
  virtual bool try_remove(T &t) override { return k_msgq_get(&q, &t, K_NO_WAIT) == 0; }
  virtual size_t level() const override { return k_msgq_num_used_get(const_cast<k_msgq *>(&q)); }
  virtual bool remove_timeout(T &t, uint64_t timeout_us) override { return k_msgq_get(&q, &t, K_USEC(timeout_us)) == 0; }

  virtual void abort() {}

protected:
  struct k_msgq q;
};

class ZephyrTime : public litt::Time {
public:
  ZephyrTime() : Time() {}
  virtual ~ZephyrTime() = default;
  virtual uint64_t get_us() const override { return k_ticks_to_us_near64(k_uptime_ticks()); }
  virtual void sleep_us(uint64_t us) const override { k_sleep(K_USEC(us)); }
};

class ZephyrMutex : public litt::Mutex {
public:
  ZephyrMutex() : litt::Mutex() { k_mutex_init(&mtx); }

  virtual ~ZephyrMutex() = default;

  virtual bool lock() override { return k_mutex_lock(&mtx, K_NO_WAIT) == 0; }

  virtual void unlock() override { k_mutex_unlock(&mtx); }

  virtual bool lock_timeout(uint64_t timeout_us) override { return k_mutex_lock(&mtx, K_USEC(timeout_us)); }

protected:
  k_mutex mtx;
};

#endif // _ZEPHYR_LITT_DS_H_
