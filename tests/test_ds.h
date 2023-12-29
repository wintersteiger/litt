#ifndef _LITT_TEST_DS_H_
#define _LITT_TEST_DS_H_

#include <math.h>
#include <stdint.h>

#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include <litt/ds.h>

uint64_t time_us() {
  auto t = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
}

class TestMutex : public litt::Mutex {
public:
  TestMutex() = default;

  virtual ~TestMutex() = default;

  virtual bool lock() override {
    m.lock();
    return true;
  }
  virtual void unlock() override { m.unlock(); }

  virtual bool lock_timeout(uint64_t timeout_us) override {
    return m.try_lock_for(std::chrono::microseconds(timeout_us));
  }

protected:
  std::timed_mutex m;
};

class TestSemaphore : public litt::BinarySemaphore {
public:
  TestSemaphore() {}

  virtual ~TestSemaphore() {}

  virtual void acquire_blocking() override {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this] { return !acquired; });
    acquired = true;
  }

  virtual bool acquire_timeout(uint64_t us) override {
    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, std::chrono::microseconds(us), [this] { return !acquired; }))
      return false;
    // std::cout << time.get_us() << " acquired w/ T/O by " << std::this_thread::get_id() << std::endl;
    acquired = true;
    return true;
  }

  virtual bool release() override {
    std::unique_lock<std::mutex> lock(mtx);
    acquired = false;
    lock.unlock();
    cv.notify_one();
    return true;
  }

  virtual bool try_acquire() override {
    std::lock_guard<std::mutex> lock(mtx);
    if (acquired)
      return false;
    acquired = true;
    return true;
  }

protected:
  std::mutex mtx;
  std::condition_variable cv;
  std::atomic<bool> acquired = true;
};

template <typename T> class TestQueue : public litt::Queue<T> {
public:
  TestQueue(size_t size) : litt::Queue<T>(size), max_size(size) {}

  virtual ~TestQueue() = default;

  void abort() { keep_running = false; }

  virtual bool full() const override { return d.size() >= max_size; }

  virtual bool empty() const override { return d.empty(); }

  virtual void add(const T &data) override {
    using namespace std::chrono_literals;

    while (d.size() >= max_size && keep_running)
      std::this_thread::sleep_for(1ms);

    if (keep_running)
      d.push(data);
  }

  virtual bool try_add(const T &data) override {
    if (d.size() >= max_size)
      return false;
    d.push(data);
    return true;
  }

  virtual T remove() override {
    using namespace std::chrono_literals;

    while (d.empty() && keep_running)
      std::this_thread::sleep_for(1ms);

    if (!keep_running)
      return {};

    T r = d.front();
    d.pop();
    return r;
  }

  virtual bool try_remove(T &t) override {
    if (d.empty())
      return false;
    t = d.front();
    d.pop();
    return true;
  }

  virtual bool remove_timeout(T &t, uint64_t timeout_us) override {
    using namespace std::chrono_literals;

    auto deadline = std::chrono::high_resolution_clock::now() + std::chrono::microseconds(timeout_us);
    do {
      if (try_remove(t))
        return true;
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::high_resolution_clock::now() <= deadline);

    return false;
  }

  virtual size_t level() const override { return d.size(); }

protected:
  size_t max_size;
  std::queue<T> d;
  volatile bool keep_running = true;
};

class TestTime : public litt::Time {
public:
  TestTime() : Time() {}
  virtual ~TestTime() = default;

  virtual uint64_t get_us() const override {
    auto t = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
  }

  virtual void sleep_us(uint64_t us) const override { std::this_thread::sleep_for(std::chrono::microseconds(us)); }
};

class TestTimer : public litt::Timer {
public:
  TestTimer(callback_t ftick, void *data = nullptr) : litt::Timer(ftick, data) {}

  TestTimer(uint64_t delay_us, uint64_t period_us, callback_t ftick, callback_t fstop = nullptr, void *data = nullptr)
      : litt::Timer(delay_us, period_us, ftick, fstop, data) {}

  virtual ~TestTimer() {
    abort = true;
    cv.notify_all();
    if (timer_thread) {
      timer_thread->join();
      delete timer_thread;
      timer_thread = nullptr;
    }
  }

  virtual void start(uint64_t delay_us = 0) override {
    stop(false);

    this->delay_us = delay_us;

    if (delay_us == 0)
      ftick(this, nullptr);

    if (timer_thread) {
      timer_thread->join();
      delete timer_thread;
    }

    abort = false;
    timer_thread = spawn_thread(this, delay_us != 0 ? delay_us : period_us);
  }

  virtual void stop(bool run_fstop = true) override {
    abort = true;
    cv.notify_all();
    if (run_fstop && fstop)
      fstop(this, data);
  }

protected:
  std::thread *timer_thread = nullptr;
  bool abort = false;
  std::mutex mtx;
  std::condition_variable cv;

  static std::thread *spawn_thread(TestTimer *t, uint64_t wait_time_us) {
    auto r = new std::thread([t, wait_time_us]() {
      uint64_t wt = wait_time_us;
      while (!t->abort && wt != 0) {
        std::unique_lock<std::mutex> lock(t->mtx);
        t->cv.wait_for(lock, std::chrono::microseconds(wt));
        if (t->abort) {
          if (t->fstop)
            t->fstop(t, t->data);
          return;
        } else {
          if (t->ftick && t->ftick(t, t->data))
            wt = t->period_us;
          else
            return;
        }
      }
    });
    return r;
  }
};

class TestCHIF : public litt::CentralHeatingInterface {
public:
  TestCHIF() = default;
  virtual ~TestCHIF() = default;

  virtual void enable() override { enbld = true; }
  virtual void disable() override { enbld = false; }
  virtual bool enabled() override { return enbld; }

  virtual bool flame() const override { return enbld && fsp > ft; }

  virtual float flow_setpoint() const override { return fsp; }
  virtual float flow_temperature() const override { return ft; }

  virtual bool set_flow_setpoint(float temperature) override {
    fsp = temperature;
    return true;
  }

  void set_flow_temperature(float t) {
    // Note: Normally done by the transport layer.
    ft = t;
  }

protected:
  bool enbld = false;
  float fsp = nanf(""), ft = 24.0f;
};

#endif // _LITT_TEST_DS_H_
