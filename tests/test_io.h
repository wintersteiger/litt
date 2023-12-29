#ifndef _LITT_TEST_IO_H_
#define _LITT_TEST_IO_H_

#include <math.h>
#include <stdint.h>

#include <condition_variable>
#include <mutex>
#include <queue>
#include <sstream>

#include <litt/opentherm/transport.h>

#include "test_ds.h"

class TestPins {
public:
  TestPins() {}

  TestPins(const TestPins &other) {}

  virtual ~TestPins() = default;

  std::queue<uint32_t> in;
  TestMutex in_mtx;

  std::queue<uint32_t> out;
  TestMutex out_mtx;

  std::string sizes() const {
    std::stringstream ss;
    ss << "[in=" << in.size() << ", out=" << out.size() << "]";
    return ss.str();
  }

  void clear() {
    while (!in.empty())
      in.pop();
    while (!out.empty())
      out.pop();
  }
};

class TestIO : public litt::OpenTherm::IO<TestPins> {
public:
  TestIO(const TestPins &pins) : litt::OpenTherm::IO<TestPins>(pins) {}
  virtual ~TestIO() = default;

  virtual void put(const litt::OpenTherm::Frame &f) override {
    TestPins &mpins = *const_cast<TestPins *>(&pins);
    mpins.out_mtx.lock();
    mpins.out.push(f);
    mpins.out_mtx.unlock();
  }

  virtual litt::OpenTherm::Frame get_blocking() override {
    using namespace std::literals::chrono_literals;

    TestPins &mpins = *const_cast<TestPins *>(&pins);

    while (true) {
      mpins.in_mtx.lock();
      if (!mpins.in.empty()) {
        uint32_t f = mpins.in.front();
        mpins.in.pop();
        mpins.in_mtx.unlock();
        return litt::OpenTherm::Frame(f);
      }
      mpins.in_mtx.unlock();
      std::this_thread::sleep_for(1ms);
    }
  }

  virtual bool get_timeout(litt::OpenTherm::Frame &f, uint64_t timeout_us) override {
    using namespace std::literals::chrono_literals;

    TestPins &mpins = *const_cast<TestPins *>(&pins);

    auto deadline = std::chrono::high_resolution_clock::now() + std::chrono::microseconds(timeout_us);
    do {
      if (mpins.in_mtx.lock_timeout(timeout_us)) {
        if (!mpins.in.empty()) {
          f = mpins.in.front();
          mpins.in.pop();
          // std::cout << time.get_us() << " IO: get_timeout: " << r.f.to_string() << std::endl;
          mpins.in_mtx.unlock();
          return true;
        }
        // else
        //   std::cout << "IO: get_timeout: empty" << std::endl;
        mpins.in_mtx.unlock();
      }
    } while (std::chrono::high_resolution_clock::now() <= deadline);

    return false;
  }

  virtual void log(const char *fmt, ...) override {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
    fflush(stdout);
  }

  std::string pin_sizes() const { return pins.sizes(); }

protected:
  TestTime time;
};

static constexpr unsigned test_max_concurrent_requests = 3;

class TestSlave : public litt::OpenTherm::Slave<TestTimer, TestMutex, TestSemaphore, TestTime, TestQueue, TestIO,
                                                test_max_concurrent_requests> {
public:
  TestSlave(TestPins &pins) : litt::OpenTherm::Slave<TestTimer, TestMutex, TestSemaphore, TestTime, TestQueue, TestIO, 3>(pins) {}

  virtual ~TestSlave() = default;

  virtual void on_opentherm_unsupported() override { throw std::runtime_error("OpenTherm not supported"); }
};

class TestMaster : public litt::OpenTherm::Master<TestTimer, TestMutex, TestSemaphore, TestTime, TestQueue, TestIO,
                                                  test_max_concurrent_requests> {
public:
  TestMaster(TestPins &pins)
      : litt::OpenTherm::Master<TestTimer, TestMutex, TestSemaphore, TestTime, TestQueue, TestIO, 3>(pins) {}

  virtual ~TestMaster() = default;

  virtual void on_late_frame(litt::OpenTherm::RequestID rid) const override {
    std::cout << "M: Late request #" << rid << " " << io.pin_sizes() << std::endl;
  }

  virtual void on_dropped_frame(litt::OpenTherm::RequestID rid) const override {
    std::cout << "M: Dropped request #" << rid << " " << io.pin_sizes() << std::endl;
  }
};

#endif // _LITT_TEST_IO_H_