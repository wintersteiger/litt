#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"

#include <atomic>
#include <thread>

#include <litt/opentherm/transport.h>
#include <litt/thermostat.h>

#include "test_ds.h"
#include "test_io.h"

using namespace litt;
using namespace litt::OpenTherm;
using namespace std::chrono_literals;

static TestTime tt;

static TestPins master_pins;
static TestPins slave_pins;

static std::atomic_bool test_abort = false;
static std::atomic_bool kill_wire = false;

static std::thread wire([]() {
  while (!kill_wire) {
    {
      if (!master_pins.out.empty()) {
        if (master_pins.out_mtx.lock_timeout(1000)) {
          if (slave_pins.in_mtx.lock_timeout(1000)) {
            uint32_t f = master_pins.out.front();
            slave_pins.in.push(f);
            master_pins.out.pop();
            // std::cout << tt.get_us() << " W: m -> s: " << Frame(f).to_string() << std::endl;
            slave_pins.in_mtx.unlock();
          }
          master_pins.out_mtx.unlock();
        }
      }
    }
    {
      if (!slave_pins.out.empty()) {
        if (slave_pins.out_mtx.lock_timeout(1000)) {
          if (master_pins.in_mtx.lock_timeout(1000)) {
            uint32_t f = slave_pins.out.front();
            master_pins.in.push(f);
            slave_pins.out.pop();
            // std::cout << tt.get_us() << " W: s -> m: " << Frame(f).to_string() << std::endl;
            master_pins.in_mtx.unlock();
          }
          slave_pins.out_mtx.unlock();
        }
      }

      // std::cout << "wire: " << master_pins.sizes() << "/" << slave_pins.sizes() << std::endl;
    }

    // std::this_thread::sleep_for(10ms);
  }
});

class MasterApp : public RichApplication {
public:
  MasterApp() : transport(master_pins), RichApplication(transport) {
    test_abort = false;
    master_pins.clear();
  }

  virtual ~MasterApp() {
    test_abort = true;
    transport.abort();
    master_pins.in.push(0); // Master may be blocking, so we tickle it with one last frame.
    if (master_rx.joinable())
      master_rx.join();
    if (master_tx.joinable())
      master_tx.join();
  }

  virtual void run() override {
    transport.set_frame_callback(
        [](Application &app, const Frame &frame) {
          app.process(frame); // TODO: Get rid of the need to call process manually
          // std::cout << tt.get_us() << " M: " << frame.to_string() << std::endl;
          return true;
        },
        this);

    transport.start();
    master_tx = std::thread([this]() { transport.tx_until([]() { return test_abort.load(); }); });
    master_rx = std::thread([this]() { transport.rx_until([]() { return test_abort.load(); }); });

    // std::cout << "master_tx = " << master_tx.get_id() << std::endl;
    // std::cout << "master_rx = " << master_rx.get_id() << std::endl;

    pthread_setname_np(master_tx.native_handle(), "Master TX");
    pthread_setname_np(master_rx.native_handle(), "Master RX");
  }

  virtual void on_fault_indication() override { std::cout << "MASTER FAULT" << std::endl; }

  void propagate() {
    while (transport.num_pending_requests() != 0 && !test_abort)
      std::this_thread::sleep_for(10ms);
  }

  std::thread master_tx, master_rx;
  TestMaster transport;
};

class SlaveApp : public RichApplication {
public:
  SlaveApp() : transport(slave_pins), RichApplication(transport) {
    slave_pins.clear();
    test_abort = false;
  }

  virtual ~SlaveApp() {
    test_abort = true;
    transport.abort();
    slave_pins.in.push(0); // Slave may be blocking, so we tickle it with one last frame.
    if (slave_rx.joinable())
      slave_rx.join();
  }

  virtual void run() override {
    transport.set_frame_callback(
        [](Application &app, const Frame &frame) {
          app.process(frame); // TODO: Get rid of the need to call process manually
          // std::cout << tt.get_us() << " S: " << frame.to_string() << std::endl;
          return true;
        },
        this);

    transport.start();
    slave_rx = std::thread([this]() { transport.rx_until([]() { return test_abort.load(); }); });
    // std::cout << "slave_rx = " << slave_rx.get_id() << std::endl;

    pthread_setname_np(slave_rx.native_handle(), "Slave RX");
  }

  virtual void on_fault_indication() override { std::cout << "SLAVE FAULT" << std::endl; }

  std::thread slave_rx;
  TestSlave transport;
};

TEST_CASE("Initial frames") {
  MasterApp m;
  SlaveApp s;

  m.run();
  s.run();

  std::this_thread::sleep_for(3s);

  REQUIRE(m.transport.tx_frame_count >= 2);
  REQUIRE(s.transport.rx_frame_count >= 2);
  REQUIRE(m.transport.rx_frame_count >= 2);
}

TEST_CASE("Write to a data ID") {
  MasterApp m;
  SlaveApp s;

  m.run();
  s.run();

  auto rid = m.transport.tx(Frame(WriteData, m.year.nr, (uint16_t)0x07E7));
  REQUIRE(rid != NoRequestID);

  m.propagate();

  REQUIRE(m.transport.is_finished(rid));

  auto did = s.find(m.year.nr);
  REQUIRE(did);
  REQUIRE(did->value == 0x07E7);
}

TEST_CASE("Read data ID later") {
  MasterApp m;
  SlaveApp s;

  m.transport.disable_master_timer = true;

  m.run();
  s.run();

  auto rid = m.transport.tx(Frame(WriteData, m.year.nr, (uint16_t)0x07E7));
  REQUIRE(rid != NoRequestID);

  for (size_t i = 0; i < test_max_concurrent_requests - 1; i++) {
    while (m.transport.is_tx_queue_full())
      std::this_thread::sleep_for(10ms);
    auto rid2 = m.transport.tx(Frame(WriteData, m.date.nr, (uint16_t)0x0101));
    REQUIRE(rid2 != NoRequestID);
  }

  m.propagate();

  REQUIRE(m.transport.is_finished(rid));

  Frame f;
  auto status = m.transport.get(rid, f);
  REQUIRE(status == RequestStatus::OK);
  REQUIRE(f.id() == m.year.nr);
  REQUIRE(f.msg_type() == MsgType::WriteACK);

  auto did = s.find(m.year.nr);
  REQUIRE(did);
  REQUIRE(did->value == 0x07E7);
}

TEST_CASE("Read data ID too late") {
  MasterApp m;
  SlaveApp s;

  m.transport.disable_master_timer = true;

  m.run();
  s.run();

  auto rid = m.transport.tx(Frame(WriteData, m.year.nr, (uint16_t)0x07E7));
  REQUIRE(rid != NoRequestID);

  for (size_t i = 0; i < test_max_concurrent_requests + 1; i++) {
    while (m.transport.is_tx_queue_full())
      std::this_thread::sleep_for(10ms);
    auto rid2 = m.transport.tx(Frame(WriteData, m.date.nr, (uint16_t)0x0101));
    REQUIRE(rid2 != NoRequestID);
  }

  m.propagate();

  if (m.transport.is_finished(rid)) {
    Frame f;
    REQUIRE(m.transport.get(rid, f) == RequestStatus::PURGED);
  }
}

TEST_CASE("Write to unknown data ID") {
  MasterApp m;
  SlaveApp s;

  m.transport.disable_master_timer = true;

  m.run();
  s.run();

  REQUIRE(s.find(128) == nullptr);

  auto rid = m.transport.tx(Frame(WriteData, 128, (uint16_t)0x1234));
  REQUIRE(rid != NoRequestID);
  Frame f;
  auto status = m.transport.get(rid, f);
  REQUIRE(status == RequestStatus::PENDING);

  m.propagate();
  REQUIRE(m.transport.is_finished(rid));
  REQUIRE(s.find(128) == nullptr);

  status = m.transport.get(rid, f);
  REQUIRE(status == RequestStatus::OK);
  REQUIRE(f.id() == 128);
  REQUIRE(f.msg_type() == MsgType::UnknownDataID);
}

int main(int argc, char **argv) {
  doctest::Context context;
  context.applyCommandLine(argc, argv);

  pthread_setname_np(pthread_self(), "Main");
  pthread_setname_np(wire.native_handle(), "Wire");

  int res = context.run();

  kill_wire = true;
  wire.join();

  if (context.shouldExit())
    return res;

  // More?

  return res;
}