// CM Wintersteiger, 2022

#ifndef _LITT_OPENTHERM_TRANSPORT_H_
#define _LITT_OPENTHERM_TRANSPORT_H_

#include <inttypes.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// #include <thread>

#ifndef PRIu64
#define PRIu64 "llu"
#endif

#include <litt/ds.h>
#include <litt/serialization.h>

// OpenTherm 2.2 transport layer

namespace litt {
namespace OpenTherm {

struct Frame;

class Application;

template <typename T> class IO {
public:
  typedef T PinsType;

  IO(const PinsType &pins) : pins(pins) {}
  virtual ~IO() = default;
  virtual void put(const Frame &) = 0;
  virtual void put(Frame &&f) { put(f); }
  virtual Frame get_blocking() = 0;
  virtual bool get_timeout(Frame &f, uint64_t timeout_us) = 0;

  virtual void log(const char *fmt, ...) = 0;

protected:
  const PinsType &pins;
};

enum MsgType : uint8_t {
  // Master-to-Slave
  ReadData = 0b000,
  WriteData = 0b001,
  InvalidData = 0b010,
  // 0b011 reserved
  // Slave-to-Master
  ReadACK = 0b100,
  WriteACK = 0b101,
  DataInvalid = 0b110,
  UnknownDataID = 0b111
};

struct Frame {
  uint32_t data = 0;

  Frame() = default;

  Frame(uint32_t data) : data(data) {}

  Frame(MsgType msg_type, uint8_t id) {
    data = (static_cast<uint32_t>(msg_type) << 28) | (static_cast<uint32_t>(id) << 16);
  }

  Frame(MsgType msg_type, uint8_t id, int16_t value) {
    data = (static_cast<uint32_t>(msg_type) << 28) | (static_cast<uint32_t>(id) << 16) | value;
  }

  Frame(MsgType msg_type, uint8_t id, uint16_t value) {
    data = (static_cast<uint32_t>(msg_type) << 28) | (static_cast<uint32_t>(id) << 16) | value;
  }

  Frame(MsgType msg_type, uint8_t id, uint8_t db1, uint8_t db2) {
    data = (static_cast<uint32_t>(msg_type) << 28) | (static_cast<uint32_t>(id) << 16) |
           (static_cast<uint32_t>(db1) << 8) | db2;
  }

  Frame(MsgType msg_type, uint8_t id, float value) {
    data = (static_cast<uint32_t>(msg_type) << 28) | (static_cast<uint32_t>(id) << 16) | ((uint16_t)(value * 256.0f));
  }

  bool parity() const { return data >> 31 != 0; }
  MsgType msg_type() const { return static_cast<MsgType>((data >> 28) & 0x07); }
  uint8_t id() const { return (data >> 16) & 0xFF; }
  uint16_t value() const { return data & 0xFFFF; }
  uint8_t data_byte_1() const { return (data >> 8) & 0xFF; }
  uint8_t data_byte_2() const { return data & 0xFF; }

  bool compute_parity() const {
    bool r = false;
    uint32_t t = data & 0x7FFFFFFF;
    while (t != 0) {
      if (t % 2)
        r = !r;
      t >>= 1;
    }
    return r;
  }

  bool parity_ok() const { return compute_parity() == parity(); }

  operator uint32_t() const { return (data & 0x7FFFFFFF) | (compute_parity() ? 0x80000000 : 0); }

  const char *to_string() const {
    static char strbuf[32];
    const char *fn = "Unknown-Msg-Type";
    switch (msg_type()) {
    case ReadData:
      fn = "Read-Data";
      break;
    case WriteData:
      fn = "Write-Data";
      break;
    case InvalidData:
      fn = "Invalid-Data";
      break;
    case ReadACK:
      fn = "Read-Ack";
      break;
    case WriteACK:
      fn = "Write-Ack";
      break;
    case DataInvalid:
      fn = "Data-Invalid";
      break;
    case UnknownDataID:
      fn = "Unknown-DataID";
      break;
    }
    snprintf(strbuf, sizeof(strbuf), "%s(%d, %04x)", fn, id(), value());
    return strbuf;
  }
};

using RequestID = uint64_t;
static constexpr RequestID NoRequestID = UINT64_MAX;

enum class RequestStatus { OK = 0, TIMED_OUT, SKIPPED, ERROR, PURGED, PENDING };

struct Request {
  Request(RequestID id = NoRequestID, bool skip_if_busy = false, Frame f = {0},
          void (*callback)(Application *, RequestStatus, RequestID, const Frame &) = nullptr,
          Application *app = nullptr)
      : id(id), skip_if_busy(skip_if_busy), f(f), callback(callback), application(app) {}
  virtual ~Request() = default;

  RequestID id = NoRequestID;
  bool skip_if_busy = false;
  Frame f = {0};
  void (*callback)(Application *, RequestStatus, RequestID, const Frame &) = nullptr;
  Application *application = nullptr;
};

struct Response {
  RequestID id = NoRequestID;
  Frame f = {0};
  RequestStatus status = RequestStatus::ERROR;
};

class TransportBase {
public:
  TransportBase() = default;

  virtual ~TransportBase() = default;

  virtual void set_frame_callback(bool (*cb)(Application &, const Frame &), Application *obj) {
    frame_callback = cb;
    frame_callback_obj = obj;
  }

  virtual RequestID tx(const Frame &f, bool skip_if_busy = false,
                       void (*callback)(Application *, RequestStatus, RequestID rid, const Frame &) = nullptr,
                       Application *app = nullptr) = 0;

  virtual bool process(const Frame &f) {
    if (f.parity_ok()) {
      switch (f.msg_type()) {
      case ReadData:
      case WriteData:
      case InvalidData:
      case ReadACK:
      case WriteACK:
      case DataInvalid:
      case UnknownDataID:
        return frame_callback ? frame_callback(*frame_callback_obj, f) : true;
      default:
        return false;
      }
    }
    return false;
  }

protected:
  bool (*frame_callback)(Application &, const Frame &) = nullptr;
  Application *frame_callback_obj = nullptr;
};

template <typename TimerType, typename MutexType, typename SemaphoreType, typename TimeType,
          template <typename> class QueueType, typename IOType, size_t max_concurrent_requests = 16>
class Transport : public TransportBase {
public:
#pragma pack(push, 1)
  struct Statistics {
    uint64_t frames_dropped = 0, frames_late = 0;

    bool serialize(uint8_t *&buf, size_t &sz) const {
      using litt::serialize;
      return serialize(frames_dropped, buf, sz) && serialize(frames_late, buf, sz);
    }

    bool deserialize(const uint8_t *&buf, size_t &sz) {
      using litt::deserialize;
      bool r = deserialize(frames_dropped, buf, sz) && deserialize(frames_late, buf, sz);
      if (!r)
        *this = Statistics();
      return r;
    }

    size_t serialized_size() const { return sizeof(frames_dropped) + sizeof(frames_late); }
  };
#pragma pack(pop)

  Transport(const typename IOType::PinsType &pins_, void (*rx_fblink)(bool) = nullptr,
            void (*tx_fblink)(bool) = nullptr)
      : io(pins_, rx_fblink, tx_fblink), rx_frame_count(0) {}

  virtual ~Transport() {}

  TimeType time;
  IOType io;
  Frame rx_last;
  uint64_t rx_frame_count = 0, tx_frame_count = 0;

  virtual void start() = 0;

  virtual void on_opentherm_supported() {}

  virtual void on_opentherm_unsupported() = 0;

  volatile bool keep_running = true;

  virtual void abort() { keep_running = false; }

  struct RXResult {
    Frame f;
    bool ok;
  };

  virtual RXResult rx_once() = 0;

  void rx_forever() {
    rx_until([]() { return false; });
  }

  void rx_until(bool (*fstop)()) {
    while (!fstop() && keep_running) {
      auto rxr = rx_once();
      if (rxr.ok)
        process(rxr.f);
    }
  }
};

template <typename TimerType, typename MutexType, typename SemaphoreType, typename TimeType,
          template <typename> class QueueType, typename IOType, size_t max_concurrent_requests = 16>
class Master
    : public Transport<TimerType, MutexType, SemaphoreType, TimeType, QueueType, IOType, max_concurrent_requests> {
protected:
  using DeviceT = Transport<TimerType, MutexType, SemaphoreType, TimeType, QueueType, IOType, max_concurrent_requests>;
  using DeviceT::io;
  using DeviceT::keep_running;
  using DeviceT::rx_last;
  using DeviceT::time;

public:
  using DeviceT::frame_callback;
  using DeviceT::frame_callback_obj;
  using DeviceT::rx_forever;
  using DeviceT::rx_frame_count;
  using DeviceT::tx;
  using DeviceT::tx_frame_count;

  bool disable_master_timer = false;

  typename DeviceT::Statistics statistics;

  Master(const typename IOType::PinsType &pins_, void (*rx_fblink)(bool) = nullptr, void (*tx_fblink)(bool) = nullptr)
      : DeviceT(pins_, rx_fblink, tx_fblink), master_timer(
                                                  0, 1000000,
                                                  [](Timer *, void *data) {
                                                    // LOG_INF("master timer tick");
                                                    ((Master *)data)->next_master_msg();
                                                    return true;
                                                  },
                                                  [](Timer *, void *data) {
                                                    LOG_ERR("master timer stopped");
                                                    return true;
                                                  }, this),
        plus_check_timer(
            0, 20000000,
            [](Timer *timer, void *data) {
              timer->stop();
              return false;
            },
            [](Timer *timer, void *data) {
              DeviceT *d = static_cast<DeviceT *>(data);
              if (d->rx_frame_count == 0)
                d->on_opentherm_unsupported();
              else
                d->on_opentherm_supported();
              return true;
            },
            this),
        tx_sem_release_timer(
            0, 0,
            [](Timer *timer, void *data) {
              Master *d = static_cast<Master *>(data);
              timer->stop(false);
              d->tx_sem.release();
              return true;
            },
            nullptr, this),
        next_request_id(0), requests(max_concurrent_requests), status(0x02) {
    for (size_t i = 0; i < max_concurrent_requests; i++)
      responses[i].status = RequestStatus::ERROR;
  }

  virtual ~Master() = default;

  virtual void abort() override {
    DeviceT::abort();
    requests.abort();
  }

  virtual void on_opentherm_unsupported() override { master_timer.stop(); }

  virtual void start() override {
    tx_sem.release();
    rx_sem.release();
    if (!disable_master_timer)
      master_timer.start(1000000);
    plus_check_timer.start(20000000);
  }

  virtual bool ch_enabled() { return status & 0x01; }
  virtual bool dhw_enabled() { return status & 0x02; }
  virtual bool cooling_enabled() { return status & 0x04; }
  virtual bool otc_active() { return status & 0x08; }
  virtual bool ch2_enabled() { return status & 0x10; }

  virtual bool summer_mode() { return status & 0x20; }  // Not in OT 2.2
  virtual bool dhw_blocking() { return status & 0x40; } // Not in OT 2.2

  void set_status(uint8_t s) {
    uint8_t before = status;
    status = s;
    if (before != s)
      tx_status();
  }

  const uint8_t &get_status(void) const { return status; }

  void set_ch(bool enabled) {
    uint8_t before = status;
    if (enabled)
      status |= 0x01;
    else
      status &= ~0x01;
    if (before != status)
      tx_status();
  }

  void set_dhw(bool enabled) {
    uint8_t before = status;
    if (enabled)
      status |= 0x02;
    else
      status &= ~0x02;
    if (before != status)
      tx_status();
  }

  void set_ch2(bool enabled) {
    uint8_t before = status;
    if (enabled)
      status |= 0x10;
    else
      status &= ~0x10;
    if (before != status)
      tx_status();
  }

  void set_otc(bool enabled) {
    uint8_t before = status;
    if (enabled)
      status |= 0x08;
    else
      status &= ~0x08;
    if (before != status)
      tx_status();
  }

  virtual void next_master_msg() { tx(Frame(OpenTherm::MsgType::ReadData, 0x00, status, 0x00), true); }

  virtual typename DeviceT::RXResult rx_once() override {
    typename DeviceT::RXResult r = { Frame(0), false };

#ifdef NONBLOCKING_MASTER
    do {
      r.ok = io.get_timeout(r.f, 1e5);
    } while (!r.ok && keep_running);

    if (r.ok) {
      // rx_tx_mtx.lock();
      rx_last = r.f;
      rx_frame_count++;
      // rx_tx_mtx.unlock();
      rx_sem.release();
    }
#else
    r = {io.get_blocking(), true};
    rx_last = r.f;
    rx_frame_count++;
    rx_sem.release();
#endif

    return r;
  }

  RequestStatus converse(const Request &req, Frame &reply) {
    uint64_t frame_time = 0;
    volatile bool acquired = false;
    const uint64_t max_frame_time = 1000000;

    if (req.skip_if_busy && !tx_sem.try_acquire())
      return RequestStatus::SKIPPED;

    if (!disable_master_timer)
      master_timer.stop(false);

    io.set_cur_req_id(req.id);
    // LOG_INF("TX %lld: %08x", req.id, req.f.data);

    {
      if (!req.skip_if_busy)
        tx_sem.acquire_blocking();
      rx_sem.acquire_blocking();

      uint64_t start_time = time.get_us();
      io.put(req.f);

      // After 34ms frame time, at least 20ms and at most 800ms before we can
      // expect the reply to start. Add 34ms for the reply for a total of 800+34+34=868.
      // (Max 1.15 sec between sends specified.)
      acquired = rx_sem.acquire_timeout(max_frame_time - 34000);
      frame_time = time.get_us() - start_time;

      if (acquired) {
        // rx_tx_mtx.lock();
        reply = rx_last;
        // rx_tx_mtx.unlock();
      }

      // At least 100 ms before the next send.
      tx_sem_release_timer.start(1e5);
      rx_sem.release();
    }

    if (!acquired)
      on_dropped_frame(req.id);
    else if (frame_time > max_frame_time)
      // This path should not be reached if the slave behaves according to the specification. If it is reached, it's
      // likely due to imprecise time measurement or timer triggering.
      on_late_frame(req.id);

    if (!disable_master_timer) {
      if (frame_time < max_frame_time)
        master_timer.start(max_frame_time - frame_time);
      else {
        master_timer.tick();
        master_timer.start(1e6);
      }
    }

    tx_frame_count++;

    return acquired ? RequestStatus::OK : RequestStatus::TIMED_OUT;
  }

  virtual bool process(const Frame &f) override {
    if (f.parity_ok()) {
      switch (f.msg_type()) {
      case ReadACK:
      case WriteACK:
      case DataInvalid:
      case UnknownDataID:
        break;
      default:
        io.log("unexpected message type: %d", f.msg_type());
        return false;
      }
      return frame_callback ? frame_callback(*frame_callback_obj, f) : true;
    } else
      io.log("Master: parity check failed: %08" PRIx32 " (should be %08" PRIx32 ")", f.data, (uint32_t)f);
    return false;
  }

  virtual RequestID tx(const Frame &f, bool skip_if_busy = false,
                       void (*callback)(Application *, RequestStatus, RequestID, const Frame &) = nullptr,
                       Application *app = nullptr) override {
    if (next_request_id == NoRequestID)
      next_request_id++;
    Request req(next_request_id++, skip_if_busy, f, callback, app);
    auto rinx = req.id % max_concurrent_requests;
    Response &rsp = responses[rinx];
    rsp.id = req.id;
    rsp.status = RequestStatus::PENDING;
    if (!requests.try_add(req)) {
      rsp.status = RequestStatus::ERROR;
      return NoRequestID;
    } else
      return req.id;
  }

  bool is_finished(uint64_t rid) {
    Response &r = responses[rid % max_concurrent_requests];
    return r.status != RequestStatus::PENDING;
  }

  unsigned num_pending_requests() const {
    // return requests.level();

    unsigned r = 0;
    for (const auto &e : responses)
      if (e.status == RequestStatus::PENDING)
        r++;
    return r;
  }

  bool is_tx_queue_full() const { return requests.level() >= max_concurrent_requests; }

  RequestStatus get(uint64_t rid, Frame &f) {
    auto rinx = rid % max_concurrent_requests;
    // std::cout << "M: get: rinx=" << rinx << std::endl;
    Response &r = responses[rinx];
    if (r.id != rid)
      return RequestStatus::PURGED;
    if (r.status != RequestStatus::PENDING)
      f = r.f;
    return r.status;
  }

  void tx_forever() {
    tx_until([]() { return false; });
  }

  void tx_until(bool (*fstop)()) {
    while ((!fstop() || !requests.empty()) && keep_running) {
      auto req = requests.remove();

      if (keep_running) {
        auto rinx = req.id % max_concurrent_requests;
        Response &rsp = responses[rinx];
        rsp.id = req.id;

        RequestStatus status = converse(req, rsp.f);

        if (req.callback && req.application)
          req.callback(req.application, status, rsp.id, rsp.f);

        rsp.status = status;
      }
    }
  }

  virtual void on_dropped_frame(RequestID rid) { statistics.frames_dropped++; }

  virtual void on_late_frame(RequestID rid) { statistics.frames_late++; }

protected:
  TimerType master_timer, plus_check_timer, tx_sem_release_timer;
  SemaphoreType tx_sem, rx_sem;
  MutexType rx_tx_mtx;

  RequestID next_request_id = 0;
  QueueType<Request> requests;
  Response responses[max_concurrent_requests];

  uint8_t status;

  void tx_status() { tx(Frame(ReadData, 0, status, 0x00)); }
};

template <typename TimerType, typename MutexType, typename SemaphoreType, typename TimeType,
          template <typename> class QueueType, typename IOType, size_t max_concurrent_requests = 16>
class Slave
    : public Transport<TimerType, MutexType, SemaphoreType, TimeType, QueueType, IOType, max_concurrent_requests> {
  using DeviceT = Transport<TimerType, MutexType, SemaphoreType, TimeType, QueueType, IOType, max_concurrent_requests>;

public:
  using DeviceT::frame_callback;
  using DeviceT::frame_callback_obj;
  using DeviceT::io;
  using DeviceT::keep_running;
  using DeviceT::rx_frame_count;
  using DeviceT::rx_last;
  using DeviceT::Statistics;
  using DeviceT::time;
  using DeviceT::tx;

  Slave(const typename IOType::PinsType &pins_, void (*rx_fblink)(bool) = nullptr, void (*tx_fblink)(bool) = nullptr)
      : DeviceT(pins_, rx_fblink, tx_fblink) {}

  virtual ~Slave() = default;

  virtual void start() override {}

  virtual typename DeviceT::RXResult rx_once() override {
    typename DeviceT::RXResult r;

#ifdef NONBLOCKING_SLAVE
    do {
      r.ok = io.get_timeout(r.f, 1e5);
    } while (!r.ok && keep_running);

    if (r.ok) {
      rx_last = r.f;
      rx_frame_count++;
    }
#else
    r = {io.get_blocking(), true};
    rx_last = r.f;
    rx_frame_count++;
#endif

    // if (r.ok) std::cout << time.get_us() << " S: rx_once: frame received by " << std::this_thread::get_id() <<
    // std::endl;

    return r;
  }

  virtual bool process(const Frame &f) override {
    if (f.parity_ok()) {
      switch (f.msg_type()) {
      case ReadData:
      case WriteData:
      case InvalidData:
        return frame_callback ? frame_callback(*frame_callback_obj, f) : true;
      default:
        io.log("unexpected message type: %d", f.msg_type());
        return false;
      }
    } else
      io.log("Slave: parity check failed: %08" PRIx32 " (should be %08" PRIx32 ")", f.data, (uint32_t)f);
    return false;
  }

  virtual RequestID tx(const Frame &f, bool skip_if_busy = false,
                       void (*callback)(Application *, RequestStatus, RequestID, const Frame &) = nullptr,
                       Application *app = nullptr) override {
    // std::cout << "S: tx: " << f.to_string() << std::endl;
    io.put(f);
    if (next_request_id == NoRequestID)
      next_request_id++;
    if (callback)
      callback(app, RequestStatus::OK, next_request_id, f);
    return next_request_id++;
  }

protected:
  RequestID next_request_id = 0;
};

} // namespace OpenTherm
} // namespace litt

#endif // _LITT_OPENTHERM_TRANSPORT_H_
