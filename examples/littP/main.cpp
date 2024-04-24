#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "lwipopts.h"

#include <boards/pico_w.h>

#include <hardware/clocks.h>
#include <hardware/exception.h>
#include <hardware/flash.h>
#include <hardware/pll.h>
#include <hardware/rtc.h>
#include <hardware/timer.h>
#include <hardware/watchdog.h>
#include <pico/cyw43_arch.h>
#include <pico/cyw43_arch/arch_threadsafe_background.h>
#include <pico/multicore.h>
#include <pico/mutex.h>
#include <pico/platform.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>

#include <lwip/apps/fs.h>
#include <lwip/apps/httpd.h>
#include <lwip/apps/sntp.h>
#include <lwip/stats.h>
#include <lwip/sys.h>
#include <lwip/tcpip.h>
#include <lwip/timeouts.h>

#include <litt/opentherm/application.h>
#include <litt/opentherm/boiler_interface.h>
#include <litt/opentherm/transport.h>
#include <litt/pid.h>
#include <litt/thermostat.h>

#include "pico_litt_ds.h"
#include "pico_litt_io.h"
#include "pico_mqtt_client.h"

using namespace litt;
using namespace litt::OpenTherm;

alarm_pool_t *PicoTimer::alarm_pool = NULL;

auto_init_recursive_mutex(log_mtx);

void vllog(const char *fmt, va_list args) {
  recursive_mutex_enter_blocking(&log_mtx);
  printf("[%012llu] ", time_us_64());
  vprintf(fmt, args);
  printf("\n");
  fflush(stdout);
  recursive_mutex_exit(&log_mtx);
}

void llog(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vllog(fmt, args);
  va_end(args);
}

static void hard_fault_exception_handler(void) {
  ::llog("EXCEPTION: HARD FAULT ON CORE %d", get_core_num());
  watchdog_enable(10, true);
  while (true)
    ;
}

static uint64_t get_flash_size() {
  static constexpr const size_t flash_cmd_size = 4;
  uint8_t txbuf[flash_cmd_size] = {0x9f};
  uint8_t rxbuf[flash_cmd_size] = {0x00};
  flash_do_cmd(txbuf, rxbuf, flash_cmd_size);
  return 1 << rxbuf[3];
}

typedef MQTTClient<CommandFrame::serialized_size(), 16> TMQTTClient;

#define MAX_TEMPERATURES 10

static char all_temperature_topics[sizeof(TEMPERATURE_TOPICS)];

class MyTransport : public Master<PicoTimer, PicoMutex, PicoSemaphore, PicoTime, PicoQueue, PicoIO> {
public:
  enum State { INIT, SLAVE_INFO, NORMAL };
  State state = INIT;

  MyTransport(const PicoPins &pins) : Master(pins), state(INIT), master_index(0), flow_setpoint(0.0f) {}

  virtual ~MyTransport() = default;

  Frame init_master_frames[2] = {Frame(ReadData, 3), Frame(ReadData, 0)};

  Frame slave_info_frames[7] = {Frame(ReadData, 3),       Frame(ReadData, 125), Frame(ReadData, 15),
                                Frame(ReadData, 127),     Frame(ReadData, 49),  Frame(ReadData, 57),
                                Frame(WriteData, 1, 0.0f)};

  Frame master_frames[10] = {Frame(ReadData, 0),  Frame(ReadData, 10),      Frame(ReadData, 17), Frame(ReadData, 18),
                             Frame(ReadData, 19), Frame(ReadData, 25),      Frame(ReadData, 27), Frame(ReadData, 28),
                             Frame(ReadData, 35), Frame(WriteData, 1, 0.0f)};

  void advance() {
    auto next = [this](size_t sz, State next_state) {
      master_index = (master_index + 1) % (sz / sizeof(Frame));
      if (master_index == 0)
        state = next_state;
      return master_index;
    };

    switch (state) {
    case INIT:
      master_index = next(sizeof(init_master_frames), SLAVE_INFO);
      if (rx_frame_count == 0)
        state = INIT;
      break;
    case SLAVE_INFO:
      master_index = next(sizeof(slave_info_frames), NORMAL);
      break;
    case NORMAL:
      master_index = next(sizeof(master_frames), NORMAL);
      break;
    }
  }

  virtual void next_master_msg() override {
    Frame f;

    switch (state) {
    case INIT:
      f = init_master_frames[master_index];
      break;
    case SLAVE_INFO:
      f = slave_info_frames[master_index];
      break;
    case NORMAL:
      f = master_frames[master_index];
    }

    switch (f.id()) {
    case 0:
      f = Frame(ReadData, f.id(), status, 0x00);
      break;
    case 1:
      if (state == NORMAL)
        f = Frame(f.msg_type(), f.id(), flow_setpoint);
      break;
    }

    tx(f, true);
    advance();
  }

  virtual void on_opentherm_supported() override {
    llog("> OpenTherm/plus support detected (%" PRIu64 " frames).", rx_frame_count);
  }

  virtual void on_opentherm_unsupported() override {
    llog("X No OpenTherm/plus reply after startup; the slave does not seem to "
         "support OpenTherm/plus.");
#ifdef NDEBUG
    llog("OpenTherm/- not supported; giving up.");
    Master::on_opentherm_unsupported();
#else
    llog("> Not giving up, though.");
#endif
  }

  virtual void on_dropped_frame(RequestID rid) override { llog("Frame %" PRIu64 " dropped", rid); }

  virtual void on_late_frame(RequestID rid) override { llog("Frame %" PRIu64 " late", rid); }

  void set_flow_setpoint(float temperature) { flow_setpoint = temperature; }

protected:
  size_t master_index;
  float flow_setpoint;
};

using MyThermostat = PIDDrivenThermostat<MAX_TEMPERATURES, PicoTime, PicoTimer>;

class MyApp : public RichApplication, public MyThermostat, public TMQTTClient {
#pragma pack(push, 1)
  struct Configuration {
    Configuration(Thermostat::Configuration &thermostat) :
      thermostat(thermostat)
    {
      strcpy(all_temperature_topics, TEMPERATURE_TOPICS);
      char *p = strtok(all_temperature_topics, ",");
      while (p != NULL) {
        char *pipe = strchr(p, '|');
        *pipe = '\0';
        mqtt.temperature_topics[num_temperatures] = p;
        mqtt.temperature_names[num_temperatures++] = pipe + 1;
        p = strtok(NULL, ",");
      }
    }

    struct Wifi {
      char ssid[33] = WIFI_SSID;
      char password[64] = WIFI_PASSWORD;
      uint32_t auth = WIFI_AUTH;
    };

    Wifi wifi;

    struct MQTT {
      char host[32] = MQTT_BROKER_HOST;
      char client_id[32] = MQTT_CLIENT_ID;
      char client_user[32] = MQTT_USER;
      char client_pass[32] = MQTT_PASS;

      char frame_topic[64] = MQTT_CLIENT_ID "/frames";
      char command_in_topic[64] = MQTT_CLIENT_ID "/cmd/in";
      char command_out_topic[64] = MQTT_CLIENT_ID "/cmd/out";
      char *temperature_topics[MAX_TEMPERATURES];
      char *temperature_names[MAX_TEMPERATURES];
    };

    MQTT mqtt;

    size_t num_temperatures = 0;

    Thermostat::Configuration &thermostat;

    bool serialize(uint8_t *buf, size_t sz) {
#define PSZ p, sz - (p - (char *)buf)
      char *p = (char *)buf;
      p += snprintf(PSZ, "%s", wifi.ssid);
      p += snprintf(PSZ, "%s", wifi.password);
      p += snprintf(PSZ, "%lu", wifi.auth);
      p += snprintf(PSZ, "%s", mqtt.host);
      p += snprintf(PSZ, "%s", mqtt.client_id);
      p += snprintf(PSZ, "%s", mqtt.client_user);
      p += snprintf(PSZ, "%s", mqtt.client_pass);
      p += snprintf(PSZ, "%s", mqtt.frame_topic);
      p += snprintf(PSZ, "%s", mqtt.command_in_topic);
      p += snprintf(PSZ, "%s", mqtt.command_out_topic);
#undef PSZ
      // TODO: refactor and add thermostat configuration.
      return ((size_t)(p - (char *)buf)) <= sz;
    }

    bool deserialize(const uint8_t *buf, size_t sz) {
      // TODO
      return true;
    }
  };
#pragma pack(pop)

public:
  Configuration configuration;

  MyApp(const PicoPins &pins)
      : RichApplication(transport),
        MyThermostat(boiler.ch1, {2.75f, 0.0f, 0.5f, 0.25f, 0.5f}, 21.0f), TMQTTClient(),
        configuration(MyThermostat::configuration),
        transport(pins),
        boiler(transport, *this),
        flash_size(get_flash_size()),
        wifi_link_timer(0, 1000000, on_wifi_link_timer_cb, nullptr, this) {
    mutex_init(&flash_mtx);
    transport.set_frame_callback(RichApplication::sprocess, this);
  }

  virtual ~MyApp() = default;

  virtual void run() override {
    transport.start();

    if (!wifi_connect())
      llog("Could not connect to wifi.");

    httpd_init();

    transport.rx_forever([](bool on) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0); });
  }

  PicoTime time;

  virtual void on_spike_protect(bool on) override { llog("> spike protect := %s", (on ? "on" : "off")); }

  virtual void on_fault_indication() override {
    // Slave indicates a fault; try to get a fault code.
    transport.tx(Frame(ReadData, 5));
  }

  virtual void on_flame_change(bool on) override {
    llog("> flame := %s", on ? "on" : "off");
    Thermostat::on_flame_change(on);
  }

  virtual void on_max_flow_setpoint_bounds_change(const FlowSetpointBounds &from,
                                                  const FlowSetpointBounds &to) override {
    // Max TSet upper/lower bounds
    // Check: Is lb the lower bound of Max T_set or of T_set?
    set_max_flow_setpoint_bounds(0.0f, to.upper_bound);
    transport.tx(Frame(WriteData, maxtset.nr, to_f88(to.upper_bound << 8)));
  }

  virtual void on_flow_temperature_change(float from, float to) override {
    auto fsp = boiler.ch1.flow_setpoint();
    check_spike_protect(from, to, fsp, fsp);
  }

  virtual void on_read_ack(uint8_t data_id, uint16_t data_value) override {
    RichApplication::on_read_ack(data_id, data_value);
    slave_id_infos[data_id].seen = true;
  }

  virtual void on_write_ack(uint8_t data_id, uint16_t data_value) override {
    RichApplication::on_write_ack(data_id, data_value);
    slave_id_infos[data_id].seen = true;
  }

  virtual void on_unknown_data_id(uint8_t data_id, uint16_t data_value) override {
    RichApplication::on_unknown_data_id(data_id, data_value);
    slave_id_infos[data_id].unsupported = true;
  }

  MyTransport &dev() { return transport; }

  virtual bool process(const Frame &f) override {
    if (!RichApplication::process(f))
      return false;

    mqtt_publish_queue_add(configuration.mqtt.frame_topic, f);
    MQTTClient::publish_all_queued(); // Note: should this be done elsewhere?
    return true;
  }

  void tx_forever() { transport.tx_forever(); }

  struct SlaveIDInfo {
    bool seen = false;
    bool unsupported = false;
  };

  SlaveIDInfo slave_id_infos[256] = {{0}};

  size_t tx_frame_count() const { return transport.tx_frame_count; }
  size_t rx_frame_count() const { return transport.rx_frame_count; }

  const PIDController<PicoTime> &pid(uint8_t pid_id) const { return MyThermostat::pids[pid_id]; }

  // void on_real_time_available(bool available) { real_time.set_available(available); }

protected:
  MyTransport transport;
  OpenTherm::BoilerInterface<MyTransport> boiler;

  const uint64_t flash_size = 0;
  mutex_t flash_mtx;

  virtual void on_flow_setpoint_change(float from, float to, bool approximated) override {
    llog("> setpoint := %0.2f %s", to, (approximated ? "(approx.)" : ""));
    Thermostat::on_flow_setpoint_change(from, to, approximated);
  }

  virtual bool execute(const CommandFrame &f) override {
    using namespace OpenTherm;

    auto cmd_id = f.command_id;
    switch (cmd_id) {
    case CommandID::INVALID:
      return on_invalid_cmd(f.user_data, f.payload);
    case CommandID::GET_VERSION:
      return on_get_version_cmd(f.user_data);
    case CommandID::OPENTHERM_REQUEST:
      return on_opentherm_request_cmd(f.user_data, Frame(f.payload));
    case CommandID::SET_OPENTHERM_STATUS:
      return on_set_opentherm_status_cmd(f.user_data, f.payload);
    case CommandID::LOAD_CONFIG:
      return on_load_config_cmd(f.user_data);
    case CommandID::SAVE_CONFIG:
      return on_save_config_cmd(f.user_data);

    default:
      return MyThermostat::execute(f);
    }
  }

  virtual bool on_opentherm_request_cmd(uint16_t user_data, Frame frame) {
    RequestID rid = transport.tx(
        frame, false,
        [](Application *a, RequestStatus s, RequestID rid, const Frame &f) {
          MyApp *app = static_cast<MyApp *>(a);
          auto &req = app->mqtt_cmd_requests[rid % TMQTTClient::max_concurrent_requests];
          if (req.rid != rid)
            llog("warning: outdated MQTT request ID");
          req.rid = NoRequestID;
          if (s == RequestStatus::OK)
            app->mqtt_publish_cmd_response(CommandID::OPENTHERM_REPLY, req.user_data, f);
          else
            app->mqtt_publish_cmd_response(req.user_data, s);
        },
        this);

    if (rid != NoRequestID) {
      mqtt_cmd_requests[rid % TMQTTClient::max_concurrent_requests] = {.rid = rid, .user_data = user_data};
      mqtt_publish_cmd_response(user_data, RequestStatus::OK);
    } else
      mqtt_publish_cmd_response(user_data, RequestStatus::ERROR);

    return rid != NoRequestID;
  }

  virtual bool on_invalid_cmd(uint16_t user_data, uint32_t payload) {
    mqtt_publish_cmd_response(user_data, RequestStatus::ERROR);
    return true;
  }

  virtual bool on_get_version_cmd(uint16_t user_data) {
    mqtt_publish_cmd_response(CommandID::VERSION_REPLY, user_data, api_version);
    return true;
  }

  virtual void set_opentherm_status(uint8_t status) {
    llog("> status := %02" PRIx8, status);
    transport.set_status(status);
    if (status & 0x01)
      MyThermostat::on_spike_protect(false);
  }

  virtual bool on_set_opentherm_status_cmd(uint16_t user_data, uint8_t status) {
    set_opentherm_status(status);
    mqtt_publish_cmd_response(user_data, RequestStatus::OK);
    return true;
  }

  virtual bool on_set_flow_setpoint_cmd(uint16_t user_data, float flow_setpoint) override {
    llog("> setpoint := %0.2f", flow_setpoint);
    Thermostat::on_set_flow_setpoint_cmd(user_data, flow_setpoint);
    mqtt_publish_cmd_response(user_data, RequestStatus::OK);
    return true;
  }

  virtual bool on_temperature_report_cmd(uint16_t user_data, uint8_t pid_id, float t) override {
    llog("> report(%u, %0.2f)", pid_id, t);
    bool r = MyThermostat::on_temperature_report_cmd(user_data, pid_id, t);
    mqtt_publish_cmd_response(user_data, r ? RequestStatus::OK : RequestStatus::ERROR);
    return true;
  }

  virtual bool on_set_pid_setpoint_cmd(uint16_t user_data, uint8_t pid_id, float room_setpoint) override {
    llog("> pids[%u].setpoint := %0.2f", pid_id, room_setpoint);
    bool r = MyThermostat::on_set_pid_setpoint_cmd(user_data, pid_id, room_setpoint);
    mqtt_publish_cmd_response(user_data, r ? RequestStatus::OK : RequestStatus::ERROR);
    return true;
  }

  virtual bool on_set_pid_kp_cmd(uint16_t user_data, uint8_t pid_id, float value) override {
    llog("> pids[%u].kp := %0.2f", pid_id, value);
    bool r = MyThermostat::on_set_pid_kp_cmd(user_data, pid_id, value);
    mqtt_publish_cmd_response(user_data, r ? RequestStatus::OK : RequestStatus::ERROR);
    return true;
  }

  virtual bool on_set_pid_ki_cmd(uint16_t user_data, uint8_t pid_id, float value) override {
    llog("> pids[%u].ki := %0.2f", pid_id, value);
    bool r = MyThermostat::on_set_pid_ki_cmd(user_data, pid_id, value);
    mqtt_publish_cmd_response(user_data, r ? RequestStatus::OK : RequestStatus::ERROR);
    return true;
  }

  virtual bool on_set_pid_kd_cmd(uint16_t user_data, uint8_t pid_id, float value) override {
    llog("> pids[%u].kd := %0.2f", pid_id, value);
    bool r = MyThermostat::on_set_pid_kd_cmd(user_data, pid_id, value);
    mqtt_publish_cmd_response(user_data, r ? RequestStatus::OK : RequestStatus::ERROR);
    return true;
  }

  virtual bool on_set_pid_cum_err_cmd(uint16_t user_data, uint8_t pid_id, float value) override {
    llog("> pids[%u].cum_err := %0.2f", pid_id, value);
    bool r = MyThermostat::on_set_pid_cum_err_cmd(user_data, pid_id, value);
    mqtt_publish_cmd_response(user_data, r ? RequestStatus::OK : RequestStatus::ERROR);
    return true;
  }

  virtual bool on_set_pid_weight_cmd(uint16_t user_data, uint8_t pid_id, float value) override {
    llog("> weights[%u] := %0.2f", pid_id, value);
    bool r = MyThermostat::on_set_pid_weight_cmd(user_data, pid_id, value);
    mqtt_publish_cmd_response(user_data, r ? RequestStatus::OK : RequestStatus::ERROR);
    return true;
  }

  virtual bool on_set_thermostat_mode_cmd(uint16_t user_data, Mode mode) override {
    llog("> mode := %s", mode_string(mode));
    bool r = MyThermostat::on_set_thermostat_mode_cmd(user_data, mode);
    mqtt_publish_cmd_response(user_data, r ? RequestStatus::OK : RequestStatus::ERROR);
    return true;
  }

  virtual bool on_set_pid_automatic_cmd(uint16_t user_data, uint8_t pid_id, float room_setpoint) override {
    llog("> pids[%u].automatic -> %0.2f", pid_id, room_setpoint);
    bool r = MyThermostat::on_set_pid_automatic_cmd(user_data, pid_id, room_setpoint);
    mqtt_publish_cmd_response(user_data, r ? RequestStatus::OK : RequestStatus::ERROR);
    return true;
  }

  virtual bool on_load_config_cmd(uint16_t user_data) {
    mutex_enter_blocking(&flash_mtx);
    const uint8_t *ptr = (uint8_t *)(XIP_BASE + flash_size - FLASH_SECTOR_SIZE);
    size_t space_remaining = sizeof(Configuration);
    configuration.deserialize(ptr, space_remaining);
    mutex_exit(&flash_mtx);
    return true;
  }

  virtual bool on_save_config_cmd(uint16_t user_data) {
    mutex_enter_blocking(&flash_mtx);
    static_assert(sizeof(Configuration) < 5 * FLASH_PAGE_SIZE);
    size_t pages = (sizeof(Configuration) / FLASH_PAGE_SIZE) + 1;
    size_t size = FLASH_PAGE_SIZE * pages;
    uint8_t buffer[size] = {0};
    // memcpy(buffer, &config, size);
    configuration.serialize(buffer, size);
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(flash_size - FLASH_SECTOR_SIZE, buffer, size);
    restore_interrupts(ints);
    mutex_exit(&flash_mtx);
    return true;
  }

protected: // MQTT
  struct MQTTRequest {
    RequestID rid = NoRequestID;
    uint16_t user_data = 0;
  };

  void mqtt_publish_queue_add(const char *topic, const Frame &f) {
    MQTTPublishRequest req;
    req.topic = topic;
    req.length = snprintf(req.payload, sizeof(req.payload), "%08" PRIx32, (uint32_t)f);
    MQTTClient::publish_queue_add(req);
  }

  MQTTRequest mqtt_cmd_requests[TMQTTClient::max_concurrent_requests];

  void mqtt_publish_cmd_response(CommandID cid, uint16_t user_id, uint32_t payload) {
    CommandFrame cf(cid, user_id, payload);

    MQTTPublishRequest req;
    req.topic = configuration.mqtt.command_out_topic;
    if (!cf.to_string(req.payload, sizeof(req.payload))) {
      llog("cf.to_string failed");
      snprintf(req.payload, sizeof(req.payload), "00%04x00000000", user_id);
    }
    req.length = cf.serialized_size();
    MQTTClient::publish_queue_add(req);
  }

  void mqtt_publish_cmd_response(uint16_t user_id, RequestStatus s) {
    CommandFrame cf(CommandID::CONFIRMATION, user_id, static_cast<uint32_t>(s));

    MQTTPublishRequest req;
    req.topic = configuration.mqtt.command_out_topic;
    if (!cf.to_string(req.payload, sizeof(req.payload))) {
      llog("cf.to_string failed");
      snprintf(req.payload, sizeof(req.payload), "00%04x00000000", user_id);
    }
    req.length = cf.serialized_size();
    MQTTClient::publish_queue_add(req);
  }

  virtual void on_message(const char *topic, const uint8_t *payload, size_t payload_len) override {
    if (strncmp(topic, configuration.mqtt.command_in_topic, sizeof(configuration.mqtt.command_in_topic)) == 0) {
      CommandFrame cf((const char *)payload, payload_len);
      if (!execute(cf))
        llog("command frame processing failed for message of size %u (cmd %u)", payload_len, cf.command_id);
    } else {
      for (size_t pid_id = 0; pid_id < configuration.num_temperatures; pid_id++) {
        if (strcmp(topic, configuration.mqtt.temperature_topics[pid_id]) == 0) {
          if (payload_len != sizeof(uint32_t))
            llog("unexpected MQTT data size: %u", payload_len);
          else {
            uint32_t n = 0;
            for (uint16_t i = 0; i != sizeof(uint32_t); i++)
              n = (n << 8) | payload[i];
            float temp = *reinterpret_cast<float *>(&n);
            if (temp <= -10.0f || temp >= +50.0f)
              llog("? suspicious report: n=%08x temperature=%0.2f", n, temp);
            llog("> report(%u, %0.2f)", pid_id, temp);
            if (!MyThermostat::on_temperature_report_cmd(0, pid_id, temp))
              llog("X report failed");
          }
        }
      }

      // llog("X Message on unhandled topic %s", topic);
    }
  }

protected: // Wifi
  bool wifi_link_down = true;
  PicoTimer wifi_link_timer;

  static bool on_wifi_link_timer_cb(Timer *timer, void *obj) {
    MyApp *app = static_cast<MyApp *>(obj);
    return app->on_wifi_link_timer(timer);
  };

  const char *link_status_string(int status) {
    switch (status) {
    case CYW43_LINK_DOWN:
      return "down";
    case CYW43_LINK_JOIN:
      return "connected";
    case CYW43_LINK_NOIP:
      return "connected, but no ip";
    case CYW43_LINK_UP:
      return "up";
    case CYW43_LINK_FAIL:
      return "connection failed";
    case CYW43_LINK_NONET:
      return "no matching SSID found";
    case CYW43_LINK_BADAUTH:
      return "authenticatation failure";
    default:
      return "unknown";
    }
  }

  bool on_network_down() {
    sntp_stop();
    return true;
  }

  bool on_network_up() {
    MQTTClient::Configuration cfg{
        configuration.mqtt.host,           configuration.mqtt.client_id, configuration.mqtt.client_user, configuration.mqtt.client_pass, {},
        configuration.num_temperatures + 1};
    cfg.topics[0] = configuration.mqtt.command_in_topic;
    for (size_t i = 0; i < configuration.num_temperatures; i++)
      cfg.topics[i + 1] = configuration.mqtt.temperature_topics[i];
    MQTTClient::initialize(cfg);

    ip_addr_t ip;
    ip4addr_aton("192.168.0.41", &ip);
    sntp_setserver(0, &ip);
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_init();

    return true;
  }

  bool on_wifi_link_timer(Timer *) {
    static int previous_ts = CYW43_LINK_NONET;

    int ts = cyw43_tcpip_link_status(&cyw43_state, 0);
    if (ts != previous_ts) {
      if (ts == CYW43_LINK_UP)
        llog("> wifi link: up, ip: %s", ipaddr_ntoa(&cyw43_state.netif[0].ip_addr));
      else
        llog("> wifi link: %s", link_status_string(ts));
    }

    if (previous_ts != ts && ts <= 0) {
      ts = cyw43_arch_wifi_connect_async(configuration.wifi.ssid, configuration.wifi.password, configuration.wifi.auth);
      wifi_link_down = true;
      on_network_down();
    }

    if (previous_ts != ts && ts == CYW43_LINK_UP) {
      wifi_link_down = false;
      on_network_up();
    }

    previous_ts = ts;
    return true;
  };

  bool wifi_connect() {
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
      llog("Wifi init failed.");
      return false;
    }

    llog("Connecting to '%s'", configuration.wifi.ssid);
    cyw43_wifi_pm(&cyw43_state, CYW43_AGGRESSIVE_PM);
    cyw43_arch_enable_sta_mode();
    wifi_link_timer.start();

    return true;
  }
};

// TODO: Static initialization is unsafe.
MyApp app({.rx = 27, .tx = 26, .owned = true, .tx_inverted = true});

extern "C" {
#define MAX_RESPONSE_SIZE 8 * 1024

void sntp_set_system_time_us(uint32_t sec, uint32_t usec) {
  // Note: apparently `sec` in UNIX epoch, not NTP epoch?
  time_t seconds_since_1970 = sec;

  struct tm *gm = gmtime(&seconds_since_1970);

  datetime_t t = {.year = static_cast<int16_t>(gm->tm_year + 1900),
                  .month = static_cast<int8_t>(gm->tm_mon + 1),
                  .day = static_cast<int8_t>(gm->tm_mday),
                  .dotw = static_cast<int8_t>(gm->tm_wday),
                  .hour = static_cast<int8_t>(gm->tm_hour),
                  .min = static_cast<int8_t>(gm->tm_min),
                  .sec = static_cast<int8_t>(gm->tm_sec)};

  if (!rtc_set_datetime(&t))
    llog("X failed to set RTC");

  // TODO
  // app.on_real_time_available(true);
}

struct FSCustomData {
  uint64_t request_id = -1;
  const char *status = NULL;
  char lbuf[MAX_RESPONSE_SIZE];
  size_t lbuf_sz = 0;
};

static int make_http_response(char *state, FSCustomData *data) {
  auto &sz = data->lbuf_sz;
  auto *status = data->status;

  if (!status && data->request_id != NoRequestID) {
    Frame response;
    switch (app.dev().get(data->request_id, response)) {
    case RequestStatus::OK:
      sz = snprintf(data->lbuf, sizeof(data->lbuf), "%08" PRIx32, (uint32_t)response);
      status = "200";
      break;
    case RequestStatus::TIMED_OUT:
      sz = snprintf(data->lbuf, sizeof(data->lbuf), "timed out");
      status = "408";
      break;
    default:
      sz = snprintf(data->lbuf, sizeof(data->lbuf), "request failed with status %d", (int)status);
      status = "500";
    }
  }

  return snprintf(state, MAX_RESPONSE_SIZE, "%s\n", data->lbuf);
}

size_t make_status_page(char *buf, size_t sz) {
#define PSZ p, sz - (p - buf)

  char *p = buf;
  p += snprintf(PSZ, "<!DOCTYPE html>");
  p += snprintf(PSZ, "<html>");
  p += snprintf(PSZ, "<head>"
                     "<title>Thermostat</title>"
                     "<style>"
                     "td {text-align: right}"
                     "td.txt {text-align: left}"
                     "table, th, td {border: 1px solid;}"
                     "table { display: inline-block }"
                     "</style>"
                     "</head>");

  p += snprintf(PSZ, "<body>");
  p += snprintf(PSZ, "<h2>Controller Status</h2>");

  p += snprintf(PSZ, "<table>");

  datetime_t hr_now;
  rtc_get_datetime(&hr_now);

  uint64_t time_us = time_us_64();
  uint64_t sec = time_us / 1000000;
  uint64_t min = sec / 60;
  sec %= 60;
  uint64_t hrs = min / 60;
  min %= 60;
  uint64_t days = hrs / 24;
  hrs %= 24;

  p += snprintf(PSZ, "<tr>");
  p += snprintf(PSZ,
                "<td class=\"txt\">Date/Time</td><td>%04u-%02u-%02u "
                "%02u:%02u:%02u</td></tr>",
                hr_now.year, hr_now.month, hr_now.day, hr_now.hour, hr_now.min, hr_now.sec);
  p += snprintf(PSZ,
                "<td class=\"txt\">Uptime</td><td>%02" PRIu64 ":%02" PRIu64 ":%02" PRIu64 ":%02" PRIu64 "</td></tr>",
                days, hrs, min, sec);
  p += snprintf(PSZ, "<tr><td class=\"txt\"># MQTT reconnects</td><td>%u</td></tr>", app.num_mqtt_reinit);
  p += snprintf(PSZ, "<tr><td class=\"txt\"># frames rx/tx</td><td>%u/%u</td></tr>", app.rx_frame_count(),
                app.tx_frame_count());
  p += snprintf(PSZ, "<tr><td class=\"txt\">Mode</td><td>%s</td></tr>", app.get_mode_string());
  p += snprintf(PSZ,
                "<tr><td class=\"txt\"># cycles "
                "tiny/short/normal</td><td>%llu/%llu/%llu</td></tr>",
                app.statistics.tiny_cycles, app.statistics.short_cycles, app.statistics.normal_cycles);
  p += snprintf(PSZ,
                "<tr><td class=\"txt\">Turn-on spike protection"
                "</td><td>%s</td></tr>",
                app.is_spike_protect_active() ? "on" : "off");
  p += snprintf(PSZ,
                "<tr><td class=\"txt\"># spike protection triggered"
                "</td><td>%lu</td></tr>",
                app.statistics.num_spike_protected);
  p += snprintf(PSZ,
                "<tr><td class=\"txt\">&Delta;T"
                "</td><td>%.2f &deg;C</td></tr>",
                to_f88(app.tboiler.value) - to_f88(app.tret.value));
  p += snprintf(PSZ, "</table>");

  p += snprintf(PSZ, "<h2>OpenTherm Data IDs</h2>");
  p += snprintf(PSZ, "<table>");
  p += snprintf(PSZ, "<tr><th>Data "
                     "ID</th><th>Raw</th><th>Value</th><th>Name</"
                     "th><th>Description</th></tr>");
  for (size_t i = 0; i < 256; i++)
    if (app.slave_id_infos[i].seen) {
      const auto *id = app.find(i);
      if (id) {
        p += snprintf(PSZ,
                      "<tr><td>%u</td><td>%04x</td><td>%s</td><td "
                      "class=\"txt\">%s</td><td class=\"txt\">",
                      i, id->value, (const char *)*id, id->data_object);
        for (const char *d = id->description; *d; d++) {
          switch (*d) {
          case '&':
            p += snprintf(PSZ, "&amp;");
            break;
          case '\xF8':
            p += snprintf(PSZ, "&deg;");
            break;
          case '\xE6':
            p += snprintf(PSZ, "&micro;");
            break;
          default:
            p += snprintf(PSZ, "%c", *d);
          }
        }
        p += snprintf(PSZ, "</td></tr>");
      }
    }
  p += snprintf(PSZ, "</table>");

  p += snprintf(PSZ, "<h2>PID Controllers</h2>");
  auto n = app.configuration.num_temperatures;
  p += snprintf(PSZ, "<table>");

  p += snprintf(PSZ, "<tr><th>Topic</th><th>Name</th><th>Age</th><th>Room</"
                     "th><th>Setpoint</th>"
                     "<th>kp</th><th>ki</th><th>kd</th><th>Min</th>"
                     "<th>Max</th><th>Output</th><th>Cum. "
                     "Error</th><th>Weight</th><th>Mode</th></tr>");
  uint64_t now = time_us_64();
  for (size_t i = 0; i < n; i++) {
    p += snprintf(PSZ, "<tr>");
    p += snprintf(PSZ, "<td class=\"txt\">%s</td>", app.configuration.mqtt.temperature_topics[i]);
    p += snprintf(PSZ, "<td class=\"txt\">%s</td>", app.configuration.mqtt.temperature_names[i]);
    p += snprintf(PSZ, "<td>%.2f</td>", (now - app.room_temperature_time(i)) / 1e6);
    p += snprintf(PSZ, "<td>%.2f</td>", app.room_temperature(i));
    p += snprintf(PSZ, "<td>%.2f</td>", app.pid(i).setpoint);
    p += snprintf(PSZ, "<td>%f</td>", app.pid(i).coefficients.kp);
    p += snprintf(PSZ, "<td>%f</td>", app.pid(i).coefficients.ki);
    p += snprintf(PSZ, "<td>%f</td>", app.pid(i).coefficients.kd);
    p += snprintf(PSZ, "<td>%.2f</td>", app.pid(i).min);
    p += snprintf(PSZ, "<td>%.2f</td>", app.pid(i).max);
    p += snprintf(PSZ, "<td>%.2f</td>", app.pid(i).output);
    p += snprintf(PSZ, "<td>%f</td>", app.pid(i).cum_err);
    p += snprintf(PSZ, "<td>%.2f</td>", app.pid_weight(i));
    p += snprintf(PSZ, "<td>%s</td>", app.pid_mode(i));
    p += snprintf(PSZ, "</tr>");
  }
  p += snprintf(PSZ, "</table>");

  p += snprintf(PSZ, "</body>");
  p += snprintf(PSZ, "</html>");
  return p - buf;
#undef PSZ
}

int fs_open_custom(struct fs_file *file, const char *name) {
  if (strcmp(name, "/status.ssi") == 0) {

    memset(file, 0, sizeof(struct fs_file));
    file->flags = FS_FILE_FLAGS_CUSTOM | FS_FILE_FLAGS_SSI;
    file->state = fs_state_init(file, name);
    file->len = 0;
    file->index = 0;

    auto data = (FSCustomData *)mem_malloc(sizeof(FSCustomData));
    if (!data) {
      llog("file data allocation failure");
      return 0;
    } else {
      data->request_id = NoRequestID;
      data->status = NULL;
      data->lbuf_sz = 0;
    }

    file->pextension = data;

    return 1;
  }
  return 0;
}

void fs_close_custom(struct fs_file *file) {
  if (file && file->pextension) {
    mem_free(file->pextension);
    file->pextension = NULL;
  }
}

void *fs_state_init(struct fs_file *file, const char *name) {
  void *r = mem_calloc(MAX_RESPONSE_SIZE, 1);
  if (!r)
    llog("FS state allocation failed");
  return r;
}

void fs_state_free(struct fs_file *file, void *state) {
  if (state)
    mem_free(state);
  file->state = NULL;
}

u8_t fs_canread_custom(struct fs_file *file) {
  auto data = (FSCustomData *)file->pextension;
  if (data == NULL)
    return 1;
  bool ready = true;
  if (data->request_id != NoRequestID)
    ready = app.dev().is_finished(data->request_id);
  if (ready && file->len == 0)
    file->len = make_http_response((char *)file->state, data);
  return ready ? 1 : 0;
}

u8_t fs_wait_read_custom(struct fs_file *file, fs_wait_cb callback_fn, void *callback_arg) { return 1; }

int fs_read_async_custom(struct fs_file *file, char *buffer, int count, fs_wait_cb callback_fn, void *callback_arg) {
  const char *from = (const char *)file->state + file->index;
  int rem = file->len - file->index;
  int len = count > rem ? rem : count;
  memcpy(buffer, from, len);
  file->index += count;
  return len;
}

void httpd_cgi_handler(struct fs_file *file, const char *uri, int num_params, char **keys, char **values, void *state) {
  if (state != NULL) {
    auto data = (FSCustomData *)file->pextension;
    if (data == NULL) {
      data->lbuf_sz = snprintf(data->lbuf, sizeof(data->lbuf), "missing extension data");
      data->status = "500";
    } else {
      if (strcmp(uri, "/status.ssi") == 0) {
        data->lbuf_sz = make_status_page(data->lbuf, sizeof(data->lbuf));
        data->status = "200";
      } else
        data->status = "404";
    }
  }
}
}

static void core1_main(void) {
  multicore_lockout_victim_init();
  exception_set_exclusive_handler(HARDFAULT_EXCEPTION, hard_fault_exception_handler);

  // pio_interrupt_clear(pio0, 0);
  // pio_set_irq0_source_enabled(pio0, pis_interrupt0, true);
  // irq_set_exclusive_handler(PIO0_IRQ_0, []() {
  //   if (pio_interrupt_get(pio0, 0)) {
  //     pio_interrupt_clear(pio0, 0);
  //     ::llog("irq: %08x; dropped frame", pio0->irq);
  //   }
  // });
  // irq_set_enabled(PIO0_IRQ_0, true);

  app.tx_forever();
}

static char input_buffer[1024] = {0};
static size_t input_buffer_inx = 0;

static bool on_input_timer_cb(Timer *timer, void *obj) {
  for (int c = getchar_timeout_us(0); c != PICO_ERROR_TIMEOUT; c = getchar_timeout_us(0)) {
    if (c != '\n' && c != '\r' && input_buffer_inx < sizeof(input_buffer)) {
      input_buffer[input_buffer_inx++] = c;
      input_buffer[input_buffer_inx] = '\0';
    } else {
      llog("* %s", input_buffer);
      input_buffer_inx = 0;
      input_buffer[0] = '\0';
    }
  }
  return true;
};

PicoTimer input_timer(0, 100000, on_input_timer_cb, nullptr, nullptr);

int main() {
  multicore_lockout_victim_init();
  exception_set_exclusive_handler(HARDFAULT_EXCEPTION, hard_fault_exception_handler);

  stdio_init_all();
  sleep_ms(2000);

  llog("littP starting...");
  llog("Build date: %s %s", __DATE__, __TIME__);

  llog("RP2040 chip version %u, rom version %u", rp2040_chip_version(), rp2040_rom_version());

  char board_id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
  pico_get_unique_board_id_string(board_id, sizeof(board_id));
  llog("Board ID: %s", board_id);

  // llog("Flash size: %llu bytes", get_flash_size()); // TODO: What's wrong with this?
  llog("Running at %.2f MHz", clock_get_hz(clk_sys) / 1e6);

  stats_init();

  rtc_init();
  datetime_t dt0 = {1900, 1, 1, 0, 0, 0, 0};
  rtc_set_datetime(&dt0);

  input_timer.start();

  multicore_reset_core1();
  multicore_launch_core1(core1_main);

  app.run();

  return 0;
}
