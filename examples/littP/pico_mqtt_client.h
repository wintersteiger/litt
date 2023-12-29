#ifndef _PICO_MQTT_CLIENT_H_
#define _PICO_MQTT_CLIENT_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lwipopts.h"

#include <pico/cyw43_arch.h>
#include <pico/cyw43_arch/arch_threadsafe_background.h>

#include <lwip/apps/mqtt.h>

#include "pico_litt_ds.h"

extern void vllog(const char *fmt, va_list args);
extern void llog(const char *fmt, ...);

template <size_t MAX_MSG_SIZE, size_t MAX_CONCURRENT> class MQTTClient {
public:
  struct Configuration {
    const char *host = nullptr;
    const char *client_id = nullptr;
    const char *client_user = nullptr;
    const char *client_pass = nullptr;

    const char *topics[32] = {};
    size_t num_topics = 0;
  };

protected:
  struct MQTTPublishRequest {
    const char *topic;
    char payload[32];
    size_t length;
  };

  Configuration config;
  mqtt_client_t *client = nullptr;
  PicoSemaphore tx_sem;
  PicoQueue<MQTTPublishRequest> tx_queue;
  uint8_t data_buffer[MAX_MSG_SIZE * MAX_CONCURRENT];
  size_t data_arrived = 0;

public:
  static constexpr const size_t max_concurrent_requests = MAX_CONCURRENT;
  size_t num_mqtt_reinit = 0;

  MQTTClient() : tx_queue(max_concurrent_requests) {}

  virtual ~MQTTClient() = default;

  virtual void on_message(const char *topic, const uint8_t *payload,
                          size_t payload_len) = 0;

  void publish_queue_add(const MQTTPublishRequest &req) {
    if (!tx_queue.try_add(req))
      llog("dropping MQTT publish request");
  }

  void publish_all_queued() {
    static volatile bool busy = false;

    if (!client || busy)
      return;

    busy = true;

    MQTTPublishRequest req;
    while (tx_queue.try_remove(req)) {
      if (!client)
        reinitialize();

      for (bool retry = true; retry;) {
        tx_sem.acquire_blocking();
        {
          int err = pmqtt_publish(
              client, req.topic, req.payload, req.length, 0, 0,
              [](void *arg, err_t err) {
                if (err != ERR_OK && err != ERR_TIMEOUT)
                  llog("X mqtt_publish failed (callback): err=%d", err);
                if (err != ERR_TIMEOUT)
                  static_cast<PicoSemaphore *>(arg)->release();
              },
              &tx_sem);

          if (err != ERR_OK) {
            llog("X mqtt_publish failed (call): err=%d", err);
            reinitialize();
            retry = true;
          } else if (!tx_sem.acquire_timeout(MQTT_REQ_TIMEOUT * 1e6)) {
            llog("? mqtt_publish retry after timeout");
            retry = true;
          } else
            retry = false;
        }
        tx_sem.release();
      }
    }

    busy = false;
  }

  static void on_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTTClient *c = static_cast<MQTTClient *>(arg);
    c->on_publish(topic, tot_len);
  }

  u32_t next_msg_size = 0;
  char next_msg_topic[32] = "";

  void on_publish(const char *topic, u32_t tot_len) {
    next_msg_size = tot_len;
    strncpy(next_msg_topic, topic, sizeof(next_msg_topic));
  }

  static void on_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTTClient *c = static_cast<MQTTClient *>(arg);
    c->on_data(data, len, flags);
  }

  void on_data(const u8_t *data, u16_t len, u8_t flags) {
    for (uint16_t i = 0; i < len && data_arrived < sizeof(data_buffer); i++)
      data_buffer[data_arrived++] = data[i];

    if (flags & MQTT_DATA_FLAG_LAST) {
      on_message(next_msg_topic, data_buffer, data_arrived);

      data_arrived = 0;
      next_msg_size = 0;
      next_msg_topic[0] = '\0';
    }
  }

  static void on_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status) {
    MQTTClient *c = static_cast<MQTTClient *>(arg);
    c->on_connection(client, status);
  }

  void on_connection(mqtt_client_t *client, mqtt_connection_status_t status) {
    if (status != MQTT_CONNECT_ACCEPTED)
      llog("> MQTT connection status: %d", status);
  }

  void reinitialize() {
    llog("> MQTT reconnecting");
    num_mqtt_reinit++;
    if (client) {
      mqtt_disconnect(client);
      mqtt_client_free(client);
      client = NULL;
    }
    initialize(config);
  }

  void initialize(const Configuration &config) {
    this->config = config;
    data_arrived = 0;

    ip_addr_t ip;
    if (ipaddr_aton(config.host, &ip) != 1) {
      llog("X ipaddr_aton failed");
      return;
    }

    u16_t port = 1883;

    client = pmqtt_client_new();

    if (!client) {
      llog("X failed to allocate new MQTT client");
      return;
    }

    struct mqtt_connect_client_info_t client_info = {0};
    client_info.client_id = config.client_id;
    client_info.client_user = config.client_user;
    client_info.client_pass = config.client_pass;
    client_info.will_topic = NULL;

    err_t r = pmqtt_client_connect(client, &ip, port, on_connection_cb, this,
                                   &client_info);
    if (r != ERR_OK) {
      llog("X mqtt_client_connect failed: %d", r);
      return;
    }

    pmqtt_set_inpub_callback(client, on_publish_cb, on_data_cb, this);

    for (size_t i = 0; i < config.num_topics; i++)
      subscribe(config.topics[i]);
  }

  err_t subscribe(const char *topic) {
    err_t r = pmqtt_subscribe(
        client, topic, 0,
        [](void *arg, err_t err) {
          if (err != ERR_OK)
            llog("X mqtt_subscribe timed out: %d", err);
        },
        NULL);

    if (r != ERR_OK)
      llog("X mqtt_subscribe to '%s' failed: %d", topic, r);

    return r;
  }

protected:
  inline mqtt_client_t *pmqtt_client_new(void) {
    cyw43_arch_lwip_begin();
    mqtt_client_t *r = mqtt_client_new();
    cyw43_arch_lwip_end();
    return r;
  }

  err_t
  pmqtt_client_connect(mqtt_client_t *client, const ip_addr_t *ipaddr,
                       u16_t port, mqtt_connection_cb_t cb, void *arg,
                       const struct mqtt_connect_client_info_t *client_info) {
    cyw43_arch_lwip_begin();
    err_t r = mqtt_client_connect(client, ipaddr, port, cb, arg, client_info);
    cyw43_arch_lwip_end();
    return r;
  }

  err_t pmqtt_subscribe(mqtt_client_t *client, const char *topic, u8_t qos,
                        mqtt_request_cb_t cb, void *arg) {
    cyw43_arch_lwip_begin();
    err_t r = mqtt_subscribe(client, topic, qos, cb, arg);
    cyw43_arch_lwip_end();
    return r;
  }

  err_t pmqtt_publish(mqtt_client_t *client, const char *topic,
                      const void *payload, u16_t payload_length, u8_t qos,
                      u8_t retain, mqtt_request_cb_t cb, void *arg) {
    cyw43_arch_lwip_begin();
    err_t r = mqtt_publish(client, topic, payload, payload_length, qos, retain,
                           cb, arg);
    cyw43_arch_lwip_end();
    return r;
  }

  void pmqtt_set_inpub_callback(mqtt_client_t *client,
                                mqtt_incoming_publish_cb_t pub_cb,
                                mqtt_incoming_data_cb_t data_cb, void *arg) {
    cyw43_arch_lwip_begin();
    mqtt_set_inpub_callback(client, pub_cb, data_cb, arg);
    cyw43_arch_lwip_end();
  }
};

#endif // _PICO_MQTT_CLIENT_H_