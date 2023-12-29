// CM Wintersteiger, 2022

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);

#include <math.h>
#include <stdio.h>
#include <time.h>

#include <hal/nrf_power.h>

extern "C" {
#include <addons/zcl/zb_zcl_basic_addons.h>
#include <zb_errors.h>
#include <zb_nrf_platform.h>
#include <zb_osif.h>
#include <zcl/zb_zcl_basic.h>
#include <zcl/zb_zcl_common.h>
#include <zcl/zb_zcl_reporting.h>
#include <zcl/zb_zcl_thermostat.h>
#include <zcl/zb_zcl_time.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>

#include "zb_zcl_opentherm.h"
}

#include <litt/opentherm/application.h>
#include <litt/opentherm/boiler_interface.h>
#include <litt/opentherm/transport.h>
#include <litt/scheduler.h>
#include <litt/serialization.h>
#include <litt/thermostat.h>

#include "zephyr_litt_ds.h"
#include "zephyr_litt_io.h"

using namespace litt;
using namespace litt::OpenTherm;

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

static const struct gpio_dt_spec leds[] = {GPIO_DT_SPEC_GET(LED0_NODE, gpios), GPIO_DT_SPEC_GET(LED1_NODE, gpios),
                                           GPIO_DT_SPEC_GET(LED2_NODE, gpios)};

#define OT_IN_NODE DT_ALIAS(otin)
#define OT_OUT_NODE DT_ALIAS(otout)

#define MAX_DEMANDERS 64
#define MAX_SCHEDULER_TRANSITIONS (24 * 7)

#define DEVICE_ID 0x0052
#define DEVICE_VERSION 1

#define IN_CLUSTER_NUM 4
#define OUT_CLUSTER_NUM 3
#define NUM_IN_OUT_CLUSTERS (IN_CLUSTER_NUM + OUT_CLUSTER_NUM)

#define REPORT_ATTR_COUNT                                                                                              \
  (ZB_ZCL_THERMOSTAT_REPORT_ATTR_COUNT + 11 + ZB_ZCL_OPENTHERM_REPORT_ATTR_COUNT + ZB_ZCL_TIME_REPORT_ATTR_COUNT)

#define DECLARE_CLUSTER_LIST(cluster_list_name, basic_attr_list, thermostat_attr_list, opentherm_attr_list,            \
                             time_attr_list)                                                                           \
  zb_zcl_cluster_desc_t cluster_list_name[] = {                                                                        \
      ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_BASIC, ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),                  \
                          (basic_attr_list), ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),                   \
      ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_THERMOSTAT, ZB_ZCL_ARRAY_SIZE(thermostat_attr_list, zb_zcl_attr_t),        \
                          (thermostat_attr_list), ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),              \
      ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_OPENTHERM, ZB_ZCL_ARRAY_SIZE(opentherm_attr_list, zb_zcl_attr_t),          \
                          (opentherm_attr_list), ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),               \
      ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_TIME, ZB_ZCL_ARRAY_SIZE((time_attr_list), zb_zcl_attr_t),                  \
                          (time_attr_list), ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),                    \
      ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_BASIC, ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),                  \
                          (basic_attr_list), ZB_ZCL_CLUSTER_CLIENT_ROLE, ZB_ZCL_MANUF_CODE_INVALID),                   \
      ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_TIME, ZB_ZCL_ARRAY_SIZE((time_attr_list), zb_zcl_attr_t),                  \
                          (time_attr_list), ZB_ZCL_CLUSTER_CLIENT_ROLE, ZB_ZCL_MANUF_CODE_INVALID),                    \
      ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_THERMOSTAT, 0, NULL, ZB_ZCL_CLUSTER_CLIENT_ROLE,                           \
                          ZB_ZCL_MANUF_CODE_INVALID)}

#define DECLARE_SIMPLE_DESC(ep_name, ep_id, in_clust_num, out_clust_num)                                               \
  ZB_DECLARE_SIMPLE_DESC(in_clust_num, out_clust_num);                                                                 \
  ZB_AF_SIMPLE_DESC_TYPE(in_clust_num, out_clust_num)                                                                  \
  simple_desc_##ep_name = {ep_id,                                                                                      \
                           ZB_AF_HA_PROFILE_ID,                                                                        \
                           DEVICE_ID,                                                                                  \
                           DEVICE_VERSION,                                                                             \
                           0,                                                                                          \
                           in_clust_num,                                                                               \
                           out_clust_num,                                                                              \
                           {ZB_ZCL_CLUSTER_ID_BASIC, ZB_ZCL_CLUSTER_ID_THERMOSTAT, ZB_ZCL_CLUSTER_ID_OPENTHERM,        \
                            ZB_ZCL_CLUSTER_ID_TIME, ZB_ZCL_CLUSTER_ID_BASIC, ZB_ZCL_CLUSTER_ID_TIME,                   \
                            ZB_ZCL_CLUSTER_ID_THERMOSTAT}}

#define DECLARE_SIMPLE_DESC_EMPTY(ep_name, ep_id)                                                                      \
  ZB_DECLARE_SIMPLE_DESC(0, 0);                                                                                        \
  ZB_AF_SIMPLE_DESC_TYPE(0, 0)                                                                                         \
  simple_desc_##ep_name = {ep_id, ZB_AF_HA_PROFILE_ID, DEVICE_ID, DEVICE_VERSION, 0, 0, 0, {}}

#define DECLARE_ENDPOINT(ep_name, ep_id, cluster_list)                                                                 \
  DECLARE_SIMPLE_DESC(ep_name, ep_id, IN_CLUSTER_NUM, OUT_CLUSTER_NUM);                                                \
  ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info##ep_name, REPORT_ATTR_COUNT);                                      \
  ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id, ZB_AF_HA_PROFILE_ID, 0, NULL,                                            \
                              ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t), cluster_list,                    \
                              (zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name, REPORT_ATTR_COUNT,                    \
                              reporting_info##ep_name, 0, NULL)

#define ZB_ZCL_ATTR_THERMOSTAT_EXTRA_STATUS_ID 0xFF00
#define ZB_ZCL_ATTR_THERMOSTAT_SPIKE_PROTECTION_COUNT_ID 0xFF01
#define ZB_ZCL_ATTR_THERMOSTAT_TINY_CYCLES_ID 0xFF02
#define ZB_ZCL_ATTR_THERMOSTAT_SHORT_CYCLES_ID 0xFF03
#define ZB_ZCL_ATTR_THERMOSTAT_NORMAL_CYCLES_ID 0xFF04
#define ZB_ZCL_ATTR_THERMOSTAT_TINY_CYCLE_PROTECTION_COUNT_ID 0xFF05

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_THERMOSTAT_EXTRA_STATUS_ID(data_ptr)                                        \
  {                                                                                                                    \
    ZB_ZCL_ATTR_THERMOSTAT_EXTRA_STATUS_ID, ZB_ZCL_ATTR_TYPE_U32,                                                      \
        ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING, (void *)data_ptr                                  \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_THERMOSTAT_SPIKE_PROTECTION_COUNT_ID(data_ptr)                              \
  {                                                                                                                    \
    ZB_ZCL_ATTR_THERMOSTAT_SPIKE_PROTECTION_COUNT_ID, ZB_ZCL_ATTR_TYPE_U32,                                            \
        ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING, (void *)data_ptr                                  \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_THERMOSTAT_TINY_CYCLES_ID(data_ptr)                                         \
  {                                                                                                                    \
    ZB_ZCL_ATTR_THERMOSTAT_TINY_CYCLES_ID, ZB_ZCL_ATTR_TYPE_U32,                                                       \
        ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING, (void *)data_ptr                                  \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_THERMOSTAT_SHORT_CYCLES_ID(data_ptr)                                        \
  {                                                                                                                    \
    ZB_ZCL_ATTR_THERMOSTAT_SHORT_CYCLES_ID, ZB_ZCL_ATTR_TYPE_U32,                                                      \
        ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING, (void *)data_ptr                                  \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_THERMOSTAT_NORMAL_CYCLES_ID(data_ptr)                                       \
  {                                                                                                                    \
    ZB_ZCL_ATTR_THERMOSTAT_NORMAL_CYCLES_ID, ZB_ZCL_ATTR_TYPE_U32,                                                     \
        ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING, (void *)data_ptr                                  \
  }

#define ZB_ZCL_DECLARE_THERMOSTAT_ATTRIB_LIST_EXT_PLUS(                                                                \
    attr_list, local_temperature, outdoor_temperature, abs_min_heat_setpoint_limit, abs_max_heat_setpoint_limit,       \
    abs_min_cool_setpoint_limit, abs_max_cool_setpoint_limit, PI_cooling_demand, PI_heating_demand,                    \
    HVAC_system_type_configuration, local_temperature_calibration, occupied_cooling_setpoint,                          \
    occupied_heating_setpoint, unoccupied_cooling_setpoint, unoccupied_heating_setpoint, min_heat_setpoint_limit,      \
    max_heat_setpoint_limit, min_cool_setpoint_limit, max_cool_setpoint_limit, min_setpoint_dead_band, remote_sensing, \
    control_seq_of_operation, system_mode, start_of_week, number_of_weekly_transitions, number_of_daily_transitions,   \
    programming_operation_mode, extra_status, spike_protection_count, tiny_cycles, short_cycles, normal_cycles)        \
  ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(attr_list, ZB_ZCL_THERMOSTAT)                                      \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, (local_temperature))                               \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID, (outdoor_temperature))                           \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_LIMIT_ID, (abs_min_heat_setpoint_limit))           \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_LIMIT_ID, (abs_max_heat_setpoint_limit))           \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_COOL_SETPOINT_LIMIT_ID, (abs_min_cool_setpoint_limit))           \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_COOL_SETPOINT_LIMIT_ID, (abs_max_cool_setpoint_limit))           \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_PI_COOLING_DEMAND_ID, (PI_cooling_demand))                               \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID, (PI_heating_demand))                               \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_HVAC_SYSTEM_TYPE_CONFIGURATION_ID, (HVAC_system_type_configuration))     \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_CALIBRATION_ID, (local_temperature_calibration))       \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, (occupied_cooling_setpoint))               \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, (occupied_heating_setpoint))               \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_COOLING_SETPOINT_ID, (unoccupied_cooling_setpoint))           \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID, (unoccupied_heating_setpoint))           \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID, (min_heat_setpoint_limit))                   \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID, (max_heat_setpoint_limit))                   \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID, (min_cool_setpoint_limit))                   \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID, (max_cool_setpoint_limit))                   \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_MIN_SETPOINT_DEAD_BAND_ID, (min_setpoint_dead_band))                     \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID, (remote_sensing))                                     \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_ID, (control_seq_of_operation))            \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, (system_mode))                                           \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_START_OF_WEEK_ID, (start_of_week))                                       \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_NUMBER_OF_WEEKLY_TRANSITIONS_ID, (number_of_weekly_transitions))         \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_NUMBER_OF_DAILY_TRANSITIONS_ID, (number_of_daily_transitions))           \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_PROGRAMMING_OPERATION_MODE_ID, (programming_operation_mode))  \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_EXTRA_STATUS_ID, (extra_status))                                         \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_SPIKE_PROTECTION_COUNT_ID, (spike_protection_count))                     \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_TINY_CYCLES_ID, (tiny_cycles))                                           \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_SHORT_CYCLES_ID, (short_cycles))                                         \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_NORMAL_CYCLES_ID, (normal_cycles))                                       \
  ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

#define ENDPOINT_ID 1
#define BASIC_APP_VERSION 02
#define BASIC_STACK_VERSION 10
#define BASIC_HW_VERSION 01
#define BASIC_MANUF_NAME "ACME Corp."
#define BASIC_MANUF_ID 0x1234
#define BASIC_MODEL_ID "littZ"
#define BASIC_DATE_CODE __DATE__
#define BASIC_POWER_SOURCE ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE
#define BASIC_LOCATION_DESC "Boiler"
#define BASIC_PH_ENV ZB_ZCL_BASIC_ENV_UNSPECIFIED
#define BASIC_SW_BUILD_ID "0.0.2"

void reboot() {
  LOG_ERR("rebooting the device");
  LOG_PANIC();
  zb_reset(0);
}

extern "C" {
void mpsl_assert_handle(const char *file, uint32_t line) {
  LOG_ERR("MPSL assert at %s:%d\n", file, line);
  reboot();
}

void z_SysFatalErrorHandler(unsigned int reason, const z_arch_esf_t *esf) {
  LOG_ERR("Fatal system error (z, reason: %u)\n", reason);
  reboot();
}

void k_sys_fatal_error_handler(unsigned int reason, const z_arch_esf_t *esf) {
  LOG_ERR("Fatal system error (k, reason: %u)\n", reason);
  reboot();
}
}

static char ieee_addr_string[32] = "";

static const char *addr_to_string(const zb_ieee_addr_t &addr) {
  if (ieee_addr_to_str(ieee_addr_string, sizeof(ieee_addr_string), addr) <= 0)
    sprintf(ieee_addr_string, "<invalid>");
  return ieee_addr_string;
}

static void zb_zcl_time_sync_time_server_found_cb(zb_ret_t status, zb_uint32_t auth_level, zb_uint16_t short_addr,
                                                  zb_uint8_t ep, // Not the endpoint?
                                                  zb_uint32_t nw_time);

static bool reading_thermostats_in_progress = false;
static zb_ieee_addr_t s_addr;
static zb_uint8_t s_ep;
static zb_uint16_t s_cluster_id;
static zb_uint16_t s_attribute_id;

static void read_attribute(const zb_ieee_addr_t &addr, zb_uint8_t ep, zb_uint16_t cluster_id,
                           zb_uint16_t attribute_id) {
  zb_ret_t r;

  ZB_64BIT_ADDR_COPY(s_addr, addr);
  s_ep = ep;
  s_cluster_id = cluster_id;
  s_attribute_id = attribute_id;

  LOG_DBG("read %04x/%04x from %04x:%d (%s)", cluster_id, attribute_id, zb_address_short_by_ieee(s_addr), s_ep,
          addr_to_string(s_addr));

  // This?
  // zb_osif_disable_all_inter();
  // bufid = zb_buf_get_out();
  // zb_osif_enable_all_inter();

  if ((r = zb_buf_get_out_delayed([](zb_bufid_t bufid) {
         zb_uint8_t *cmd_ptr;
         cmd_ptr = (zb_uint8_t *)ZB_ZCL_START_PACKET(bufid);
         ZB_ZCL_CONSTRUCT_GENERAL_COMMAND_REQ_FRAME_CONTROL(cmd_ptr, ZB_ZCL_DISABLE_DEFAULT_RESPONSE);
         ZB_ZCL_CONSTRUCT_COMMAND_HEADER(cmd_ptr, ZB_ZCL_GET_SEQ_NUM(), ZB_ZCL_CMD_READ_ATTRIB);
         cmd_ptr = (zb_uint8_t *)zb_put_next_htole16(cmd_ptr, s_attribute_id);
         ZB_ZCL_GENERAL_SEND_READ_ATTR_REQ(bufid, cmd_ptr, s_addr, ZB_APS_ADDR_MODE_64_ENDP_PRESENT, s_ep, ENDPOINT_ID,
                                           ZB_AF_HA_PROFILE_ID, s_cluster_id, NULL);
       })) != RET_OK)
    LOG_WRN("zb_buf_get_out_delayed failed: %d", r);
}

static void read_demand(const zb_ieee_addr_t &addr, zb_uint8_t ep) {
  read_attribute(addr, ep, ZB_ZCL_CLUSTER_ID_THERMOSTAT, ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID);
}

static void read_demand_if_bound(zb_bufid_t bufid, zb_uint16_t nwk_addr, zb_ieee_addr_t ieee_addr,
                                 zb_uint8_t start_index) {
  if (ZB_NWK_IS_ADDRESS_BROADCAST(nwk_addr)) {
    LOG_DBG("mgmt bind request to broadcast address %04x dropped", nwk_addr);
    zb_buf_free(bufid);
    return;
  }

  zb_zdo_mgmt_bind_param_t *req = ZB_BUF_GET_PARAM(bufid, zb_zdo_mgmt_bind_param_t);
  req->dst_addr = nwk_addr;
  req->start_index = start_index;

  auto r = zb_zdo_mgmt_bind_req(bufid, [](zb_bufid_t bufid) {
    zb_uint16_t my_short = zb_get_short_address();
    zb_ieee_addr_t my_long;
    zb_get_long_address(my_long);
    zb_ieee_addr_t their_addr;

    const zb_zdo_mgmt_bind_resp_t *rsp = (zb_zdo_mgmt_bind_resp_t *)zb_buf_begin(bufid);
    if (rsp->status == ZB_ZDP_STATUS_TIMEOUT) {
      LOG_DBG("zb_zdo_mgmt_bind_req timed out (tsn: %d)", rsp->tsn);
      zb_buf_free(bufid);
    }
    if (rsp->status != ZB_ZDP_STATUS_SUCCESS) {
      LOG_DBG("zb_zdo_mgmt_bind_req failed (tsn: %d)", rsp->tsn);
      zb_buf_free(bufid);
    } else {
      const zb_zdo_binding_table_record_t *records = (const zb_zdo_binding_table_record_t *)(rsp + 1);
      uint32_t next_start_index = ((uint32_t)rsp->start_index + rsp->binding_table_list_count);

      for (uint32_t i = rsp->start_index; i < next_start_index; i++) {
        const auto &record = *records;

        if (ZB_IEEE_ADDR_IS_VALID(record.src_address)) {
          ZB_64BIT_ADDR_COPY(their_addr, record.src_address);

          if ((record.dst_addr_mode == ZB_APS_ADDR_MODE_64_ENDP_PRESENT &&
               ZB_64BIT_ADDR_CMP(record.dst_address.addr_long, my_long)) ||
              (record.dst_addr_mode == ZB_APS_ADDR_MODE_16_ENDP_PRESENT && record.dst_address.addr_short == my_short)) {
            read_demand(record.src_address, record.src_endp);
          }
        } else
          LOG_INF("invalid ieee address: %s", addr_to_string(their_addr));

        if (records->dst_addr_mode == ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT) {
          static const size_t GROUP_BIND_TABLE_RECORD_SIZE =
              ZB_OFFSETOF(zb_zdo_binding_table_record_t, dst_address) + sizeof(zb_uint16_t);
          records = (zb_zdo_binding_table_record_t *)((zb_uint8_t *)records + GROUP_BIND_TABLE_RECORD_SIZE);
        } else
          records++;
      }

      if (next_start_index < rsp->binding_table_entries) {
        zb_buf_reuse(bufid);
        read_demand_if_bound(bufid, zb_address_short_by_ieee(their_addr), their_addr, next_start_index);
      } else
        zb_buf_free(bufid);
    }
  });

  if (r == ZB_ZDO_INVALID_TSN) {
    LOG_WRN("zb_zdo_mgmt_bind_req cannot be performed now");
    zb_buf_free(bufid);
  }
}

static void read_demand_from_bound_thermostats() {
  if (reading_thermostats_in_progress)
    return;

  reading_thermostats_in_progress = true;

  zb_ret_t r;
  if ((r = zb_buf_get_out_delayed([](zb_bufid_t bufid) {
         zb_zdo_match_desc_param_t *req = (zb_zdo_match_desc_param_t *)zb_buf_initial_alloc(
             bufid, sizeof(zb_zdo_match_desc_param_t) + sizeof(zb_uint16_t));
         req->nwk_addr = ZB_NWK_BROADCAST_ALL_DEVICES;
         req->addr_of_interest = req->nwk_addr;
         req->profile_id = ZB_AF_HA_PROFILE_ID;
         req->num_in_clusters = 1;
         req->num_out_clusters = 1;
         req->cluster_list[0] = ZB_ZCL_CLUSTER_ID_THERMOSTAT;
         req->cluster_list[1] = ZB_ZCL_CLUSTER_ID_THERMOSTAT;

         auto r = zb_zdo_match_desc_req(bufid, [](zb_bufid_t bufid) {
           auto *mdr = (zb_zdo_match_desc_resp_t *)zb_buf_begin(bufid);
           auto *ind = ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t);

           if (mdr->status == ZB_ZDP_STATUS_SUCCESS) {
             if (mdr->match_len == 0)
               zb_buf_free(bufid);
             else {
               zb_buf_reuse(bufid);

               zb_zdo_ieee_addr_req_param_t *req = ZB_BUF_GET_PARAM(bufid, zb_zdo_ieee_addr_req_param_t);
               req->dst_addr = ind->src_addr;
               req->nwk_addr = ind->src_addr;
               req->start_index = 0;
               req->request_type = 0;

               auto r = zb_zdo_ieee_addr_req(bufid, [](zb_bufid_t bufid) {
                 auto rsp = (zb_zdo_ieee_addr_resp_t *)zb_buf_begin(bufid);
                 if (rsp->status == ZB_ZDP_STATUS_SUCCESS) {
                   zb_ieee_addr_t ieee_addr;
                   ZB_64BIT_ADDR_COPY(rsp->ieee_addr_remote_dev, ieee_addr);
                   zb_uint16_t nwk_addr = rsp->nwk_addr_remote_dev;
                   zb_buf_reuse(bufid);
                   read_demand_if_bound(bufid, nwk_addr, ieee_addr, 0);
                 } else {
                   LOG_DBG("zb_zdo_ieee_addr_req failed (tsn: %u)", rsp->tsn);
                   zb_buf_free(bufid);
                 }
               });

               if (r == ZB_ZDO_INVALID_TSN) {
                 LOG_DBG("zb_zdo_ieee_addr_req cannot be performed now");
                 zb_buf_free(bufid);
               }
             }
           } else {
             if (mdr->status != ZB_ZDP_STATUS_TIMEOUT)
               LOG_DBG("zb_zdo_match_desc_req failed (tsn: %u, status: %d)", mdr->tsn, mdr->status);
             zb_buf_free(bufid);
           }

           reading_thermostats_in_progress = false;
         });

         if (r == ZB_ZDO_INVALID_TSN) {
           zb_buf_free(bufid);
           LOG_ERR("zb_zdo_match_desc_req cannot be performed now");
           reading_thermostats_in_progress = false;
         }
       })) != RET_OK)
    LOG_INF("zb_buf_get_out_delayed failed: %d", r);
}

static bool time_sync_in_progress = false;

static void start_time_sync() {
  if (time_sync_in_progress)
    return;

  zb_zcl_time_server_synchronize(ENDPOINT_ID, zb_zcl_time_sync_time_server_found_cb);
  time_sync_in_progress = true;
}

zb_bool_t rtc_cb(zb_uint32_t time) {
  LOG_WRN("rtc_cb");
  return ZB_TRUE;
}

static bool zb_stack_initialised = false;
static bool zb_joining_signal_received = false;

void zboss_signal_handler(zb_bufid_t bufid) {
  zb_zdo_app_signal_hdr_t *sig_hdr = NULL;
  zb_zdo_app_signal_type_t sig_type = zb_get_app_signal(bufid, &sig_hdr);
  zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);

  if (sig_type != ZB_COMMON_SIGNAL_CAN_SLEEP)
    gpio_pin_set_dt(&leds[2], 1);

  if (sig_type != ZB_NLME_STATUS_INDICATION)
    ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

  // if (sig_type != ZB_NLME_STATUS_INDICATION && sig_type != ZB_COMMON_SIGNAL_CAN_SLEEP)
  //   LOG_INF("signal %d", sig_type);

  // Workaround for KRKNWK-12017
  // See https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/known_issues.html?v=v2-3-0

  switch (sig_type) {
  case ZB_ZDO_SIGNAL_SKIP_STARTUP:
    zb_stack_initialised = true;
    break;
  case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
  case ZB_BDB_SIGNAL_DEVICE_REBOOT:
  case ZB_BDB_SIGNAL_STEERING:
    zb_joining_signal_received = true;
    break;
  case ZB_ZDO_SIGNAL_LEAVE:
    if (status == RET_OK) {
      zb_zdo_signal_leave_params_t *leave_params = ZB_ZDO_SIGNAL_GET_PARAMS(sig_hdr, zb_zdo_signal_leave_params_t);

      /* Set joining_signal_received to false so broken rejoin procedure can be detected correctly. */
      if (leave_params->leave_type == ZB_NWK_LEAVE_TYPE_REJOIN)
        zb_joining_signal_received = false;
      break;
    }
  case ZB_NLME_STATUS_INDICATION: {
    zb_zdo_signal_nlme_status_indication_params_t *nlme_status_ind =
        ZB_ZDO_SIGNAL_GET_PARAMS(sig_hdr, zb_zdo_signal_nlme_status_indication_params_t);
    if (nlme_status_ind->nlme_status.status == ZB_NWK_COMMAND_STATUS_PARENT_LINK_FAILURE) {
      if (zb_stack_initialised && !zb_joining_signal_received) {
        LOG_ERR("Broken zigbee rejoin procedure detected, rebooting the device.");
        reboot();
      }
    }
    break;
  }
  }

  switch (sig_type) {
  case ZB_ZDO_SIGNAL_SKIP_STARTUP:
  case ZB_COMMON_SIGNAL_CAN_SLEEP:
  case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
  case ZB_NLME_STATUS_INDICATION:
  case ZB_BDB_SIGNAL_TC_REJOIN_DONE:
    break;
  case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
  case ZB_BDB_SIGNAL_DEVICE_REBOOT:
  case ZB_BDB_SIGNAL_STEERING: {
    start_time_sync();
    read_demand_from_bound_thermostats();
    break;
  }
  case ZB_ZDO_SIGNAL_LEAVE: {
    zb_zdo_signal_leave_params_t *leave_params = ZB_ZDO_SIGNAL_GET_PARAMS(sig_hdr, zb_zdo_signal_leave_params_t);

    if (leave_params->leave_type == ZB_NWK_LEAVE_TYPE_REJOIN) {
      // Usually, zboss automatically calls start_network_rejoin(). The automatic rejoin attempts stop after
      // `ZB_DEV_REJOIN_TIMEOUT_MS` and the user is expected to push a button to trigger further attempts.
      // We want to retry forever, so we call `user_input_indicate` right here.
      LOG_DBG("automatically pushing rejoin button");
      user_input_indicate();
    } else
      LOG_WRN("left network without request to rejoin");
    break;
  }
  default:
    LOG_INF("Unhandled zigbee signal: %d", sig_type);
  }

  if (bufid)
    zb_buf_free(bufid);

  if (sig_type != ZB_COMMON_SIGNAL_CAN_SLEEP)
    gpio_pin_set_dt(&leds[2], 0);
}

class AttributeBase {
public:
  AttributeBase(uint8_t role, uint16_t cluster_id, uint16_t attribute_id)
      : role(role), cluster_id(cluster_id), attribute_id(attribute_id) {}

  virtual ~AttributeBase() = default;

  virtual operator zb_uint8_t *() = 0;

  uint8_t role;
  uint16_t cluster_id;
  uint16_t attribute_id;

protected:
  virtual zb_uint8_t *next_value_ptr() = 0;
  virtual void commit() = 0;

  struct UpdateRequest {
    AttributeBase *attribute;
  };

  static ZephyrQueue<AttributeBase::UpdateRequest> update_queue;

  static void update(zb_bufid_t bufid) {
    UpdateRequest *req = (UpdateRequest *)zb_buf_begin(bufid);

    if (req) {
      AttributeBase *attr = req->attribute;
      zb_zcl_status_t r = zb_zcl_set_attr_val(ENDPOINT_ID, attr->cluster_id, attr->role, attr->attribute_id,
                                              attr->next_value_ptr(), ZB_FALSE);
      if (r != ZB_ZCL_STATUS_SUCCESS)
        LOG_ERR("error while updating %04x/%04x: status=%02x", attr->cluster_id, attr->attribute_id, r);
      else
        attr->commit();
    }

    zb_buf_free(bufid);
  }

  static void schedule_update(AttributeBase *attr) {
    AttributeBase::UpdateRequest tmp = {.attribute = attr};

    if (zb_buf_memory_low() || zb_buf_is_oom_state())
      LOG_ERR("error while scheduling attribute update: zboss low on memory");
    else if (!update_queue.try_add(tmp)) {
      LOG_ERR("error while scheduling attribute update: queue full");
    } else {
      zb_ret_t r;
      if ((r = zb_buf_get_out_delayed([](zb_bufid_t bufid) {
             zb_ret_t r;
             AttributeBase::UpdateRequest *req = (AttributeBase::UpdateRequest *)zb_buf_initial_alloc(
                 bufid, sizeof(AttributeBase::UpdateRequest) + sizeof(zb_uint16_t));
             if (!req) {
               LOG_ERR("error while allocating buffer for attribute update");
               zb_buf_free(bufid);
               return;
             }
             AttributeBase::UpdateRequest ureq;
             if (!update_queue.try_remove(ureq)) {
               LOG_ERR("error while scheduling attribute update: queue empty");
               zb_buf_free(bufid);
             } else {
               req->attribute = ureq.attribute;
               if ((r = ZB_SCHEDULE_APP_CALLBACK(AttributeBase::update, bufid)) != 0) {
                 LOG_ERR("error while scheduling update for %04x/%04x: status=%02x", req->attribute->cluster_id,
                         req->attribute->attribute_id, r);
                 zb_buf_free(bufid);
               }
             }
           })) != RET_OK) {
        LOG_ERR("error while requesting buffer for update of %04x/%04x: status=%02x", attr->cluster_id,
                attr->attribute_id, r);
        LOG_ERR("zboss: thread: created: %d suspended: %d", zigbee_debug_zboss_thread_is_created(),
                zigbee_is_zboss_thread_suspended());
        if (zigbee_is_zboss_thread_suspended())
          zigbee_debug_resume_zboss_thread();
      }
    }
  }
};

ZephyrQueue<AttributeBase::UpdateRequest> AttributeBase::update_queue = ZephyrQueue<AttributeBase::UpdateRequest>(64);

template <typename T> class Attribute : public AttributeBase {
public:
  Attribute(uint8_t role, uint16_t cluster_id, uint16_t attribute_id) : AttributeBase(role, cluster_id, attribute_id) {}

  Attribute(uint8_t role, uint16_t cluster_id, uint16_t attribute_id, const T &initial_value)
      : AttributeBase(role, cluster_id, attribute_id), data(initial_value), next_data(initial_value) {}

  virtual ~Attribute() = default;

  operator T &() { return dirty ? next_data : data; }

  operator const T &() const { return dirty ? next_data : data; }

  T &operator=(const T &value) {
    next_data = value;
    dirty = true;
    schedule_update(this);
    return *this;
  }

  virtual operator zb_uint8_t *() override { return (zb_uint8_t *)(dirty ? &next_data : &data); }

  T *dptr() { return dirty ? &next_data : &data; }

  Attribute<T> &operator++() {
    *this = (dirty ? next_data : data) + 1;
    return *this;
  }

  Attribute<T> &operator--() {
    *this = (dirty ? next_data : data) - 1;
    return *this;
  }

  void operator++(int) { operator++(); }

  void operator--(int) { operator--(); }

protected:
  // Note: The dirty flag allows us to use the "future" value of the attribute (in `next_data`), while zboss still sees
  // the old value (in `data`) until any scheduled updates are done and committed. This avoids spurious invalid-value
  // errors, e.g. when updating heating demand to a non-zero value while the system mode is set to 0 (off).
  bool dirty = false;

  T data = {0};
  T next_data = {0};

  virtual zb_uint8_t *next_value_ptr() override { return (zb_uint8_t *)&next_data; }

  virtual void commit() override {
    memcpy(&data, &next_data, sizeof(T));
    dirty = false;
  };
};

template <typename T> class ArrayAttribute : public Attribute<T> {
public:
  ArrayAttribute(uint8_t role, uint16_t cluster_id, uint16_t attribute_id, const T &initial_value)
      : Attribute<T>(role, cluster_id, attribute_id) {
    size_t n = sizeof(initial_value) / sizeof(initial_value[0]);
    for (size_t i = 0; i < n; i++)
      Attribute<T>::data[i] = Attribute<T>::next_data[i] = initial_value[i];
  }

  virtual ~ArrayAttribute() = default;

  T &operator=(const T &value) {
    memcpy(Attribute<T>::next_data, value, sizeof(T));
    Attribute<T>::dirty = true;
    schedule_update(this);
    return *this;
  }
};

struct zb_zcl_basic_attrs_rich_t {
  static const constexpr uint8_t R = ZB_ZCL_CLUSTER_SERVER_ROLE;
  static const constexpr uint16_t C = ZB_ZCL_CLUSTER_ID_BASIC;

  zb_zcl_basic_attrs_rich_t() {
    ZB_ZCL_SET_STRING_VAL(mf_name, BASIC_MANUF_NAME, ZB_ZCL_STRING_CONST_SIZE(BASIC_MANUF_NAME));
    ZB_ZCL_SET_STRING_VAL(model_id, BASIC_MODEL_ID, ZB_ZCL_STRING_CONST_SIZE(BASIC_MODEL_ID));
    ZB_ZCL_SET_STRING_VAL(date_code, BASIC_DATE_CODE, ZB_ZCL_STRING_CONST_SIZE(BASIC_DATE_CODE));
    ZB_ZCL_SET_STRING_VAL(location_id, BASIC_LOCATION_DESC, ZB_ZCL_STRING_CONST_SIZE(BASIC_LOCATION_DESC));
    ZB_ZCL_SET_STRING_VAL(sw_ver, BASIC_SW_BUILD_ID, ZB_ZCL_STRING_CONST_SIZE(BASIC_SW_BUILD_ID));
  }

  Attribute<zb_uint8_t> zcl_version = Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, ZB_ZCL_VERSION);
  Attribute<zb_uint8_t> app_version =
      Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, BASIC_APP_VERSION);
  Attribute<zb_uint8_t> stack_version =
      Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, BASIC_STACK_VERSION);
  Attribute<zb_uint8_t> hw_version = Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, BASIC_HW_VERSION);
  ArrayAttribute<zb_char_t[33]> mf_name = ArrayAttribute<zb_char_t[33]>(R, C, ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                                                        ZB_ZCL_BASIC_MANUFACTURER_NAME_DEFAULT_VALUE);
  ArrayAttribute<zb_char_t[33]> model_id = ArrayAttribute<zb_char_t[33]>(R, C, ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                                                         ZB_ZCL_BASIC_MODEL_IDENTIFIER_DEFAULT_VALUE);
  ArrayAttribute<zb_char_t[17]> date_code =
      ArrayAttribute<zb_char_t[17]>(R, C, ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, ZB_ZCL_BASIC_DATE_CODE_DEFAULT_VALUE);
  Attribute<zb_uint8_t> power_source =
      Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, BASIC_POWER_SOURCE);
  ArrayAttribute<zb_char_t[17]> location_id = ArrayAttribute<zb_char_t[17]>(
      R, C, ZB_ZCL_ATTR_BASIC_LOCATION_DESCRIPTION_ID, ZB_ZCL_BASIC_LOCATION_DESCRIPTION_DEFAULT_VALUE);
  Attribute<zb_uint8_t> ph_env = Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_BASIC_PHYSICAL_ENVIRONMENT_ID, BASIC_PH_ENV);
  ArrayAttribute<zb_char_t[17]> sw_ver =
      ArrayAttribute<zb_char_t[17]>(R, C, ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, ZB_ZCL_BASIC_SW_BUILD_ID_DEFAULT_VALUE);
};

struct zb_zcl_thermostat_attrs_rich_t {
  static const constexpr uint8_t R = ZB_ZCL_CLUSTER_SERVER_ROLE;
  static const constexpr uint16_t C = ZB_ZCL_CLUSTER_ID_THERMOSTAT;

  Attribute<zb_int16_t> local_temperature = Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
                                                                  ZB_ZCL_THERMOSTAT_LOCAL_TEMPERATURE_DEFAULT_VALUE);
  Attribute<zb_int16_t> outdoor_temperature =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID, 0x8000);
  Attribute<zb_int16_t> abs_min_heat_setpoint_limit =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_LIMIT_ID, 0);
  Attribute<zb_int16_t> abs_max_heat_setpoint_limit =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_LIMIT_ID, 10000);
  Attribute<zb_int16_t> abs_min_cool_setpoint_limit =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_COOL_SETPOINT_LIMIT_ID, 0);
  Attribute<zb_int16_t> abs_max_cool_setpoint_limit =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_COOL_SETPOINT_LIMIT_ID, 10000);
  Attribute<zb_uint8_t> PI_cooling_demand = Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_PI_COOLING_DEMAND_ID,
                                                                  ZB_ZCL_THERMOSTAT_PI_COOLING_DEMAND_MIN_VALUE);
  Attribute<zb_uint8_t> PI_heating_demand = Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID,
                                                                  ZB_ZCL_THERMOSTAT_PI_HEATING_DEMAND_MIN_VALUE);
  Attribute<zb_uint8_t> HVAC_system_type_configuration =
      Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_HVAC_SYSTEM_TYPE_CONFIGURATION_ID,
                            ZB_ZCL_THERMOSTAT_HVAC_SYSTEM_TYPE_CONFIGURATION_DEFAULT_VALUE);
  Attribute<zb_int8_t> local_temperature_calibration =
      Attribute<zb_int8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_CALIBRATION_ID,
                           ZB_ZCL_THERMOSTAT_LOCAL_TEMPERATURE_CALIBRATION_DEFAULT_VALUE);
  Attribute<zb_int16_t> occupied_cooling_setpoint =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, 10000);
  Attribute<zb_int16_t> occupied_heating_setpoint =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, 0x0BB8);
  Attribute<zb_int16_t> unoccupied_cooling_setpoint =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_COOLING_SETPOINT_ID, 10000);
  Attribute<zb_int16_t> unoccupied_heating_setpoint =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID, 500);
  Attribute<zb_int16_t> min_heat_setpoint_limit =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID, 0);
  Attribute<zb_int16_t> max_heat_setpoint_limit =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID, 10000);
  Attribute<zb_int16_t> min_cool_setpoint_limit =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID, 0);
  Attribute<zb_int16_t> max_cool_setpoint_limit =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID, 10000);
  Attribute<zb_int8_t> min_setpoint_dead_band = Attribute<zb_int8_t>(
      R, C, ZB_ZCL_ATTR_THERMOSTAT_MIN_SETPOINT_DEAD_BAND_ID, ZB_ZCL_THERMOSTAT_MIN_SETPOINT_DEAD_BAND_DEFAULT_VALUE);
  Attribute<zb_uint8_t> remote_sensing = Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID,
                                                               ZB_ZCL_THERMOSTAT_REMOTE_SENSING_DEFAULT_VALUE);
  Attribute<zb_uint8_t> control_seq_of_operation =
      Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_ID, 0x02);
  Attribute<zb_uint8_t> system_mode =
      Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF);

  Attribute<zb_uint8_t> start_of_week = Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_START_OF_WEEK_ID, 0x01);
  Attribute<zb_uint8_t> number_of_weekly_transitions =
      Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_NUMBER_OF_WEEKLY_TRANSITIONS_ID, MAX_SCHEDULER_TRANSITIONS);
  Attribute<zb_uint8_t> number_of_daily_transitions =
      Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_NUMBER_OF_DAILY_TRANSITIONS_ID, MAX_SCHEDULER_TRANSITIONS / 7);

  Attribute<zb_uint8_t> programming_operation_mode =
      Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_PROGRAMMING_OPERATION_MODE_ID, 0x00);

  Attribute<zb_uint32_t> extra_status =
      Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_EXTRA_STATUS_ID, 0x00000000);
  Attribute<zb_uint32_t> spike_protection_count =
      Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_SPIKE_PROTECTION_COUNT_ID, 0);

  Attribute<zb_uint32_t> tiny_cycles = Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_TINY_CYCLES_ID, 0);
  Attribute<zb_uint32_t> short_cycles = Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_SHORT_CYCLES_ID, 0);
  Attribute<zb_uint32_t> normal_cycles = Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_THERMOSTAT_NORMAL_CYCLES_ID, 0);
};

struct zb_zcl_opentherm_attrs_rich_t {
  static const constexpr uint8_t R = ZB_ZCL_CLUSTER_SERVER_ROLE;
  static const constexpr uint16_t C = ZB_ZCL_CLUSTER_ID_OPENTHERM;

  Attribute<zb_int16_t> version = Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_OPENTHERM_VERSION_ID, 0x0001);
  Attribute<zb_uint16_t> status = Attribute<zb_uint16_t>(R, C, ZB_ZCL_ATTR_OPENTHERM_STATUS_ID, 0x00);
  Attribute<zb_int16_t> flow_setpoint = Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_OPENTHERM_FLOW_SETPOINT_ID, 0x8000);
  Attribute<zb_int16_t> flow_temperature =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_OPENTHERM_FLOW_TEMPERATURE_ID, 0x8000);
  Attribute<zb_int16_t> return_temperature =
      Attribute<zb_int16_t>(R, C, ZB_ZCL_ATTR_OPENTHERM_RETURN_TEMPERATURE_ID, 0x8000);
  Attribute<zb_uint32_t> frames_dropped = Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_OPENTHERM_FRAMES_DROPPED_ID, 0);
  Attribute<zb_uint32_t> frames_late = Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_OPENTHERM_FRAMES_LATE_ID, 0);
  Attribute<zb_uint32_t> last_cmd_result = Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_OPENTHERM_LAST_CMD_RESULT_ID, 0U);
};

struct zb_zcl_time_attrs_rich_t {
  static const constexpr uint8_t R = ZB_ZCL_CLUSTER_SERVER_ROLE;
  static const constexpr uint16_t C = ZB_ZCL_CLUSTER_ID_TIME;

  Attribute<zb_uint32_t> time = Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_TIME_TIME_ID, ZB_ZCL_TIME_TIME_DEFAULT_VALUE);
  Attribute<zb_uint8_t> time_status =
      Attribute<zb_uint8_t>(R, C, ZB_ZCL_ATTR_TIME_TIME_STATUS_ID, ZB_ZCL_TIME_TIME_STATUS_DEFAULT_VALUE);
  Attribute<zb_int32_t> time_zone =
      Attribute<zb_int32_t>(R, C, ZB_ZCL_ATTR_TIME_TIME_ZONE_ID, ZB_ZCL_TIME_TIME_ZONE_DEFAULT_VALUE);
  Attribute<zb_uint32_t> dst_start =
      Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_TIME_DST_START_ID, ZB_ZCL_TIME_DST_START_DEFAULT_VALUE);
  Attribute<zb_uint32_t> dst_end =
      Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_TIME_DST_END_ID, ZB_ZCL_TIME_DST_END_DEFAULT_VALUE);
  Attribute<zb_uint32_t> dst_shift =
      Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_TIME_DST_SHIFT_ID, ZB_ZCL_TIME_DST_SHIFT_DEFAULT_VALUE);
  Attribute<zb_uint32_t> standard_time =
      Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_TIME_STANDARD_TIME_ID, ZB_ZCL_TIME_STANDARD_TIME_DEFAULT_VALUE);
  Attribute<zb_uint32_t> local_time =
      Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID, ZB_ZCL_TIME_LOCAL_TIME_DEFAULT_VALUE);
  Attribute<zb_uint32_t> last_set_time =
      Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_TIME_LAST_SET_TIME_ID, ZB_ZCL_TIME_LAST_SET_TIME_DEFAULT_VALUE);
  Attribute<zb_uint32_t> valid_until_time =
      Attribute<zb_uint32_t>(R, C, ZB_ZCL_ATTR_TIME_VALID_UNTIL_TIME_ID, ZB_ZCL_TIME_VALID_UNTIL_TIME_DEFAULT_VALUE);
};

struct device_ctx_t {
  zb_zcl_basic_attrs_rich_t basic;
  zb_zcl_thermostat_attrs_rich_t thermostat;
  zb_zcl_opentherm_attrs_rich_t opentherm;
  zb_zcl_time_attrs_rich_t time;
};

static device_ctx_t dev_ctx;

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(basic_attr_list, dev_ctx.basic.zcl_version.dptr(),
                                     dev_ctx.basic.app_version.dptr(), dev_ctx.basic.stack_version.dptr(),
                                     dev_ctx.basic.hw_version.dptr(), dev_ctx.basic.mf_name.dptr(),
                                     dev_ctx.basic.model_id.dptr(), dev_ctx.basic.date_code.dptr(),
                                     dev_ctx.basic.power_source.dptr(), dev_ctx.basic.location_id.dptr(),
                                     dev_ctx.basic.ph_env.dptr(), dev_ctx.basic.sw_ver.dptr());

ZB_ZCL_DECLARE_THERMOSTAT_ATTRIB_LIST_EXT_PLUS(
    thermostat_attr_list, dev_ctx.thermostat.local_temperature.dptr(), dev_ctx.thermostat.outdoor_temperature.dptr(),
    dev_ctx.thermostat.abs_min_heat_setpoint_limit.dptr(), dev_ctx.thermostat.abs_max_heat_setpoint_limit.dptr(),
    dev_ctx.thermostat.abs_min_cool_setpoint_limit.dptr(), dev_ctx.thermostat.abs_max_cool_setpoint_limit.dptr(),
    dev_ctx.thermostat.PI_cooling_demand.dptr(), dev_ctx.thermostat.PI_heating_demand.dptr(),
    dev_ctx.thermostat.HVAC_system_type_configuration.dptr(), dev_ctx.thermostat.local_temperature_calibration.dptr(),
    dev_ctx.thermostat.occupied_cooling_setpoint.dptr(), dev_ctx.thermostat.occupied_heating_setpoint.dptr(),
    dev_ctx.thermostat.unoccupied_cooling_setpoint.dptr(), dev_ctx.thermostat.unoccupied_heating_setpoint.dptr(),
    dev_ctx.thermostat.min_heat_setpoint_limit.dptr(), dev_ctx.thermostat.max_heat_setpoint_limit.dptr(),
    dev_ctx.thermostat.min_cool_setpoint_limit.dptr(), dev_ctx.thermostat.max_cool_setpoint_limit.dptr(),
    dev_ctx.thermostat.min_setpoint_dead_band.dptr(), dev_ctx.thermostat.remote_sensing.dptr(),
    dev_ctx.thermostat.control_seq_of_operation.dptr(), dev_ctx.thermostat.system_mode,
    dev_ctx.thermostat.start_of_week.dptr(), dev_ctx.thermostat.number_of_weekly_transitions.dptr(),
    dev_ctx.thermostat.number_of_daily_transitions.dptr(), dev_ctx.thermostat.programming_operation_mode.dptr(),
    dev_ctx.thermostat.extra_status.dptr(), dev_ctx.thermostat.spike_protection_count.dptr(),
    dev_ctx.thermostat.tiny_cycles.dptr(), dev_ctx.thermostat.short_cycles.dptr(),
    dev_ctx.thermostat.normal_cycles.dptr());

ZB_ZCL_DECLARE_OPENTHERM_ATTRIB_LIST(opentherm_attr_list, dev_ctx.opentherm.version.dptr(),
                                     dev_ctx.opentherm.status.dptr(), dev_ctx.opentherm.flow_setpoint.dptr(),
                                     dev_ctx.opentherm.flow_temperature.dptr(),
                                     dev_ctx.opentherm.return_temperature.dptr(),
                                     dev_ctx.opentherm.frames_dropped.dptr(), dev_ctx.opentherm.frames_late.dptr(),
                                     dev_ctx.opentherm.last_cmd_result.dptr(),
                                     dev_ctx.opentherm_attr.cluster_revision.dptr());

ZB_ZCL_DECLARE_TIME_ATTRIB_LIST(time_attr_list, dev_ctx.time.time.dptr(), dev_ctx.time.time_status.dptr(),
                                dev_ctx.time.time_zone.dptr(), dev_ctx.time.dst_start.dptr(),
                                dev_ctx.time.dst_end.dptr(), dev_ctx.time.dst_shift.dptr(),
                                dev_ctx.time.standard_time.dptr(), dev_ctx.time.local_time.dptr(),
                                dev_ctx.time.last_set_time.dptr(), dev_ctx.time.valid_until_time.dptr());

DECLARE_CLUSTER_LIST(clusterlist, basic_attr_list, thermostat_attr_list, opentherm_attr_list, time_attr_list);

DECLARE_ENDPOINT(endpoint, ENDPOINT_ID, clusterlist);

#define DECLARE_ENDPOINT_EMPTY(ep_name, ep_id)                                                                         \
  DECLARE_SIMPLE_DESC_EMPTY(ep_name, ep_id);                                                                           \
  ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info##ep_name, 0);                                                      \
  ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id, ZB_AF_HA_PROFILE_ID, 0, NULL, 0, NULL,                                   \
                              (zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name, 0, reporting_info##ep_name, 0, NULL)

#ifndef CONFIG_ZIGBEE_SHELL
ZBOSS_DECLARE_DEVICE_CTX_1_EP(device_context, endpoint);
#else
DECLARE_ENDPOINT_EMPTY(zb_shell_endpoint, CONFIG_ZIGBEE_SHELL_ENDPOINT);
ZBOSS_DECLARE_DEVICE_CTX_2_EP(device_context, endpoint, zb_shell_endpoint);
#endif

const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static wdt_timeout_cfg wdt_tcfg;
static int wdt_channel = 0;

void setup_watchdog() {
  wdt_tcfg = {.window = {.min = 0, .max = 60 * 1000},
              .callback =
                  [](const struct device *dev, int channel_id) {
                    LOG_ERR("watchdog triggered; resetting SoC\n");
                    LOG_PANIC();
                  },
              .flags = WDT_FLAG_RESET_SOC};
  wdt_channel = wdt_install_timeout(wdt, &wdt_tcfg);
  if (wdt_channel < 0)
    LOG_ERR("watchdog timer installation failed: error %d", wdt_channel);
  int r = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
  if (r != 0)
    LOG_ERR("watchdog setup failed: error %d", r);
}

static void initialize_clusters(void) {
  // Thermostat
  uint16_t nonstd_reportable_attrs[] = {ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID,
                                        ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
                                        ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID};

  for (const auto &aid : nonstd_reportable_attrs) {
    auto ad = zb_zcl_get_attr_desc_a(ENDPOINT_ID, ZB_ZCL_CLUSTER_ID_THERMOSTAT, ZB_ZCL_CLUSTER_SERVER_ROLE, aid);
    ad->access |= ZB_ZCL_ATTR_ACCESS_REPORTING;
  }

  zb_zcl_thermostat_init();

  zb_zcl_basic_init_client();
  zb_zcl_time_init_client();
  zb_zcl_thermostat_init_client();

  zb_zcl_basic_init_server();
  zb_zcl_time_init_server();
  zb_zcl_thermostat_init_server();
  zb_zcl_opentherm_init_server();
}

// The crudest RTC; a timer running at a 1sec period.
K_TIMER_DEFINE(
    rtc_timer, [](struct k_timer *) { dev_ctx.time.time++; }, NULL);

static zb_bool_t zb_zcl_set_real_time_clock(zb_uint32_t time);

static bool zb_utc_time_valid() {
  return dev_ctx.time.time != 0 && dev_ctx.time.time != ZB_ZCL_TIME_TIME_INVALID_VALUE;
}

zb_uint32_t zb_get_utc_time(void) {
  if (zb_utc_time_valid())
    return dev_ctx.time.time;
  else
    return ZB_TIME_BEACON_INTERVAL_TO_MSEC(ZB_TIMER_GET()) / 1000;
}

static void zb_zcl_time_sync_time_server_found_cb(zb_ret_t status, zb_uint32_t auth_level, zb_uint16_t short_addr,
                                                  zb_uint8_t ep, // Not the endpoint?
                                                  zb_uint32_t nw_time) {
  time_sync_in_progress = false;

  if (status == RET_TIMEOUT) {
    if (dev_ctx.time.time == ZB_ZCL_TIME_TIME_INVALID_VALUE)
      ZB_SCHEDULE_APP_CALLBACK([](uint8_t) { start_time_sync(); }, 0);
  } else if (status == RET_OK) {
    zb_zcl_set_real_time_clock(nw_time);
    LOG_INF("received network time from %04x:%u", short_addr, ep);
  } else {
    LOG_WRN("time sync: status=%u auth_level=%u short_addr=%04x ep=%u nw_time=%08x", status, auth_level, short_addr, ep,
            nw_time);
  }
}

const struct device *chip_temp_dev = NULL;

static void chip_temp_timer_ftick(struct k_timer *) {
  if (chip_temp_dev) {
    struct sensor_value val;
    int rc = sensor_sample_fetch(chip_temp_dev);
    if (rc)
      LOG_ERR("Failed to fetch chip temp sample (error %d)", rc);
    else {
      rc = sensor_channel_get(chip_temp_dev, SENSOR_CHAN_DIE_TEMP, &val);
      if (rc)
        LOG_ERR("Failed to get chip temp data (error %d)", rc);
      else
        dev_ctx.thermostat.local_temperature = sensor_value_to_double(&val) * 100.0;
    }
  }
}

K_TIMER_DEFINE(chip_temp_timer, chip_temp_timer_ftick, NULL);

static void start_chip_temp_timer() {
  chip_temp_dev = DEVICE_DT_GET_ONE(nordic_nrf_temp);
  if (!device_is_ready(chip_temp_dev))
    LOG_ERR("chip temperature sensor is not ready");
  else
    k_timer_start(&chip_temp_timer, K_SECONDS(10), K_SECONDS(10));
}

#define K_THREAD_STACK_DECLARE_STATIC(sym, size)                                                                       \
  static struct z_thread_stack_element sym[Z_KERNEL_STACK_SIZE_ADJUST(size)];

class ZigbeeRealTime : public litt::RealTime {
public:
  ZigbeeRealTime() : litt::RealTime() {}
  virtual ~ZigbeeRealTime() = default;

  virtual bool get_weekday_minutes(uint8_t &weekday, uint16_t &minutes) const override {
    if (!zb_utc_time_valid())
      return false;

    uint32_t seconds = dev_ctx.time.local_time;
    if (seconds == ZB_ZCL_TIME_TIME_INVALID_VALUE)
      seconds = dev_ctx.time.standard_time;
    if (seconds == ZB_ZCL_TIME_TIME_INVALID_VALUE)
      seconds = dev_ctx.time.time;
    if (seconds == ZB_ZCL_TIME_TIME_INVALID_VALUE)
      return false;

    static const constexpr uint32_t seconds_per_day = 60 * 60 * 24;
    static const constexpr uint32_t saturday = 6; // Sunday == 0; 01-01-2000 was a Saturday.

    auto day = (seconds / seconds_per_day) % 7;
    minutes = (seconds % seconds_per_day) / 60;
    weekday = (saturday + day) % 7;

    return true;
  }
};

class MyTransport : public Master<ZephyrTimer, ZephyrMutex, ZephyrSemaphore, ZephyrTime, ZephyrQueue, ZephyrIO> {
public:
  MyTransport(const ZephyrPins &pins, const CentralHeatingInterface &chif) : Master(pins), state(INIT), chif(chif) {}

  virtual ~MyTransport() = default;

  Frame init_master_frames[2] = {Frame(ReadData, 3), Frame(ReadData, 0)};

  Frame slave_info_frames[7] = {Frame(ReadData, 3),       Frame(ReadData, 125), Frame(ReadData, 15),
                                Frame(ReadData, 127),     Frame(ReadData, 49),  Frame(ReadData, 57),
                                Frame(WriteData, 1, 0.0f)};

  Frame master_frames[18] = {Frame(ReadData, 0), Frame(ReadData, 10),      Frame(ReadData, 0), Frame(ReadData, 17),
                             Frame(ReadData, 0), Frame(ReadData, 18),      Frame(ReadData, 0), Frame(ReadData, 19),
                             Frame(ReadData, 0), Frame(ReadData, 25),      Frame(ReadData, 0), Frame(ReadData, 27),
                             Frame(ReadData, 0), Frame(ReadData, 28),      Frame(ReadData, 0), Frame(ReadData, 35),
                             Frame(ReadData, 0), Frame(WriteData, 1, 0.0f)};

  void advance() {
    auto next = [this](size_t sz, State next_state) {
      master_index = (master_index + 1) % (sz / sizeof(Frame));
      if (master_index == 0) {
        if (state != NORMAL && next_state == NORMAL)
          on_normal();
        state = next_state;
      }
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
    case 0: {
      f = Frame(ReadData, f.id(), status, 0x00);
      break;
    }
    case 1:
      f = Frame(f.msg_type(), f.id(), chif.flow_setpoint());
      break;
    }

    tx(f, true);
    advance();
    wdt_feed(wdt, wdt_channel);
  }

  virtual void on_normal() {
    // TODO: keep thermostat locked out until here.
    // TODO: Check that the boiler supports modulation.
  }

  virtual void on_opentherm_supported() override {
    LOG_INF("OpenTherm/plus detected (received %" PRIu64 " frames)", rx_frame_count);
  }

  virtual void on_opentherm_unsupported() override {
    LOG_ERR("no OpenTherm/plus reply after startup; the slave does not seem to "
            "support OpenTherm/plus.");
#ifdef NDEBUG
    LOG_WRN("OpenTherm/- not supported; giving up.");
    Master::on_opentherm_unsupported();
#else
    LOG_INF("not giving up, though.");
#endif
  }

  virtual void on_dropped_frame(RequestID rid) const override { dev_ctx.opentherm.frames_dropped++; }

  virtual void on_late_frame(RequestID rid) const override { dev_ctx.opentherm.frames_late++; }

  void cease() {
    master_timer.stop();
    plus_check_timer.stop();
    io.cease();
  }

protected:
  enum State { INIT, SLAVE_INFO, NORMAL };
  State state = INIT;
  size_t master_index = 0;
  const CentralHeatingInterface &chif;
};

#define USE_DEMAND_DRIVEN_THERMOSTAT

#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
using MyThermostat = DemandDrivenThermostat<MAX_DEMANDERS, ZephyrTime, ZephyrTimer>;
#else
using MyThermostat = PIDDrivenThermostat<MAX_DEMANDERS, ZephyrTime, ZephyrTimer>;
#endif

using MyScheduler = Scheduler<MAX_SCHEDULER_TRANSITIONS, ZephyrTimer, ZigbeeRealTime>;

class MyApp : public RichApplication, public MyThermostat, public MyScheduler {

#pragma pack(push, 1)
  struct Configuration {
    MyThermostat::Configuration thermostat;
    MyScheduler::Configuration scheduler;

    bool serialize(uint8_t *buf, size_t sz) const {
      return thermostat.serialize(buf, sz) && scheduler.serialize(buf, sz);
    }

    bool deserialize(const uint8_t *buf, size_t sz) {
      return thermostat.deserialize(buf, sz) && scheduler.deserialize(buf, sz);
    }

    size_t serialized_size() const { return thermostat.serialized_size() + scheduler.serialized_size(); }
  };
#pragma pack(pop)

public:
  Configuration configuration;

  MyApp(const ZephyrPins &pins)
      : RichApplication(transport),
#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
        MyThermostat(configuration.thermostat, boiler.ch1, MyThermostat::MixingFunction::AVERAGE),
#else
        MyThermostat(configuration.thermostat, boiler.ch1, {2.75f, 0.0f}, 21.0f, MyThermostat::MixingFunction::AVERAGE),
#endif
        MyScheduler(configuration.scheduler), transport(pins, boiler.ch1), boiler(transport, *this),
        statistics_timer(30e6, 60 * 60 * 1e6, statistics_ftick, nullptr, this) {

    transport.set_frame_callback(RichApplication::sprocess, this);

    for (size_t i = 0; i < max_ot_reqs; i++)
      ot_req_return_addr[i].rid = NoRequestID;
  }

  virtual ~MyApp() = default;

  virtual void run() override {
    MyThermostat::on_mode_change(mode, mode);

    // Initial values for OpenTherm data IDs.
    tset = MyThermostat::flow_setpoint();

    // Start the OpenTherm master transport timers & threads.
    transport.start();

    tx_thread_tid = k_thread_create(&tx_thread, tx_thread_stack, K_THREAD_STACK_SIZEOF(tx_thread_stack), tx_thread_fun,
                                    this, NULL, NULL, -10, K_ESSENTIAL, K_NO_WAIT);

    rx_thread_tid = k_thread_create(&rx_thread, rx_thread_stack, K_THREAD_STACK_SIZEOF(rx_thread_stack), rx_thread_fun,
                                    this, NULL, NULL, -10, K_ESSENTIAL, K_NO_WAIT);

    statistics_timer.start();
  }

  virtual uint8_t transport_status() const { return transport.get_status(); }

  virtual void on_master_status_change(uint8_t from, uint8_t to) override {
    LOG_DBG("master status := %02x", to);
    RichApplication::on_master_status_change(from, to);

    dev_ctx.opentherm.status = (to << 8) | (dev_ctx.opentherm.status & 0x00FF);
  }

  virtual void on_slave_status_change(uint8_t from, uint8_t to) override {
    LOG_DBG("slave status := %02x", to);
    RichApplication::on_slave_status_change(from, to);

    dev_ctx.opentherm.status = (dev_ctx.opentherm.status & 0xFF00) | to;
  }

  virtual void on_fault_indication() override {
    LOG_WRN("slave indicates a fault; readings ASF flags");
    transport.tx(Frame(ReadData, asf_flags.nr));
  }

  virtual void on_diagnostic_indication() override {
    LOG_WRN("slave diagnostic indication; readings OEM diagnostic code");
    RichApplication::on_diagnostic_indication();

    transport.tx(Frame(ReadData, oem_diagnostic_code.nr));
  }

  virtual void on_flame_change(bool on) override {
    LOG_DBG("flame := %s", on ? "on" : "off");
    RichApplication::on_flame_change(on);

    MyThermostat::on_flame_change(on);
    dev_ctx.thermostat.tiny_cycles = statistics.tiny_cycles;
    dev_ctx.thermostat.short_cycles = statistics.short_cycles;
    dev_ctx.thermostat.normal_cycles = statistics.normal_cycles;
  }

  virtual void on_mode_change(Thermostat::Mode from, Thermostat::Mode to) override {
    LOG_INF("thermostat mode := %s", mode_string(to));
    MyThermostat::on_mode_change(from, to);

    if (to == Thermostat::Mode::OFF) {
      if (dev_ctx.thermostat.system_mode != ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF)
        dev_ctx.thermostat.PI_heating_demand = 0;
      dev_ctx.thermostat.system_mode = ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
    } else {
      dev_ctx.thermostat.system_mode = ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT;
#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
      dev_ctx.thermostat.PI_heating_demand = (zb_uint8_t)roundf(mixed_demand() * 100.0f);
#else
// TODO
#endif
    }
  }

  virtual void on_spike_protect(bool on) override {
    LOG_INF("spike protect := %s", on ? "on" : "off");
    MyThermostat::on_spike_protect(on);

    if (on) {
      dev_ctx.thermostat.extra_status |= 0x00000001;
      dev_ctx.thermostat.spike_protection_count = statistics.num_spike_protected;
    } else
      dev_ctx.thermostat.extra_status &= 0xFFFFFFFE;
  }

  virtual void on_max_flow_setpoint_bounds_change(const FlowSetpointBounds &from,
                                                  const FlowSetpointBounds &to) override {
    LOG_DBG("max flow setpoint bounds := [%u,%u]", to.lower_bound, to.upper_bound);
    MyThermostat::set_max_flow_setpoint_bounds(to.lower_bound, to.upper_bound);
    // Check: f88 conversion ok?
    transport.tx(Frame(WriteData, maxtset.nr, to_f88(((int)to.upper_bound) << 8)));
  }

  virtual void on_flow_setpoint_change(float from, float to, bool approximated) override {
    LOG_INF("flow setpoint := %0.2f %s", to, (approximated ? "(approx.)" : ""));
    MyThermostat::on_flow_setpoint_change(from, to, approximated);

    zb_int16_t rounded = (zb_int16_t)roundf(to * 100.0f);
    dev_ctx.thermostat.occupied_heating_setpoint = rounded;
    dev_ctx.opentherm.flow_setpoint = rounded;
  }

  virtual void on_flow_temperature_change(float from, float to) override {
    MyThermostat::on_flow_temperature_change(from, to);

    dev_ctx.opentherm.flow_temperature = (zb_uint16_t)roundf(to * 100.0f);

    if (zb_buf_memory_low())
      LOG_WRN("zboss buffer memory low!");
    if (zb_buf_is_oom_state())
      LOG_WRN("zboss out of memory!");
  }

  virtual void on_return_temperature_change(float from, float to) override {
    dev_ctx.opentherm.return_temperature = (zb_uint16_t)roundf(to * 100.0f);
  }

#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
  virtual void on_mixed_demand_change(float from, float to) override {
    LOG_INF("mixed demand := %0.2f", to);
    MyThermostat::on_mixed_demand_change(from, to);

    if (dev_ctx.thermostat.system_mode != ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF)
      dev_ctx.thermostat.PI_heating_demand = (zb_uint8_t)roundf(to * 100.0f);
  }
#endif

  virtual void on_outside_temperature_change(float from, float to) override {
    RichApplication::on_outside_temperature_change(from, to);
    dev_ctx.thermostat.outdoor_temperature = (zb_uint16_t)roundf(to * 100.0f);
  }

  virtual bool process(const Frame &f) override {
    bool r = RichApplication::process(f);
    dev_ctx.opentherm.last_cmd_result = f;
    return r;
  }

  virtual void on_mixing_function_change(MixingFunction function) override {
    LOG_INF("mixing function := %s", mixing_function_name(function));
    MyThermostat::on_mixing_function_change(function);
  }

  virtual void set_opentherm_status(uint8_t status) {
    if (status & 0x01)
      MyThermostat::disengage_protection();

    transport.set_status(status);
  }

  virtual bool on_set_opentherm_status_cmd(uint16_t user_data, uint8_t status) {
    set_opentherm_status(status);
    return true;
  }

  struct OTRequestReturnAddr {
    RequestID rid;
    zb_zcl_addr_t addr;
    uint8_t ep;
  };

  static constexpr size_t max_ot_reqs = 16;
  OTRequestReturnAddr ot_req_return_addr[max_ot_reqs];
  volatile size_t ot_req_return_addr_inx = 0;
  ZephyrMutex ot_req_mtx;

  void opentherm_request(const OpenTherm::Frame &frame, const zb_zcl_addr_t &return_addr, uint8_t dst_ep) {
    LockGuard g(ot_req_mtx);

    if (ot_req_return_addr[ot_req_return_addr_inx].rid != NoRequestID)
      LOG_WRN("queue full, dropping OpenTherm request");
    else {
      auto rid = transport.tx(
          frame, false,
          [](Application *appp, RequestStatus status, RequestID rid, const Frame &response) {
            auto app = static_cast<MyApp *>(appp);
            if (status == RequestStatus::OK) {
              LockGuard g(app->ot_req_mtx);
              for (size_t i = 0; i < max_ot_reqs; i++) {
                auto &ret_i = app->ot_req_return_addr[i];
                if (ret_i.rid == rid) {
                  zb_zcl_opentherm_send_request_response(&ret_i.addr, ret_i.ep, ENDPOINT_ID, ZB_AF_HA_PROFILE_ID,
                                                         BASIC_MANUF_ID, response);
                  ret_i.rid = NoRequestID;
                  break;
                }
              }
            } else
              LOG_WRN("OpenTherm request %lld failed with status %d", rid, (int)status);
          },
          this);

      if (rid != NoRequestID) {
        ot_req_return_addr[ot_req_return_addr_inx] = {.rid = rid, .addr = return_addr, .ep = dst_ep};
        ot_req_return_addr_inx = (ot_req_return_addr_inx + 1) % max_ot_reqs;
      }
    }
  }

  void cease() { transport.cease(); }

#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
  float mixed_demand() const { return MyThermostat::mixed_demand(); }
#endif

  virtual bool execute(const CommandFrame &f) override {
    using namespace OpenTherm;

    auto cmd_id = f.command_id;
    switch (cmd_id) {
    case CommandID::INVALID:
      return on_invalid_cmd(f.user_data, f.payload);
    case CommandID::LOAD_CONFIG:
      return on_load_config_cmd(f.user_data);
    case CommandID::SAVE_CONFIG:
      return on_save_config_cmd(f.user_data);
    case CommandID::SET_OPENTHERM_STATUS:
      return on_set_opentherm_status_cmd(f.user_data, f.user_data);

    default:
      return MyThermostat::execute(f);
    }
  }

  virtual bool on_invalid_cmd(uint16_t user_data, uint32_t payload) {
    LOG_ERR("invalid command: %08x", payload);
    return true;
  }

  virtual bool on_load_config_cmd(uint16_t user_data) { return false; }

  virtual bool on_save_config_cmd(uint16_t user_data) { return false; }

#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
  bool report_demand(const zb_ieee_addr_t &addr, uint8_t endpoint, float demand) {
    uint8_t id = demander_addr2id(addr);
    if (id < MAX_DEMANDERS && demands[id].level != demand)
      LOG_INF("demand(%s/%04x) := %0.2f", addr_to_string(addresses[id]), zb_address_short_by_ieee(addresses[id]),
              demand);
    return MyThermostat::report_demand(id, demand);
  }
#endif

  bool report_temperature(const zb_ieee_addr_t &addr, uint8_t endpoint, float temperature) {
    uint8_t id = demander_addr2id(addr);
    rtrvs[id].local_temperature = temperature;

    LOG_DBG("%s (%04x): %0.2f ~ %0.2f C", addr_to_string(addr),
            zb_address_short_by_ieee(const_cast<zb_ieee_addr_t &>(addr)), rtrvs[id].local_temperature,
            rtrvs[id].setpoint);

    if (isnan(rtrvs[id].setpoint))
      read_attribute(addr, endpoint, ZB_ZCL_CLUSTER_ID_THERMOSTAT, ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID);

#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
    return true;
#else
    return MyThermostat::report_temperature(id, temperature);
#endif
  }

  bool report_setpoint(const zb_ieee_addr_t &addr, uint8_t endpoint, float temperature) {
    uint8_t id = demander_addr2id(addr);
    rtrvs[id].setpoint = temperature;

    LOG_DBG("%s (%04x): %0.2f ~ %0.2f C", addr_to_string(addr),
            zb_address_short_by_ieee(const_cast<zb_ieee_addr_t &>(addr)), rtrvs[id].local_temperature,
            rtrvs[id].setpoint);

    if (isnan(rtrvs[id].local_temperature))
      read_attribute(addr, endpoint, ZB_ZCL_CLUSTER_ID_THERMOSTAT, ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID);

#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
    return true;
#else
    return MyThermostat::report_setpoint(id, temperature);
#endif
  }

  virtual void on_scheduler_start() override {
    LOG_DBG("scheduler := on");
    MyScheduler::on_scheduler_start();

    dev_ctx.thermostat.programming_operation_mode |= ZB_ZCL_THERMOSTAT_SCHEDULE_PROGRAMMING_MODE_BIT;
  }

  virtual void on_scheduler_stop() override {
    LOG_DBG("scheduler := off");
    MyScheduler::on_scheduler_stop();

    dev_ctx.thermostat.programming_operation_mode &= ~ZB_ZCL_THERMOSTAT_SCHEDULE_PROGRAMMING_MODE_BIT;
  }

  virtual void on_scheduled_heat_setpoint_change(float temperature) override {
    MyScheduler::on_scheduled_heat_setpoint_change(temperature);

    auto m =
        temperature <= configuration.thermostat.min_flow_setpoint ? Thermostat::Mode::OFF : Thermostat::Mode::AUTOMATIC;
    LOG_DBG("(scheduled) mode := %s (t=%.2f)", mode_string(m), temperature);
    set_mode(m);
  }

protected:
  MyTransport transport;
  OpenTherm::BoilerInterface<MyTransport> boiler;
  ZephyrTimer statistics_timer;

  K_THREAD_STACK_DECLARE_STATIC(tx_thread_stack, 4096);
  struct k_thread tx_thread;
  k_tid_t tx_thread_tid = 0;

  static void tx_thread_fun(void *appp, void *, void *) {
    auto app = static_cast<MyApp *>(appp);
    k_thread_name_set(k_current_get(), "littZ TX");
    app->transport.tx_forever();
  }

  K_THREAD_STACK_DECLARE_STATIC(rx_thread_stack, 4096);
  struct k_thread rx_thread;
  k_tid_t rx_thread_tid = 0;

  static void rx_thread_fun(void *appp, void *, void *) {
    auto app = static_cast<MyApp *>(appp);
    k_thread_name_set(k_current_get(), "littZ RX");
    app->transport.rx_forever([](bool v) { gpio_pin_set_dt(&leds[1], v ? 1 : 0); });
  }

  static bool statistics_ftick(Timer *, void *obj) {
    auto app = static_cast<MyApp *>(obj);
    app->get_statistics();
    return true;
  }

  void get_statistics() {
    for (size_t i = 116; i <= 123; i++)
      transport.tx(Frame(ReadData, i), false, [](Application *, RequestStatus, RequestID, const Frame &response) {});
  }

  zb_ieee_addr_t addresses[MAX_DEMANDERS];
  size_t num_addresses = 0;

public:
  struct RemoteTRV {
    float local_temperature;
    float setpoint;
  };
  RemoteTRV rtrvs[MAX_DEMANDERS];

  uint8_t demander_addr2id(const zb_ieee_addr_t &addr) {
    for (size_t i = 0; i < num_addresses; i++)
      if (ZB_64BIT_ADDR_CMP(addresses[i], addr))
        return i;
    if (num_addresses < MAX_DEMANDERS) {
      ZB_64BIT_ADDR_COPY(addresses[num_addresses], addr);
      rtrvs[num_addresses].local_temperature = nanf("");
      rtrvs[num_addresses].setpoint = nanf("");
      return num_addresses++;
    } else {
      LOG_WRN("too many network addresses to track: %s", addr_to_string(addr));
      // TODO: Remove the oldest demander?
      return 0xFF;
    }
  }

#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
  void show_demanders(const struct shell *sh) {
    // TODO: should not be a member.
    auto now = Thermostat::time.get_us();
    shell_fprintf(sh, SHELL_INFO, "[idx] ext_addr         short_addr age        level t     s/p\n");
    for (size_t i = 0; i < num_addresses; i++) {
      auto &addr = addresses[i];
      auto &d = demands[i];
      auto &rtrv = rtrvs[i];
      unsigned age_sec = (now - d.time) / 1e6f;
      unsigned age_hrs = age_sec / 3600;
      age_sec %= 3600;
      unsigned age_mins = age_sec / 60;
      age_sec %= 60;
      shell_fprintf(sh, SHELL_INFO, "[%3u] %16s     0x%04x   %02u:%02u:%02u %5.2f %5.2f %5.2f\n", i,
                    addr_to_string(addr), zb_address_short_by_ieee(addr), age_hrs, age_mins, age_sec, d.level,
                    rtrv.local_temperature, rtrv.setpoint);
    }
  }
#else
  void show_demanders(const struct shell *sh) {
    // TODO: should not be a member.
    auto now = Thermostat::time.get_us();
    shell_fprintf(sh, SHELL_INFO, "[idx] ext_addr         short_addr age        weight t     s/p\n");
    for (size_t i = 0; i < num_addresses; i++) {
      auto &addr = addresses[i];
      unsigned age_sec = (now - room_temperature_time(i)) / 1e6f;
      unsigned age_hrs = age_sec / 3600;
      age_sec %= 3600;
      unsigned age_mins = age_sec / 60;
      age_sec %= 60;
      shell_fprintf(sh, SHELL_INFO, "[%3u] %16s     0x%04x   %02u:%02u:%02u %5.2f %5.2f %5.2f\n", i,
                    addr_to_string(addr), zb_address_short_by_ieee(addr), age_hrs, age_mins, age_sec, pid_weight(i),
                    room_temperature(i), room_setpoint(i));
    }
  }
#endif

protected:
#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
  uint64_t demander_timeout_minutes = 360;

  virtual void check_demander_timeouts() {
    auto now = Thermostat::time.get_us();
    for (size_t i = 0; i < num_addresses; i++) {
      const Demand &d = demands[i];
      auto age_us = now - d.time;
      if (age_us >= demander_timeout_minutes * 60.0f * 1e6f)
        on_demander_timeout(i);
    }
  }

  virtual void on_demander_timeout(uint8_t id) {
    if (id < num_addresses) {
      LOG_DBG("demander timeout: %s (%u)", addr_to_string(addresses[id]), id);

      for (size_t i = id; i < num_addresses - 1; i++) {
        ZB_64BIT_ADDR_COPY(addresses[i], addresses[i + 1]);
        rtrvs[i] = rtrvs[i + 1];
      }

      num_addresses--;
    }

    MyThermostat::delete_demander(id);
  }
#endif
};

K_THREAD_STACK_DEFINE(MyApp::tx_thread_stack, K_THREAD_STACK_SIZEOF(MyApp::tx_thread_stack));
K_THREAD_STACK_DEFINE(MyApp::rx_thread_stack, K_THREAD_STACK_SIZEOF(MyApp::rx_thread_stack));

MyApp &app() {
  // The initialization order of global static C++ objects is undefined, but
  // function-local statics are created at the time of first use, so we keep it in here.
  static MyApp *r = nullptr;
  if (!r) {
    static ZephyrPins pins = {.rx = GPIO_DT_SPEC_GET(OT_IN_NODE, gpios),
                              .tx = GPIO_DT_SPEC_GET(OT_OUT_NODE, gpios),
                              .owned = true,
                              .tx_inverted = true};
    r = new MyApp(pins);
  }
  __ASSERT(r != nullptr, "failed to allocate app");
  return *r;
}

static zb_bool_t zb_zcl_set_real_time_clock(zb_uint32_t time) {
  bool is_different = dev_ctx.time.time == time;
  bool was_scheduling = app().is_scheduling();

  if (is_different && was_scheduling)
    app().stop_scheduling();

  dev_ctx.time.time = time;
  dev_ctx.time.time_status = 0x02;
  // TODO: Add support for timezones?

  k_timer_start(&rtc_timer, K_NO_WAIT, K_SECONDS(1));

  if (is_different && was_scheduling)
    app().start_scheduling();

  return ZB_TRUE;
}

static void zcl_device_cb(zb_uint8_t bufid) {
  zb_zcl_device_callback_param_t *cb_param = ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

  if (!cb_param)
    return;

  LOG_DBG("ZCL device callback: endpoint=%02x, device_cb_id=%02x", cb_param->endpoint, cb_param->device_cb_id);

  cb_param->status = RET_OK;

  if (cb_param->endpoint == ENDPOINT_ID) {
    switch (cb_param->device_cb_id) {
    case ZB_ZCL_SET_ATTR_VALUE_CB_ID: /* write attribute */
      switch (cb_param->cb_param.set_attr_value_param.cluster_id) {
      case ZB_ZCL_CLUSTER_ID_OPENTHERM:
        switch (cb_param->cb_param.set_attr_value_param.attr_id) {
        case ZB_ZCL_ATTR_OPENTHERM_STATUS_ID: {
          auto nv = cb_param->cb_param.set_attr_value_param.values.data16 >> 8;
          app().set_opentherm_status(nv);
        } break;
        }
      case ZB_ZCL_CLUSTER_ID_THERMOSTAT:
        switch (cb_param->cb_param.set_attr_value_param.attr_id) {
        case ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID: {
          auto nv = cb_param->cb_param.set_attr_value_param.values.data8;
          if (nv != ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF && nv != ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT)
            dev_ctx.thermostat.system_mode = nv = ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
          app().set_mode(nv == ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF ? MyThermostat::Mode::OFF
                                                                 : MyThermostat::Mode::MANUAL);
          break;
        }
        }
        break;
      }
      break;
    default:
      LOG_DBG("unhandled device callback ID: %02x", cb_param->device_cb_id);
      cb_param->status = RET_ERROR;
    }
  }
}

static zb_ret_t get_zb_zcl_ieee_addr(zb_zcl_addr_t *addr, zb_ieee_addr_t &ieee_addr) {
  if (addr->addr_type == ZB_ZCL_ADDR_TYPE_IEEE) {
    ZB_64BIT_ADDR_COPY(addr, ieee_addr);
    return RET_OK;
  } else
    return zb_address_ieee_by_short(addr->u.short_addr, ieee_addr);
}

static zb_ret_t handle_clear_weekly_schedule(uint8_t bufid, const zb_zcl_parsed_hdr_t *cmd_info) {
  app().clear_schedule();
  return RET_OK;
}

static zb_ret_t handle_get_weekly_schedule(uint8_t bufid, const zb_zcl_parsed_hdr_t *cmd_info) {
  zb_zcl_thermostat_get_weekly_schedule_req_t req;
  zb_zcl_parse_status_t status;

  ZB_ZCL_THERMOSTAT_GET_GET_WEEKLY_SCHEDULE_REQ(bufid, req, status);

  if (status != ZB_ZCL_PARSE_STATUS_SUCCESS)
    return RET_INVALID_PARAMETER;
  else {
    zb_uint8_t *cmd_ptr = (zb_uint8_t *)ZB_ZCL_START_PACKET(bufid);
    ZB_ZCL_CONSTRUCT_SPECIFIC_COMMAND_RES_FRAME_CONTROL(cmd_ptr);
    ZB_ZCL_CONSTRUCT_COMMAND_HEADER(cmd_ptr, cmd_info->seq_number, ZB_ZCL_CMD_THERMOSTAT_GET_WEEKLY_SCHEDULE_RESP);
    zb_uint8_t *count_ptr = cmd_ptr;
    ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, 0);
    ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, req.days_to_return);
    ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, req.mode_to_return);

    uint8_t days = req.days_to_return & 0x7F; // TODO: Support for away/vacation?

    for (size_t i = 0; i < app().configuration.scheduler.num_transitions; i++) {
      const auto &t = app().configuration.scheduler.transitions[i];
      if ((days & t.days) == days) {
        auto t_mode = static_cast<uint8_t>(t.mode);
        if (t_mode & req.mode_to_return) {
          cmd_ptr = (zb_uint8_t *)zb_put_next_htole16(cmd_ptr, t.time);
          if (req.mode_to_return & ZB_ZCL_THERMOSTAT_WEEKLY_SCHEDULE_MODE_FOR_SEQ_HEAT)
            cmd_ptr = (zb_uint8_t *)zb_put_next_htole16(cmd_ptr, (uint16_t)(t.heat_setpoint * 100.0f));
          if (req.mode_to_return & ZB_ZCL_THERMOSTAT_WEEKLY_SCHEDULE_MODE_FOR_SEQ_COOL)
            cmd_ptr = (zb_uint8_t *)zb_put_next_htole16(cmd_ptr, (uint16_t)(t.cool_setpoint * 100.0f));
          (*count_ptr)++;
        }
      }
    }

    if (*count_ptr == 0)
      return RET_NOT_FOUND;
    else {
      ZB_ZCL_THERMOSTAT_SEND_GET_WEEKLY_SCHEDULE_RESP(
          bufid, cmd_ptr, ZB_ZCL_PARSED_HDR_SHORT_DATA(cmd_info).source.u.short_addr, ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
          ZB_ZCL_PARSED_HDR_SHORT_DATA(cmd_info).src_endpoint, ZB_ZCL_PARSED_HDR_SHORT_DATA(cmd_info).dst_endpoint,
          cmd_info->profile_id, NULL);
    }

    return RET_BUSY;
  }
}

static zb_ret_t handle_set_weekly_schedule(uint8_t bufid, const zb_zcl_parsed_hdr_t *cmd_info) {
  zb_zcl_thermostat_set_weekly_schedule_req_t req;
  zb_zcl_parse_status_t status;

  ZB_ZCL_THERMOSTAT_GET_SET_WEEKLY_SCHEDULE_REQ(bufid, req, status);

  LOG_DBG("new weekly schedule for day_of_week=%02x", req.day_of_week);

  if (status != ZB_ZCL_PARSE_STATUS_SUCCESS)
    return RET_INVALID_PARAMETER;
  else {
    zb_zcl_thermostat_weekly_schedule_point_pair_t pair;
    zb_zcl_parse_status_t status;
    zb_ret_t r = RET_OK;

    bool was_scheduling = app().is_scheduling();
    if (was_scheduling)
      app().stop_scheduling();

    app().remove_schedule(req.day_of_week);

    MyScheduler::Transition ts[req.num_of_transitions];

    for (size_t i = 0; i < req.num_of_transitions; i++) {
      ZB_ZCL_THERMOSTAT_GET_NEXT_WEEKLY_SCHEDULE_POINT_PAIR_REQ(bufid, req.mode_for_seq, pair, status);

      if (status != ZB_ZCL_PARSE_STATUS_SUCCESS)
        return RET_INVALID_PARAMETER;

      ts[i].time = pair.transition_time;
      if (req.mode_for_seq & ZB_ZCL_THERMOSTAT_WEEKLY_SCHEDULE_MODE_FOR_SEQ_HEAT)
        ts[i].heat_setpoint = pair.heat_set_point / 100.0f;
      if (req.mode_for_seq & ZB_ZCL_THERMOSTAT_WEEKLY_SCHEDULE_MODE_FOR_SEQ_COOL)
        ts[i].cool_setpoint = pair.cool_set_point / 100.0f;
    }

    if (!app().set_schedule(req.day_of_week, static_cast<MyScheduler::Mode>(req.mode_for_seq), &ts[0],
                            req.num_of_transitions)) {
      r = RET_NO_MEMORY;
    }

    if (was_scheduling)
      app().start_scheduling();

    if (r == RET_OK)
      r = ZB_SCHEDULE_APP_CALLBACK([](zb_uint8_t) { zb_nvram_write_dataset(ZB_NVRAM_APP_DATA1); }, 0);

    return r;
  }
}

static void send_default_response(zb_bufid_t bufid, const zb_zcl_parsed_hdr_t *cmd_info, zb_uint8_t status_code) {
  zb_uint8_t *ptr;
  ptr = (zb_uint8_t *)ZB_ZCL_START_PACKET(bufid);
  ZB_ZCL_CONSTRUCT_GENERAL_COMMAND_RESP_FRAME_CONTROL(ptr);
  ZB_ZCL_CONSTRUCT_COMMAND_HEADER(ptr, cmd_info->seq_number, ZB_ZCL_CMD_DEFAULT_RESP);
  *(ptr++) = cmd_info->cmd_id;
  *(ptr++) = status_code;
  ZB_ZCL_FINISH_PACKET(bufid, ptr)
  ZB_ZCL_SEND_COMMAND_SHORT(bufid, ZB_ZCL_PARSED_HDR_SHORT_DATA(cmd_info).source.u.short_addr,
                            ZB_ZCL_PARSED_HDR_SHORT_DATA(cmd_info).dst_endpoint, ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                            ZB_ZCL_PARSED_HDR_SHORT_DATA(cmd_info).src_endpoint, cmd_info->profile_id,
                            ZB_ZCL_CLUSTER_ID_THERMOSTAT, NULL);
}

static zb_bool_t send_zb_zcl_response(zb_ret_t ret, zb_bufid_t bufid, const zb_zcl_parsed_hdr_t *cmd_info) {
  if (ret == RET_BUSY) {
    // Nothing.
  } else if (ret != RET_OK) {
    LOG_WRN("command failed: %d", ret);

    zb_uint8_t status_code;
    switch (ret) {
    case RET_NO_MEMORY:
    case RET_OUT_OF_RANGE:
      status_code = ZB_ZCL_STATUS_INSUFF_SPACE;
      break;
    case RET_NOT_FOUND:
      status_code = ZB_ZCL_STATUS_NOT_FOUND;
      break;
    case RET_INVALID_PARAMETER:
      status_code = ZB_ZCL_STATUS_INVALID_FIELD;
      break;
    default:
      status_code = ZB_ZCL_STATUS_FAIL;
      break;
    }

    send_default_response(bufid, cmd_info, status_code);
  } else if (!(cmd_info->disable_default_response))
    send_default_response(bufid, cmd_info, ZB_ZCL_STATUS_SUCCESS);
  else
    zb_buf_free(bufid);

  return ZB_TRUE;
}

static zb_uint8_t zb_endpoint_handler(zb_bufid_t bufid) {
  zb_zcl_parsed_hdr_t *cmd_infop = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);
  zb_zcl_parsed_hdr_t &cmd_info = *cmd_infop;

  if (cmd_info.cluster_id != ZB_ZCL_CLUSTER_ID_THERMOSTAT && cmd_info.cluster_id != ZB_ZCL_CLUSTER_ID_TIME)
    LOG_DBG("zb_endpoint_handler: dir=%d common=%d cluster=%04x cmd=%02x "
            "buflen=%d",
            cmd_info.cmd_direction, cmd_info.is_common_command, cmd_info.cluster_id, cmd_info.cmd_id,
            zb_buf_len(bufid));

  zb_uint8_t r = ZB_FALSE;

  if (!cmd_info.is_common_command) {
    switch (cmd_info.cluster_id) {
    case ZB_ZCL_CLUSTER_ID_OPENTHERM: {
      if (cmd_info.cmd_direction == ZB_ZCL_FRAME_DIRECTION_TO_SRV) {
        switch (cmd_info.cmd_id) {
        case ZB_ZCL_CMD_OPENTHERM_REQUEST: {
          uint8_t *data = (uint8_t *)zb_buf_begin(bufid);
          uint32_t f = 0;
          for (size_t i = 0; i < sizeof(uint32_t); i++)
            f = (f << 8) | data[i];
          app().opentherm_request(f, cmd_info.addr_data.common_data.source,
                                  cmd_info.addr_data.common_data.src_endpoint);
          zb_buf_free(bufid);
          r = ZB_TRUE;
          break;
        }
        default:
          LOG_WRN("zb_endpoint_handler: unknown OpenTherm cluster command %02x", cmd_info.cmd_id);
          r = ZB_FALSE;
        }
      }
      break;
    }
    case ZB_ZCL_CLUSTER_ID_THERMOSTAT: {
      if (cmd_info.cmd_direction == ZB_ZCL_FRAME_DIRECTION_TO_SRV) {
        switch (cmd_info.cmd_id) {
        case ZB_ZCL_CMD_THERMOSTAT_CLEAR_WEEKLY_SCHEDULE:
          r = send_zb_zcl_response(handle_clear_weekly_schedule(bufid, &cmd_info), bufid, &cmd_info);
          break;
        case ZB_ZCL_CMD_THERMOSTAT_GET_WEEKLY_SCHEDULE:
          r = send_zb_zcl_response(handle_get_weekly_schedule(bufid, &cmd_info), bufid, &cmd_info);
          break;
        case ZB_ZCL_CMD_THERMOSTAT_SET_WEEKLY_SCHEDULE:
          r = send_zb_zcl_response(handle_set_weekly_schedule(bufid, &cmd_info), bufid, &cmd_info);
          break;
        default:
          LOG_WRN("zb_endpoint_handler: unhandled thermostat cluster command %02x", cmd_info.cmd_id);
          r = ZB_FALSE;
        }
      }
      break;
    }
    default:;
    }
  } else {
    switch (cmd_info.cluster_id) {
    case ZB_ZCL_CLUSTER_ID_THERMOSTAT: {
      switch (cmd_info.cmd_id) {
      case ZB_ZCL_CMD_READ_ATTRIB_RESP: {
        zb_zcl_read_attr_res_t *attr_resp;
        do {
          ZB_ZCL_GENERAL_GET_NEXT_READ_ATTR_RES(bufid, attr_resp);
          if (attr_resp && attr_resp->status == ZB_ZCL_STATUS_SUCCESS) {
            zb_ieee_addr_t addr;
            get_zb_zcl_ieee_addr(&cmd_info.addr_data.common_data.source, addr);
            zb_uint8_t ep = cmd_info.addr_data.common_data.src_endpoint;

            switch (attr_resp->attr_id) {
#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
            case ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID:
              app().report_demand(addr, ep, *attr_resp->attr_value / 100.0f);
              break;
#endif
            case ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID:
              app().report_temperature(addr, ep, zb_zcl_attr_gets16(attr_resp->attr_value) / 100.0f);
              break;
            case ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID:
              app().report_setpoint(addr, ep, zb_zcl_attr_gets16(attr_resp->attr_value) / 100.0f);
              break;
            }
          }
        } while (attr_resp != NULL);
        break;
      }
      }
    }
    }
  }

  return r;
}

void report_attribute_cb(zb_zcl_addr_t *addr, zb_uint8_t ep, zb_uint16_t cluster_id, zb_uint16_t attr_id,
                         zb_uint8_t attr_type, zb_uint8_t *value) {
  zb_ieee_addr_t ieee_addr;
  if (get_zb_zcl_ieee_addr(addr, ieee_addr) == RET_OK && cluster_id == ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
    switch (attr_id) {
#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
    case ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID:
      app().report_demand(ieee_addr, ep, *value / 100.0f);
      break;
#endif
    case ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID:
      app().report_temperature(ieee_addr, ep, zb_zcl_attr_gets16(value) / 100.0f);
      break;
    case ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID:
      app().report_setpoint(ieee_addr, ep, zb_zcl_attr_gets16(value) / 100.0f);
      break;
    }
  }
}

void modify_attribute_cb(zb_uint8_t ep, zb_uint16_t cluster_id, zb_uint16_t attr_id, zb_uint8_t *value) {
  LOG_INF("modify_attribute_cb");
  // TODO: Save configuration?
  // TODO: Save schedule?
}

void zb_nvram_read_config(zb_uint8_t page, zb_uint32_t pos, zb_uint16_t payload_length) {
  // LOG_DBG("zb_nvram_read_config: page=%08x pos=%08u payload_length=%u", page, pos, payload_length);
  uint8_t buf[payload_length];
  const uint8_t *pbuf = &buf[0];
  size_t size = payload_length;
  zb_ret_t r = zb_nvram_read_data(page, pos, buf, payload_length);
  if (r != RET_OK)
    LOG_ERR("zb_osif_nvram_read: %d", r);
  else if (!app().configuration.deserialize(pbuf, size))
    LOG_ERR("configuration deserialization failed");
}

zb_uint16_t zb_nvram_get_config_size(void) {
  // Note: result must be a multiple of the word alignment, otherwise zboss/osif hard-faults.
  size_t sz = app().configuration.serialized_size();
  size_t max = zb_get_nvram_page_length();
  size_t r = sz > max ? 0 : sz;
  r += sizeof(int) - r % sizeof(int);
  // LOG_DBG("zb_nvram_get_config_size: r=%u", r);
  return r;
}

zb_ret_t zb_nvram_write_config(zb_uint8_t page, zb_uint32_t pos) {
  // LOG_DBG("zb_nvram_write_config: page=%08x pos=%08u", page, pos);
  size_t sz = zb_nvram_get_config_size();
  if (sz == 0)
    return RET_NO_MEMORY;
  uint8_t buf[sz];
  uint8_t *pbuf = &buf[0];
  if (!app().configuration.serialize(pbuf, sz))
    return RET_OPERATION_FAILED;
  return zb_nvram_write_data(page, pos, buf, sz);
}

static void setup_leds() {
  for (size_t i = 0; i < 3; i++) {
    if (!device_is_ready(leds[i].port))
      LOG_ERR("LED not ready.");

    int err = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_ACTIVE);
    if (err < 0)
      LOG_ERR("GPIO config failed.");

    err = gpio_pin_set_dt(&leds[i], 0);
    if (err < 0)
      LOG_ERR("gpio_pin_set_dt failed.");
  }
}

static void setup_settings() {
  int err = settings_subsys_init();
  if (err)
    LOG_ERR("settings initialization failed");

  err = settings_load();
  if (err)
    LOG_ERR("settings loading failed");
}

static void start_zigbee() {
  ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);
  ZB_AF_REGISTER_DEVICE_CTX(&device_context);
  ZB_ZCL_SET_REPORT_ATTR_CB(report_attribute_cb);
  ZB_ZCL_SET_MODIFY_ATTR_VALUE_CB(modify_attribute_cb);
  ZB_AF_SET_ENDPOINT_HANDLER(ENDPOINT_ID, zb_endpoint_handler);
  zb_nvram_register_app1_read_cb(zb_nvram_read_config);
  zb_nvram_register_app1_write_cb(zb_nvram_write_config, zb_nvram_get_config_size);
  ZB_ZCL_TIME_SET_REAL_TIME_CLOCK_CB(rtc_cb);

  initialize_clusters();

  zigbee_enable();
}

static void setup_log_timestamps() {
  if (log_set_timestamp_func(
          []() {
            if (zb_utc_time_valid()) {
              uint32_t seconds_since_2000 = dev_ctx.time.time;
              uint32_t seconds_today = seconds_since_2000 % 86400;
              return (log_timestamp_t)(((uint64_t)seconds_today) * 1000);
            } else
              return (log_timestamp_t)k_uptime_get();
          },
          1000U) != 0)
    LOG_ERR("could not set log timestamp function");
}

#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
static int sh_cmd_demand(const struct shell *sh, size_t argc, char **argv) {
  if (argc == 1) {
    shell_fprintf(sh, SHELL_ERROR, "mixed demand = %0.2f\n", app().mixed_demand());
    return 0;
  } else {
    if (argc != 4) {
      shell_fprintf(sh, SHELL_ERROR, "Invalid number of arguments\n");
      return -EINVAL;
    }

    uint16_t nwk_addr = 0;
    if (sscanf(argv[1], "%hx", &nwk_addr) != 1) {
      shell_fprintf(sh, SHELL_ERROR, "Invalid network address\n");
      return -EINVAL;
    }

    uint8_t endpoint = 0;
    if (sscanf(argv[2], "%s", &endpoint) != 1) {
      shell_fprintf(sh, SHELL_ERROR, "Invalid endpoint\n");
      return -EINVAL;
    }

    float demand = 0.0f;
    if (sscanf(argv[3], "%f", &demand) != 1 || demand < 0.0f || demand > 1.0f) {
      shell_fprintf(sh, SHELL_ERROR, "Invalid demand\n");
      return -EINVAL;
    }

    zb_ieee_addr_t ieee_addr;
    if (zb_address_ieee_by_short(nwk_addr, ieee_addr) != RET_OK) {
      shell_fprintf(sh, SHELL_ERROR, "IEEE address no known\n");
      return 1;
    } else {
      bool r = app().report_demand(ieee_addr, endpoint, demand);
      if (!r)
        shell_fprintf(sh, SHELL_ERROR, "Failed to execute command\n");
      return r ? 0 : 1;
    }
  }
}
#endif

static int sh_cmd_status(const struct shell *sh, size_t argc, char **argv) {
  if (argc == 1) {
    shell_fprintf(sh, SHELL_INFO, "%02x\n", app().transport_status());
    return 0;
  } else {
    if (argc != 2) {
      shell_fprintf(sh, SHELL_ERROR, "Invalid number of arguments\n");
      return -EINVAL;
    }

    int status = 0;
    if (sscanf(argv[1], "%02x", &status) != 1) {
      shell_fprintf(sh, SHELL_ERROR, "Invalid status\n");
      return -EINVAL;
    }

    app().set_opentherm_status(status);
    return 0;
  }
}

static int sh_cmd_mode(const struct shell *sh, size_t argc, char **argv) {
  if (argc == 1) {
    shell_fprintf(sh, SHELL_INFO, "%s\n", app().get_mode_string());
    return 0;
  } else {
    if (argc != 2) {
      shell_fprintf(sh, SHELL_ERROR, "Invalid number of arguments\n");
      return -EINVAL;
    }

    int mode = 0;
    if (sscanf(argv[1], "%02x", &mode) != 1 || mode >= static_cast<int>(MyThermostat::Mode::INVALID)) {
      shell_fprintf(sh, SHELL_ERROR, "Invalid mode\n");
      return -EINVAL;
    }

    auto m = static_cast<MyThermostat::Mode>(mode);
    app().set_mode(m);

    if (app().get_mode() != m) {
      shell_fprintf(sh, SHELL_ERROR, "Failed to set mode\n");
      return 1;
    } else
      return 0;
  }
}

static int sh_cmd_scheduler(const struct shell *sh, size_t argc, char **argv) {
  if (argc == 1) {
    shell_fprintf(sh, SHELL_INFO, "scheduler %s\n", app().is_scheduling() ? "on" : "off");

    shell_fprintf(sh, SHELL_INFO, "  - transitions: %s\n",
                  app().configuration.scheduler.num_transitions == 0 ? "None" : "");

    for (size_t i = 0; i < app().configuration.scheduler.num_transitions; i++) {
      const auto &t = app().configuration.scheduler.transitions[i];
      shell_fprintf(sh, SHELL_INFO, "    %02x %02u:%02u %02x % 6.2f % 6.2f\n", t.days, t.time / 60, t.time % 60,
                    static_cast<uint8_t>(t.mode), t.heat_setpoint, t.cool_setpoint);
    }

    if (!zb_utc_time_valid())
      shell_fprintf(sh, SHELL_INFO, "  - no time information yet\n");
    else if (app().configuration.scheduler.num_transitions > 0) {
      const auto tc = app().current_schedule_transition();
      shell_fprintf(sh, SHELL_INFO, "  - current: %02u:%02u % 6.2f % 6.2f\n", tc.time / 60, tc.time % 60,
                    tc.heat_setpoint, tc.cool_setpoint);

      uint16_t mn = app().minutes_until_next_scheduler_transition();
      if (mn == UINT16_MAX)
        shell_fprintf(sh, SHELL_INFO, "  - no transitions scheduled\n");
      else
        shell_fprintf(sh, SHELL_INFO, "  - time until next transition: %02u:%02u\n", mn / 60, mn % 60);
    }

    return 0;
  } else {
    if (argc != 2)
      shell_fprintf(sh, SHELL_ERROR, "Invalid number of arguments\n");

    bool mode = false;
    if (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "1") == 0)
      mode = true;
    else if (strcmp(argv[1], "off") == 0 || strcmp(argv[1], "0") == 0)
      mode = false;
    else {
      shell_fprintf(sh, SHELL_ERROR, "Invalid argument\n");
      return -EINVAL;
    }

    bool r = mode ? app().start_scheduling() : app().stop_scheduling();
    if (!r)
      shell_fprintf(sh, SHELL_ERROR, "Failed to %s scheduler\n", mode ? "start" : "stop");
    return r ? 0 : 1;
  }
}

static int sh_cmd_setpoint(const struct shell *sh, size_t argc, char **argv) {
  if (argc != 2) {
    shell_fprintf(sh, SHELL_ERROR, "Invalid number of arguments\n");
    return -EINVAL;
  } else {
    float temperature = 0.0f;
    if (sscanf(argv[1], "%f", &temperature) != 1) {
      shell_fprintf(sh, SHELL_ERROR, "Invalid temperature\n");
      return -EINVAL;
    }

    bool r = app().set_flow_setpoint(temperature);
    return r ? 0 : 1;
  }
}

static int sh_cmd_cease(const struct shell *sh, size_t argc, char **argv) {
  if (argc != 1) {
    shell_fprintf(sh, SHELL_ERROR, "Invalid number of arguments\n");
    return -EINVAL;
  }

  app().cease();
  return 0;
}

static int sh_cmd_find(const struct shell *sh, size_t argc, char **argv) {
  if (argc != 1) {
    shell_fprintf(sh, SHELL_ERROR, "Invalid number of arguments\n");
    return -EINVAL;
  }

  read_demand_from_bound_thermostats();
  return 0;
}

static int sh_cmd_demanders(const struct shell *sh, size_t argc, char **argv) {
  if (argc != 1) {
    shell_fprintf(sh, SHELL_ERROR, "Invalid number of arguments\n");
    return -EINVAL;
  }

  app().show_demanders(sh);
  return 0;
}

static int sh_cmd_set_mix(const struct shell *sh, size_t argc, char **argv) {
  if (argc == 1)
    shell_fprintf(sh, SHELL_INFO, "mixing function: %s\n", app().mixing_function_name(app().get_mixing_function()));
  else if (argc == 2) {
    int mf = 0;
    if (sscanf(argv[1], "%02x", &mf) != 1 || mf >= static_cast<int>(MyThermostat::MixingFunction::INVALID)) {
      shell_fprintf(sh, SHELL_ERROR, "Invalid mixing function\n");
      return -EINVAL;
    }
    bool r = app().set_mixing_function(static_cast<MyThermostat::MixingFunction>(mf));
    return r ? 0 : 1;
  }
  else {
    shell_fprintf(sh, SHELL_ERROR, "Invalid number of arguments\n");
    return -EINVAL;
  }

  return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_litt,
#ifdef USE_DEMAND_DRIVEN_THERMOSTAT
                               SHELL_CMD_ARG(demand, NULL,
                                             "Report the demand of a thermostat.\n"
                                             "Usage: demand <h:short_addr> <d:endpoint> <f:demand between 0.0 and 1.0>",
                                             sh_cmd_demand, 1, 2),
#endif
                               SHELL_CMD_ARG(demanders, NULL,
                                             "Show demanders\n"
                                             "Usage: demanders",
                                             sh_cmd_demanders, 1, 0),
                               SHELL_CMD_ARG(status, NULL,
                                             "Get/set (OpenTherm) master status.\n"
                                             "Usage: status [h:1 byte]",
                                             sh_cmd_status, 1, 1),
                               SHELL_CMD_ARG(mode, NULL,
                                             "Get/set mode (0=off, 1=automatic, 2=manual).\n"
                                             "Usage: mode [h:1 byte]",
                                             sh_cmd_mode, 1, 1),
                               SHELL_CMD_ARG(scheduler, NULL,
                                             "Enable/disable scheduler\n"
                                             "Usage: scheduler [on|off]",
                                             sh_cmd_scheduler, 1, 1),
                               SHELL_CMD_ARG(setpoint, NULL,
                                             "Set flow setpoint\n"
                                             "Usage: setpoint [d:temperature]",
                                             sh_cmd_setpoint, 1, 1),
                               SHELL_CMD_ARG(cease, NULL,
                                             "Cease operations\n"
                                             "Usage: cease",
                                             sh_cmd_cease, 1, 0),
                               SHELL_CMD_ARG(find, NULL,
                                             "Find thermostats\n"
                                             "Usage: find",
                                             sh_cmd_find, 1, 0),
                               SHELL_CMD_ARG(mix, NULL,
                                             "Set mixing function (0=first, 1=max, 2=average)\n"
                                             "Usage: mix [h:1 byte]",
                                             sh_cmd_set_mix, 1, 1),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(litt, &sub_litt, "litt commands", NULL);

int main(void) {
  log_init();

  LOG_INF("Starting littZ on %s", CONFIG_BOARD);
  LOG_INF("Build date: %s %s", __DATE__, __TIME__);

#if NRF_POWER_HAS_MAINREGSTATUS == 1
  nrf_power_mainregstatus_t pwr_status = nrf_power_mainregstatus_get(NRF_POWER);
  LOG_INF("Power mode: %s", (pwr_status == POWER_MAINREGSTATUS_MAINREGSTATUS_Normal) ? "normal" : "high");
#endif

  // zb_nvram_erase();

  setup_watchdog();
  setup_leds();
  setup_settings();
  start_zigbee();
  start_chip_temp_timer();
  setup_log_timestamps();

  app().run();

  return 0;
}
