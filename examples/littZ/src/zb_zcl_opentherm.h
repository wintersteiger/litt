// CM Wintersteiger, 2022

#ifndef ZB_ZCL_OPENTHERM_H
#define ZB_ZCL_OPENTHERM_H

#include <zcl/zb_zcl_commands.h>
#include <zcl/zb_zcl_common.h>

#define ZB_ZCL_OPENTHERM_CLUSTER_REVISION_DEFAULT ((zb_uint16_t)0x0001u)

#ifndef ZB_ZCL_CLUSTER_ID_OPENTHERM
#define ZB_ZCL_CLUSTER_ID_OPENTHERM 0xFC00
#endif

enum zb_zcl_opentherm_info_attr_e {
  ZB_ZCL_ATTR_OPENTHERM_VERSION_ID = 0x0000,
  ZB_ZCL_ATTR_OPENTHERM_STATUS_ID = 0x0001,
  ZB_ZCL_ATTR_OPENTHERM_FLOW_SETPOINT_ID = 0x0002,
  ZB_ZCL_ATTR_OPENTHERM_FLOW_TEMPERATURE_ID = 0x0003,
  ZB_ZCL_ATTR_OPENTHERM_RETURN_TEMPERATURE_ID = 0x0004,
  ZB_ZCL_ATTR_OPENTHERM_FRAMES_DROPPED_ID = 0x0005,
  ZB_ZCL_ATTR_OPENTHERM_FRAMES_LATE_ID = 0x0006,
  ZB_ZCL_ATTR_OPENTHERM_LAST_CMD_RESULT_ID = 0xfffc,
  ZB_ZCL_ATTR_OPENTHERM_CLUSTER_REVISION = 0xfffd
};

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_OPENTHERM_VERSION_ID(data_ptr)                                              \
  {                                                                                                                    \
    ZB_ZCL_ATTR_OPENTHERM_VERSION_ID, ZB_ZCL_ATTR_TYPE_U8, ZB_ZCL_ATTR_ACCESS_READ_ONLY,                               \
        (ZB_ZCL_NON_MANUFACTURER_SPECIFIC), (void *)data_ptr                                                           \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_OPENTHERM_STATUS_ID(data_ptr)                                               \
  {                                                                                                                    \
    ZB_ZCL_ATTR_OPENTHERM_STATUS_ID, ZB_ZCL_ATTR_TYPE_U16,                                                             \
        ZB_ZCL_ATTR_ACCESS_READ_WRITE | ZB_ZCL_ATTR_ACCESS_REPORTING, (ZB_ZCL_NON_MANUFACTURER_SPECIFIC),              \
        (void *)data_ptr                                                                                               \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_OPENTHERM_FLOW_SETPOINT_ID(data_ptr)                                        \
  {                                                                                                                    \
    ZB_ZCL_ATTR_OPENTHERM_FLOW_SETPOINT_ID, ZB_ZCL_ATTR_TYPE_S16,                                                      \
        ZB_ZCL_ATTR_ACCESS_READ_WRITE | ZB_ZCL_ATTR_ACCESS_REPORTING, (ZB_ZCL_NON_MANUFACTURER_SPECIFIC),              \
        (void *)data_ptr                                                                                               \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_OPENTHERM_FLOW_TEMPERATURE_ID(data_ptr)                                     \
  {                                                                                                                    \
    ZB_ZCL_ATTR_OPENTHERM_FLOW_TEMPERATURE_ID, ZB_ZCL_ATTR_TYPE_S16,                                                   \
        ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING, (ZB_ZCL_NON_MANUFACTURER_SPECIFIC),               \
        (void *)data_ptr                                                                                               \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_OPENTHERM_RETURN_TEMPERATURE_ID(data_ptr)                                   \
  {                                                                                                                    \
    ZB_ZCL_ATTR_OPENTHERM_RETURN_TEMPERATURE_ID, ZB_ZCL_ATTR_TYPE_S16,                                                 \
        ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING, (ZB_ZCL_NON_MANUFACTURER_SPECIFIC),               \
        (void *)data_ptr                                                                                               \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_OPENTHERM_FRAMES_DROPPED_ID(data_ptr)                                       \
  {                                                                                                                    \
    ZB_ZCL_ATTR_OPENTHERM_FRAMES_DROPPED_ID, ZB_ZCL_ATTR_TYPE_U32,                                                     \
        ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING, (ZB_ZCL_NON_MANUFACTURER_SPECIFIC),               \
        (void *)data_ptr                                                                                               \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_OPENTHERM_FRAMES_LATE_ID(data_ptr)                                          \
  {                                                                                                                    \
    ZB_ZCL_ATTR_OPENTHERM_FRAMES_LATE_ID, ZB_ZCL_ATTR_TYPE_U32,                                                        \
        ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING, (ZB_ZCL_NON_MANUFACTURER_SPECIFIC),               \
        (void *)data_ptr                                                                                               \
  }

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_OPENTHERM_LAST_CMD_RESULT_ID(data_ptr)                                      \
  {                                                                                                                    \
    ZB_ZCL_ATTR_OPENTHERM_LAST_CMD_RESULT_ID, ZB_ZCL_ATTR_TYPE_U32,                                                    \
        ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING, (ZB_ZCL_NON_MANUFACTURER_SPECIFIC),               \
        (void *)data_ptr                                                                                               \
  }

#define ZB_ZCL_OPENTHERM_REPORT_ATTR_COUNT 7

typedef struct {
  zb_int16_t version;
  zb_uint16_t status;
  zb_uint32_t last_cmd_result;
  zb_int16_t flow_setpoint;
  zb_int16_t flow_temperature;
  zb_int16_t return_temperature;
  zb_uint32_t frames_dropped;
  zb_uint32_t frames_late;
} zb_zcl_opentherm_attrs_t;

#define ZB_ZCL_DECLARE_OPENTHERM_ATTRIB_LIST(attr_list, version, status, flow_setpoint, flow_temperature,              \
                                             return_temperature, frames_dropped, frames_late, last_cmd_result,         \
                                             cluster_revision)                                                         \
  ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(attr_list, ZB_ZCL_OPENTHERM)                                       \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_OPENTHERM_VERSION_ID, (version))                                                    \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_OPENTHERM_STATUS_ID, (status))                                                      \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_OPENTHERM_FLOW_SETPOINT_ID, (flow_setpoint))                                        \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_OPENTHERM_FLOW_TEMPERATURE_ID, (flow_temperature))                                  \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_OPENTHERM_RETURN_TEMPERATURE_ID, (return_temperature))                              \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_OPENTHERM_FRAMES_DROPPED_ID, (frames_dropped))                                      \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_OPENTHERM_FRAMES_LATE_ID, (frames_late))                                            \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_OPENTHERM_LAST_CMD_RESULT_ID, (last_cmd_result))                                    \
  ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

enum zb_zcl_opentherm_cmd_e {
  ZB_ZCL_CMD_OPENTHERM_REQUEST = 0x2A,
};

#define ZB_ZCL_CLUSTER_ID_OPENTHERM_SERVER_ROLE_RECEIVED_CMD_LIST ZB_ZCL_CMD_OPENTHERM_REQUEST

#define ZB_ZCL_CLUSTER_ID_OPENTHERM_CLIENT_ROLE_GENERATED_CMD_LIST ZB_ZCL_CLUSTER_ID_BASIC_SERVER_ROLE_RECEIVED_CMD_LIST

void zb_zcl_opentherm_init_server(void);
void zb_zcl_opentherm_init_client(void);

#define ZB_ZCL_CLUSTER_ID_OPENTHERM_SERVER_ROLE_INIT zb_zcl_opentherm_init_server
#define ZB_ZCL_CLUSTER_ID_OPENTHERM_CLIENT_ROLE_INIT zb_zcl_opentherm_init_client

#define ZB_ZCL_OPENTHERM_SEND_REQUEST_RESPONSE(buffer, seq, addr, dst_addr_mode, dst_ep, ep, prfl_id, cb, mfid, msg)   \
  {                                                                                                                    \
    zb_uint8_t *ptr = ZB_ZCL_START_PACKET(buffer);                                                                     \
    ZB_ZCL_CONSTRUCT_SPECIFIC_COMMAND_RESP_FRAME_CONTROL_A(ptr, ZB_ZCL_FRAME_DIRECTION_TO_CLI,                         \
                                                           ZB_ZCL_MANUFACTURER_SPECIFIC);                              \
    ZB_ZCL_CONSTRUCT_COMMAND_HEADER_EXT(ptr, (seq), ZB_ZCL_MANUFACTURER_SPECIFIC, mfid, ZB_ZCL_CMD_OPENTHERM_REQUEST); \
    ZB_ZCL_PACKET_PUT_DATA32(ptr, (msg));                                                                              \
    ZB_ZCL_FINISH_PACKET(buffer, ptr)                                                                                  \
    ZB_ZCL_SEND_COMMAND_SHORT(buffer, addr, dst_addr_mode, dst_ep, ep, prfl_id, ZB_ZCL_CLUSTER_ID_OPENTHERM, cb);      \
  }

void zb_zcl_opentherm_send_request_response(const zb_zcl_addr_t *addr, zb_uint8_t dst_ep, zb_uint8_t ep,
                                            zb_uint16_t profile_id, zb_uint16_t mfid, zb_uint32_t msg);

#endif // ZB_ZCL_OPENTHERM_H
