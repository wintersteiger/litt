// CM Wintersteiger, 2022

#include <zboss_api.h>

#include "zb_zcl_opentherm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(opentherm, LOG_LEVEL_DBG);

static zb_uint8_t gs_opentherm_commands[] = { ZB_ZCL_CLUSTER_ID_OPENTHERM_SERVER_ROLE_RECEIVED_CMD_LIST };

static zb_discover_cmd_list_t gs_opentherm_server_cmd_list =
{
  sizeof(gs_opentherm_commands), gs_opentherm_commands,
  0, NULL
};

static zb_ret_t zb_zcl_opentherm_check_value(zb_uint16_t attr_id, zb_uint8_t endpoint, zb_uint8_t *value)
{
  ZVUNUSED(attr_id);
  ZVUNUSED(value);
  ZVUNUSED(endpoint);
  return RET_OK;
}

static void zb_zcl_opentherm_write_attr_hook(zb_uint8_t endpoint, zb_uint16_t attr_id, zb_uint8_t *new_value, zb_uint16_t manuf_code)
{
  ZVUNUSED(endpoint);
  ZVUNUSED(attr_id);
  ZVUNUSED(new_value);
}

zb_bool_t zb_zcl_opentherm_handler(zb_uint8_t param)
{
  zb_bool_t processed = ZB_TRUE;
  zb_zcl_parsed_hdr_t cmd_info;
  zb_ret_t status = RET_OK;

  if (ZB_ZCL_GENERAL_GET_CMD_LISTS_PARAM == param)
  {
    ZCL_CTX().zb_zcl_cluster_cmd_list = &gs_opentherm_server_cmd_list;
    return ZB_TRUE;
  }

  ZB_ZCL_COPY_PARSED_HEADER(param, &cmd_info);

  TRACE_MSG(TRACE_ZCL1, "> zb_zcl_opentherm_handler: param %hd, cmd %hd", (FMT__H_H, param, cmd_info.cmd_id));

  switch (cmd_info.cmd_id)
  {
    case ZB_ZCL_CMD_OPENTHERM_REQUEST:
      // Do we have to do something here?
      status = RET_BUSY;
      break;

    default:
      processed = ZB_FALSE;
      break;
  }

  if (processed)
  {
    if (cmd_info.disable_default_response && status == RET_OK)
    {
      TRACE_MSG( TRACE_ZCL3, "Default response disabled", (FMT__0));
      zb_buf_free(param);
    }
    else if (status != RET_BUSY)
    {
      ZB_ZCL_COPY_PARSED_HEADER(param, &cmd_info);

      ZB_ZCL_SEND_DEFAULT_RESP( param,
          ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).source.u.short_addr,
          ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
          ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).src_endpoint,
          ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).dst_endpoint,
          cmd_info.profile_id,
          ZB_ZCL_CLUSTER_ID_OPENTHERM,
          cmd_info.seq_number,
          cmd_info.cmd_id,
          status==RET_OK ? ZB_ZCL_STATUS_SUCCESS : ZB_ZCL_STATUS_INVALID_FIELD);
    }
  }

  TRACE_MSG(TRACE_ZCL1, "< zb_zcl_opentherm_handler: processed %d", (FMT__D, processed));

  return processed;
}

void zb_zcl_opentherm_init_server()
{
  zb_zcl_add_cluster_handlers(
    ZB_ZCL_CLUSTER_ID_OPENTHERM,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		zb_zcl_opentherm_check_value,
		zb_zcl_opentherm_write_attr_hook,
		zb_zcl_opentherm_handler);
}

void zb_zcl_opentherm_init_client()
{
  zb_zcl_add_cluster_handlers(
   ZB_ZCL_CLUSTER_ID_OPENTHERM,
		ZB_ZCL_CLUSTER_CLIENT_ROLE,
		zb_zcl_opentherm_check_value,
		zb_zcl_opentherm_write_attr_hook,
		zb_zcl_opentherm_handler);
}

void zb_zcl_opentherm_send_request_response(const zb_zcl_addr_t *addr, zb_uint8_t dst_ep, zb_uint8_t ep, zb_uint16_t profile_id, zb_uint16_t mfid, zb_uint32_t msg)
{
  zb_bufid_t buffer = zb_buf_get_out();
  ZB_ZCL_OPENTHERM_SEND_REQUEST_RESPONSE(buffer, ZB_ZCL_GET_SEQ_NUM(),
    addr->u, addr->addr_type, dst_ep,
    ep, profile_id,
    NULL,
    mfid,
    &msg);
}
