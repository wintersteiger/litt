#ifndef _LITT_COMMANDS_H_
#define _LITT_COMMANDS_H_

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>

#include <litt/opentherm/application.h>

namespace litt {
uint16_t api_version = 1;

enum class CommandID : uint8_t {
  INVALID = 0,
  GET_VERSION = 1,
  VERSION_REPLY = 2,
  CONFIRMATION = 3,
  OPENTHERM_REQUEST = 4,
  OPENTHERM_REPLY = 5,
  SET_OPENTHERM_STATUS = 6,
  SET_FLOW_SETPOINT = 7,
  TEMPERATURE_REPORT = 8,
  SET_PID_SETPOINT = 9,
  SET_PID_KP = 10,
  SET_PID_KI = 11,
  SET_PID_KD = 12,
  SET_PID_CUM_ERR = 13,
  SET_PID_WEIGHT = 14,
  SET_PID_AUTOMATIC = 15,
  LOAD_CONFIG = 16,
  SAVE_CONFIG = 17,
  DEMAND = 18,
  SET_THERMOSTAT_MODE = 19,
  SET_MIXING_FUNCTION = 20
};

#pragma pack(push, 1)
class CommandFrame {
public:
  CommandFrame(CommandID cid, uint16_t user_data, uint32_t payload)
      : command_id(cid), user_data(user_data), payload(payload) {}

  CommandFrame(CommandID cid, uint16_t user_data, float f) : command_id(cid), user_data(user_data) {
    payload = OpenTherm::from_f88(f);
  }

  CommandFrame(CommandID cid, uint16_t user_data, uint8_t user_id, float f) : command_id(cid), user_data(user_data) {
    payload = (user_id << 16) | OpenTherm::from_f88(f);
  }

  CommandFrame(CommandID cid, uint16_t user_data) : command_id(cid), user_data(user_data), payload(0) {}

  CommandFrame(const char *buffer, size_t size) {
    uint16_t cid, ud; // Some toolchains don't like/support SCNx8
    uint32_t pl;
    if (sscanf(buffer, "%02" SCNx16 "%04" SCNx16 "%08" SCNx32, &cid, &ud, &pl) == 3) {
      command_id = static_cast<CommandID>(cid & 0x00FF);
      user_data = ud;
      payload = pl;
    } else
      command_id = CommandID::INVALID;
  }

  virtual ~CommandFrame() = default;

  CommandID command_id = CommandID::OPENTHERM_REQUEST;
  uint16_t user_data = 0;
  uint32_t payload = 0;

  int to_string(char *buffer, size_t size) const {
    return snprintf(buffer, size, "%02" PRIx8 "%04" PRIx16 "%08" PRIx32, static_cast<uint8_t>(command_id), user_data,
                    payload);
  }

  static constexpr const size_t serialized_size() { return 14 + 1; }
};
#pragma pack(pop)

} // namespace litt

#endif // _LITT_COMMANDS_H_
