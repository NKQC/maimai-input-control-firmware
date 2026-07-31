#pragma once

#include <cstdint>

// id 由字典约定，0x00 保留。
constexpr uint8_t BUS_MSG_LED_SET = 0x01u;
constexpr uint8_t BUS_MSG_LED_STATE = 0x02u;
