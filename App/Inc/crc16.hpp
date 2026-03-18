// App/Inc/crc16.hpp
#pragma once
#include <cstdint>

namespace zlens {
uint16_t crc16_modbus(const uint8_t* data, uint16_t length);
} // namespace zlens
