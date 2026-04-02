// App/Src/crc16.cpp
#include "crc16.hpp"

namespace zlens {

uint16_t crc16_modbus(const uint8_t* data, uint16_t length) {
    return crc16_modbus(data, length, 0xFFFF);
}

uint16_t crc16_modbus(const uint8_t* data, uint16_t length, uint16_t init) {
    uint16_t crc = init;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

} // namespace zlens
