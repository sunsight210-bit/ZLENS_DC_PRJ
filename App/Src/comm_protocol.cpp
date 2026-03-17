// App/Src/comm_protocol.cpp
#include "comm_protocol.hpp"
#include "crc16.hpp"

namespace zlens {

void CommProtocol::init() {
    m_bFactoryMode = false;
}

WorkFrameResult CommProtocol::parse_work_frame(const uint8_t* data, uint16_t len) {
    WorkFrameResult r{false, 0, 0};
    if (len < WORK_FRAME_SIZE) return r;
    if (data[0] != FRAME_HEADER) return r;

    uint16_t crc_calc = crc16_modbus(data, 4);
    uint16_t crc_recv = data[4] | (data[5] << 8);
    if (crc_calc != crc_recv) return r;

    r.valid = true;
    r.cmd = data[1];
    r.param = (data[2] << 8) | data[3];
    return r;
}

void CommProtocol::build_work_frame(uint8_t cmd, uint16_t param, uint8_t* out) {
    out[0] = FRAME_HEADER;
    out[1] = cmd;
    out[2] = (param >> 8) & 0xFF;
    out[3] = param & 0xFF;
    uint16_t crc = crc16_modbus(out, 4);
    out[4] = crc & 0xFF;
    out[5] = (crc >> 8) & 0xFF;
}

FactoryFrameResult CommProtocol::parse_factory_frame(const uint8_t* data, uint16_t len) {
    FactoryFrameResult r{false, 0, 0, 0};
    if (len < FACTORY_FRAME_SIZE) return r;
    if (data[0] != FRAME_HEADER) return r;

    uint16_t crc_calc = crc16_modbus(data, 6);
    uint16_t crc_recv = data[6] | (data[7] << 8);
    if (crc_calc != crc_recv) return r;

    r.valid = true;
    r.cmd = data[1];
    r.param_high = (data[2] << 8) | data[3];
    r.param_low  = (data[4] << 8) | data[5];
    return r;
}

void CommProtocol::build_factory_frame(uint8_t cmd, uint16_t param_h, uint16_t param_l, uint8_t* out) {
    out[0] = FRAME_HEADER;
    out[1] = cmd;
    out[2] = (param_h >> 8) & 0xFF;
    out[3] = param_h & 0xFF;
    out[4] = (param_l >> 8) & 0xFF;
    out[5] = param_l & 0xFF;
    uint16_t crc = crc16_modbus(out, 6);
    out[6] = crc & 0xFF;
    out[7] = (crc >> 8) & 0xFF;
}

} // namespace zlens
