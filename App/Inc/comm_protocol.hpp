// App/Inc/comm_protocol.hpp
#pragma once
#include <cstdint>

namespace zlens {

struct WorkFrameResult {
    bool valid;
    uint8_t cmd;
    uint16_t param;
};

struct FactoryFrameResult {
    bool valid;
    uint8_t cmd;
    uint16_t param_high;
    uint16_t param_low;
};

class CommProtocol {
public:
    void init();

    // Work mode (6-byte frames)
    WorkFrameResult parse_work_frame(const uint8_t* data, uint16_t len);
    void build_work_frame(uint8_t cmd, uint16_t param, uint8_t* out);

    // Factory mode (8-byte frames)
    FactoryFrameResult parse_factory_frame(const uint8_t* data, uint16_t len);
    void build_factory_frame(uint8_t cmd, uint16_t param_h, uint16_t param_l, uint8_t* out);

    // Mode management
    bool is_factory_mode() const { return m_bFactoryMode; }
    void set_factory_mode(bool enabled) { m_bFactoryMode = enabled; }

    static constexpr uint8_t FRAME_HEADER = 0xA5;
    static constexpr uint8_t WORK_FRAME_SIZE = 6;
    static constexpr uint8_t FACTORY_FRAME_SIZE = 8;

private:
    bool m_bFactoryMode = false;
};

} // namespace zlens
