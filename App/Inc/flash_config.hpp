// App/Inc/flash_config.hpp
#pragma once
#include <cstdint>

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

#pragma pack(push, 1)
struct FLASH_CONFIG_S {
    uint16_t magic;              // 0  有效性标识
    uint16_t speed_duty;         // 2  默认速度 duty ×10 (0~1000)
    uint16_t min_speed_duty;     // 4  最小速度 duty ×10
    uint16_t max_speed_duty;     // 6  最大速度 duty ×10
    int32_t  min_position;       // 8  编码器行程下限
    int32_t  max_position;       // 12 编码器行程上限
    uint16_t min_zoom_x10;       // 16 最小倍率 ×10
    uint16_t max_zoom_x10;       // 18 最大倍率 ×10
    uint16_t zoom_step_x10;      // 20 规则表步进 (0=不规则表)
    uint8_t  lens_type;          // 22 镜头型号编号
    uint16_t stall_threshold;    // 23 堵转电流阈值
    uint8_t  reserved[21];       // 25 预留
    uint16_t crc16;              // 46 CRC16-Modbus
};  // 48B
#pragma pack(pop)

static_assert(sizeof(FLASH_CONFIG_S) == 48, "FLASH_CONFIG_S must be 48 bytes");

class FlashConfig {
public:
    static constexpr uint16_t MAGIC = 0x5A50;
    static constexpr uint32_t FLASH_ADDR = 0x0803F000;

    bool load(FLASH_CONFIG_S& stConfig);
    bool save(const FLASH_CONFIG_S& stConfig);
    static void load_defaults(FLASH_CONFIG_S& stConfig);

    static uint16_t calc_crc(const FLASH_CONFIG_S& p);
    static bool verify_crc(const FLASH_CONFIG_S& p);
    static bool check_magic(const FLASH_CONFIG_S& p);
};

} // namespace zlens
