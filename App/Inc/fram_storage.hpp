// App/Inc/fram_storage.hpp
#pragma once
#include <cstdint>

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

#pragma pack(push, 1)
struct FRAM_STATE_S {
    uint16_t magic;              // 0  有效性标识
    int32_t  current_position;   // 2  编码器当前位置
    uint16_t current_zoom_x10;   // 6  当前倍率 ×10
    uint8_t  homing_done;        // 8  回零完成标志
    uint8_t  position_valid;     // 9  位置有效标志
    uint16_t move_count;         // 10 累计移动次数
    uint8_t  last_save_reason;   // 12 上次保存原因
    uint8_t  reserved[17];       // 13 预留
    uint16_t crc16;              // 30 CRC16-Modbus
};  // 32B
#pragma pack(pop)

static_assert(sizeof(FRAM_STATE_S) == 32, "FRAM_STATE_S must be 32 bytes");

class FramStorage {
public:
    static constexpr uint16_t MAGIC = 0x5A50;
    static constexpr uint16_t PRIMARY_ADDR = 0x0000;
    static constexpr uint16_t BACKUP_ADDR  = 0x0020;

    void init(SPI_HandleTypeDef* hspi);

    bool save_state(const FRAM_STATE_S& stState);
    bool load_state(FRAM_STATE_S& stState);
    void emergency_save(int32_t position);
    bool is_valid();
    bool test_rw(uint16_t addr, uint8_t test_byte);

    // For testing
    void load_state_from_buffer(const FRAM_STATE_S& src, FRAM_STATE_S& dst);

    static uint16_t calc_crc(const FRAM_STATE_S& p);
    static bool verify_crc(const FRAM_STATE_S& p);
    static bool check_magic(const FRAM_STATE_S& p);

private:
    SPI_HandleTypeDef* m_pHspi = nullptr;

    void write_enable();
    void write_disable_wp();
    void enable_wp();
    void spi_write(uint16_t addr, const uint8_t* data, uint16_t len);
    void spi_read(uint16_t addr, uint8_t* data, uint16_t len);
    void cs_low();
    void cs_high();
};

} // namespace zlens
