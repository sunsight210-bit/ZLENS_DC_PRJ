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
struct FRAM_PARAMS_S {
    uint16_t magic_number;       // 0x5A3C
    uint8_t  version;
    int32_t  current_position;
    uint16_t current_zoom_x10;
    int32_t  min_position;
    int32_t  max_position;
    int32_t  total_range;
    uint16_t soft_limit_offset;
    uint8_t  zero_reference;
    int32_t  z_offset;
    uint8_t  lens_type;
    uint16_t min_zoom_x10;
    uint16_t max_zoom_x10;
    uint16_t stall_count;
    uint16_t baseline_current;
    uint16_t stall_threshold;
    uint8_t  homing_done;
    uint8_t  position_valid;
    int16_t  encoder_overflow;
    uint8_t  last_save_reason;
    uint8_t  encoder_compensated;
    uint8_t  reserved[13];
    uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(FRAM_PARAMS_S) == 60, "FRAM_PARAMS_S must be 60 bytes");

class FramStorage {
public:
    static constexpr uint16_t MAGIC = 0x5A3C;
    static constexpr uint16_t PRIMARY_ADDR = 0x0000;
    static constexpr uint16_t BACKUP_ADDR  = 0x0040;

    void init(SPI_HandleTypeDef* hspi);

    bool save_params(const FRAM_PARAMS_S& params);
    bool load_params(FRAM_PARAMS_S& params);
    void emergency_save(int32_t position);
    bool is_valid();
    bool test_rw(uint16_t addr, uint8_t test_byte);

    // For testing
    void load_params_from_buffer(const FRAM_PARAMS_S& src, FRAM_PARAMS_S& dst);

    static uint16_t calc_crc(const FRAM_PARAMS_S& p);
    static bool verify_crc(const FRAM_PARAMS_S& p);
    static bool check_magic(const FRAM_PARAMS_S& p);

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
