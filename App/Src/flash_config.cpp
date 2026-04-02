// App/Src/flash_config.cpp
#include "flash_config.hpp"
#include "crc16.hpp"
#include "hw_constants.hpp"
#include <cstring>

namespace zlens {

uint16_t FlashConfig::calc_crc(const FLASH_CONFIG_S& p) {
    return crc16_modbus(reinterpret_cast<const uint8_t*>(&p), sizeof(FLASH_CONFIG_S) - 2);
}

bool FlashConfig::verify_crc(const FLASH_CONFIG_S& p) {
    return calc_crc(p) == p.crc16;
}

bool FlashConfig::check_magic(const FLASH_CONFIG_S& p) {
    return p.magic == MAGIC;
}

void FlashConfig::load_defaults(FLASH_CONFIG_S& stConfig) {
    std::memset(&stConfig, 0, sizeof(stConfig));
    stConfig.magic          = MAGIC;
    stConfig.speed_duty     = 300;   // 30.0%
    stConfig.min_speed_duty =  40;   // 4.0%
    stConfig.max_speed_duty = 300;   // 30.0%
    stConfig.min_position   = 0;
    stConfig.max_position   = 65536;
    stConfig.min_zoom_x10   = 6;     // 0.6×
    stConfig.max_zoom_x10   = 70;    // 7.0×
    stConfig.zoom_step_x10  = 0;     // 不规则表
    stConfig.lens_type      = 4;
    stConfig.stall_threshold = 0;
    stConfig.crc16 = calc_crc(stConfig);
}

bool FlashConfig::load(FLASH_CONFIG_S& stConfig) {
#ifdef BUILD_TESTING
    (void)stConfig;
    return false;
#else
    const uint8_t* pFlash = reinterpret_cast<const uint8_t*>(FLASH_ADDR);
    std::memcpy(&stConfig, pFlash, sizeof(stConfig));
    return check_magic(stConfig) && verify_crc(stConfig);
#endif
}

bool FlashConfig::save(const FLASH_CONFIG_S& stConfig) {
#ifdef BUILD_TESTING
    (void)stConfig;
    return true;
#else
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_ADDR;
    erase.NbPages = 1;
    uint32_t page_err;
    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    // Write as half-words
    FLASH_CONFIG_S p = stConfig;
    p.magic = MAGIC;
    p.crc16 = calc_crc(p);

    const uint16_t* pData = reinterpret_cast<const uint16_t*>(&p);
    uint32_t addr = FLASH_ADDR;
    for (uint32_t i = 0; i < sizeof(p) / 2; ++i) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, pData[i]);
        addr += 2;
    }

    HAL_FLASH_Lock();
    return true;
#endif
}

} // namespace zlens
