// App/Src/zoom_table.cpp
#include "zoom_table.hpp"
#include "hw_constants.hpp"
#include "crc16.hpp"
#include <algorithm>
#include <cstring>
#include <cstdlib>

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#include "swo_debug.hpp"
#endif

namespace zlens {

static constexpr uint32_t FLASH_ZOOM_ADDR = 0x0803F800;

// Default 14-entry table (angle x100 values)
static const ZOOM_ENTRY_S kDefaultTable[] = {
    { 6,     0}, {10,  7200}, {15, 11450}, {20, 14800},
    {25, 18150}, {30, 21100}, {35, 23550}, {40, 25750},
    {45, 27700}, {50, 29450}, {55, 31150}, {60, 32500},
    {65, 33800}, {70, 34700}
};

void ZoomTable::init() {
    m_iCount = 0;
    std::memset(m_aEntries, 0, sizeof(m_aEntries));
}

void ZoomTable::load_defaults() {
    m_iCount = sizeof(kDefaultTable) / sizeof(kDefaultTable[0]);
    std::memcpy(m_aEntries, kDefaultTable, sizeof(kDefaultTable));
}

int32_t ZoomTable::angle_to_position(int32_t iAngle_x100) const {
    return HOME_OFFSET + static_cast<int32_t>(
        static_cast<int64_t>(iAngle_x100) * TOTAL_RANGE / FULL_ROTATION_X100);
}

int ZoomTable::find_index(uint16_t zoom_x10) const {
    for (uint16_t i = 0; i < m_iCount; ++i) {
        if (m_aEntries[i].zoom_x10 == zoom_x10) return i;
    }
    return -1;
}

void ZoomTable::sort_entries() {
    std::sort(m_aEntries, m_aEntries + m_iCount,
        [](const ZOOM_ENTRY_S& a, const ZOOM_ENTRY_S& b) {
            return a.zoom_x10 < b.zoom_x10;
        });
}

int32_t ZoomTable::get_position(uint16_t zoom_x10) const {
    int idx = find_index(zoom_x10);
    if (idx < 0) return -1;
    return angle_to_position(m_aEntries[idx].angle_x100);
}

uint16_t ZoomTable::get_nearest_zoom(int32_t position) const {
    if (m_iCount == 0) return 0;
    uint16_t nearest = m_aEntries[0].zoom_x10;
    int32_t min_dist = std::abs(position - angle_to_position(m_aEntries[0].angle_x100));
    for (uint16_t i = 1; i < m_iCount; ++i) {
        int32_t dist = std::abs(position - angle_to_position(m_aEntries[i].angle_x100));
        if (dist < min_dist) {
            min_dist = dist;
            nearest = m_aEntries[i].zoom_x10;
        }
    }
    return nearest;
}

bool ZoomTable::is_valid_zoom(uint16_t zoom_x10) const {
    return find_index(zoom_x10) >= 0;
}

uint16_t ZoomTable::get_next_zoom(uint16_t current, int8_t step) const {
    int idx = find_index(current);
    if (idx < 0) return current;
    int new_idx = idx + step;
    if (new_idx < 0) new_idx = 0;
    if (new_idx >= m_iCount) new_idx = m_iCount - 1;
    return m_aEntries[new_idx].zoom_x10;
}

uint16_t ZoomTable::get_min_zoom() const {
    return m_iCount > 0 ? m_aEntries[0].zoom_x10 : 0;
}

uint16_t ZoomTable::get_max_zoom() const {
    return m_iCount > 0 ? m_aEntries[m_iCount - 1].zoom_x10 : 0;
}

void ZoomTable::set_entry(uint16_t zoom_x10, uint16_t angle_x100) {
    int idx = find_index(zoom_x10);
    if (idx >= 0) {
        m_aEntries[idx].angle_x100 = angle_x100;
    } else if (m_iCount < MAX_ENTRIES) {
        m_aEntries[m_iCount++] = {zoom_x10, angle_x100};
        sort_entries();
    }
}

void ZoomTable::erase_all() {
    m_iCount = 0;
    std::memset(m_aEntries, 0, sizeof(m_aEntries));
}

bool ZoomTable::save_to_flash() {
#ifndef BUILD_TESTING
    swo_printf("[ZOOM] save_to_flash: %u entries\n", m_iCount);
#endif
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_ZOOM_ADDR;
    erase.NbPages = 1;
    uint32_t page_err;
    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    // Write: [count(2B)] + [entries as half-words] + [CRC16]
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_ZOOM_ADDR, m_iCount);

    uint32_t addr = FLASH_ZOOM_ADDR + hw::FLASH_HALFWORD_SIZE;
    for (uint16_t i = 0; i < m_iCount; ++i) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, m_aEntries[i].zoom_x10);
        addr += hw::FLASH_HALFWORD_SIZE;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, m_aEntries[i].angle_x100);
        addr += hw::FLASH_HALFWORD_SIZE;
    }

    // CRC over count(2B) + entries
    uint16_t crc = crc16_modbus(reinterpret_cast<const uint8_t*>(&m_iCount), sizeof(m_iCount));
    for (uint16_t i = 0; i < m_iCount; ++i) {
        crc = crc16_modbus(reinterpret_cast<const uint8_t*>(&m_aEntries[i]), sizeof(ZOOM_ENTRY_S));
    }
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, crc);

    HAL_FLASH_Lock();
    return true;
}

bool ZoomTable::load_from_flash() {
#ifdef BUILD_TESTING
    auto& log = mock::get_log();
    if (log.flash_memory.size() < 2) return false;

    m_iCount = log.flash_memory[0] | (log.flash_memory[1] << 8);
    if (m_iCount > MAX_ENTRIES) { m_iCount = 0; return false; }

    uint32_t offset = hw::FLASH_HALFWORD_SIZE;
    for (uint16_t i = 0; i < m_iCount; ++i) {
        m_aEntries[i].zoom_x10 = log.flash_memory[offset] | (log.flash_memory[offset+1] << 8);
        offset += hw::FLASH_HALFWORD_SIZE;
        m_aEntries[i].angle_x100 = log.flash_memory[offset] | (log.flash_memory[offset+1] << 8);
        offset += hw::FLASH_HALFWORD_SIZE;
    }
    return true;
#else
    // Real Flash read implementation
    const uint8_t* flash_ptr = reinterpret_cast<const uint8_t*>(FLASH_ZOOM_ADDR);
    uint16_t iRawCount = flash_ptr[0] | (flash_ptr[1] << 8);
    swo_printf("[ZOOM] Flash @0x%08lX raw count=%u (0x%04X)\n",
               (unsigned long)FLASH_ZOOM_ADDR, iRawCount, iRawCount);
    m_iCount = iRawCount;
    if (m_iCount > MAX_ENTRIES) { m_iCount = 0; return false; }

    uint32_t offset = hw::FLASH_HALFWORD_SIZE;
    for (uint16_t i = 0; i < m_iCount; ++i) {
        m_aEntries[i].zoom_x10 = flash_ptr[offset] | (flash_ptr[offset+1] << 8);
        offset += hw::FLASH_HALFWORD_SIZE;
        m_aEntries[i].angle_x100 = flash_ptr[offset] | (flash_ptr[offset+1] << 8);
        offset += hw::FLASH_HALFWORD_SIZE;
    }
    return true;
#endif
}

} // namespace zlens
