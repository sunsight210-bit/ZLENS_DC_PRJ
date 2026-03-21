// App/Inc/zoom_table.hpp
#pragma once
#include <cstdint>

namespace zlens {

struct ZOOM_ENTRY_S {
    uint16_t zoom_x10;
    uint16_t angle_x100;
};

class ZoomTable {
public:
    static constexpr uint8_t MAX_ENTRIES = 32;
    static constexpr int32_t EXTERNAL_GEAR_RATIO = 4;
    static constexpr int32_t TOTAL_RANGE = 858592;   // 214648 × 4, 大齿轮满圈
    static constexpr int32_t HOME_OFFSET = 2048;      // 归零后逻辑原点偏移
    static constexpr int32_t FULL_ROTATION_X100 = 36000; // 360° × 100

    void init();
    void load_defaults();

    // Lookup
    int32_t get_position(uint16_t zoom_x10) const;
    uint16_t get_nearest_zoom(int32_t position) const;
    bool is_valid_zoom(uint16_t zoom_x10) const;
    uint16_t get_next_zoom(uint16_t current, int8_t step) const;
    uint16_t get_min_zoom() const;
    uint16_t get_max_zoom() const;

    // Factory mode
    void set_entry(uint16_t zoom_x10, uint16_t angle_x100);
    void erase_all();
    uint8_t get_entry_count() const { return m_iCount; }

    // Flash persistence
    bool save_to_flash();
    bool load_from_flash();

private:
    ZOOM_ENTRY_S m_aEntries[MAX_ENTRIES];
    uint8_t m_iCount = 0;

    int angle_to_position(uint16_t angle_x100) const;
    int find_index(uint16_t zoom_x10) const;
    void sort_entries();
};

} // namespace zlens
