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

    // Calibration
    void set_total_range(int32_t range) { m_iTotalRange = range; }
    int32_t get_total_range() const { return m_iTotalRange; }

    // Flash persistence
    bool save_to_flash();
    bool load_from_flash();

private:
    ZOOM_ENTRY_S m_aEntries[MAX_ENTRIES];
    uint8_t m_iCount = 0;
    int32_t m_iTotalRange = 0;

    int angle_to_position(uint16_t angle_x100) const;
    int find_index(uint16_t zoom_x10) const;
    void sort_entries();
};

} // namespace zlens
