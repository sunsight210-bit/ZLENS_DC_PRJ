// App/Inc/encoder.hpp
#pragma once
#include <cstdint>
#ifdef BUILD_TESTING
#include "stm32f1xx_hal.h"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

class Encoder {
public:
    void init();
    int32_t get_position() const;
    void set_position(int32_t pos);
    int32_t get_z_position() const { return m_iZPosition; }
    void reset();

    // Called from ISR
    void handle_overflow(bool up_direction);
    void handle_z_pulse();

private:
    volatile int32_t m_iOverflow = 0;
    volatile int32_t m_iZPosition = 0;
};

} // namespace zlens
