// App/Inc/encoder.hpp
#pragma once
#include <cstdint>
#include "stm32f1xx_hal.h"

namespace zlens {

class Encoder {
public:
    // INDEX drift detection threshold (±4 steps ≈ ±8 µm)
    static constexpr int16_t DRIFT_THRESHOLD = 4;

    void init();
    int32_t get_position() const;
    void set_position(int32_t pos);
    int32_t get_z_position() const { return m_iZPosition; }
    void reset();

    // INDEX drift detection
    bool    is_drift_detected() const { return m_bDriftDetected; }
    void    clear_drift_flag()        { m_bDriftDetected = false; }
    int16_t get_drift_error()  const  { return m_iDriftError; }
    void    reset_index_tracking();

    // Called from ISR
    void handle_overflow(bool up_direction);
    void handle_z_pulse();

private:
    volatile int32_t m_iOverflow = 0;
    volatile int32_t m_iZPosition = 0;

    // INDEX drift tracking
    int32_t  m_iFirstIndexPos  = 0;
    uint16_t m_iIndexCount     = 0;
    bool     m_bFirstIndexSet  = false;
    bool     m_bDriftDetected  = false;
    int16_t  m_iDriftError     = 0;
};

} // namespace zlens
