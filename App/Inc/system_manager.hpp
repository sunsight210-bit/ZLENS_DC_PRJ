// App/Inc/system_manager.hpp
#pragma once
#include <cstdint>

namespace zlens {

enum class SYSTEM_STATE_E { INIT, SELF_TEST, HOMING, READY, BUSY, ERROR_STATE };

class SystemManager {
public:
    void init();
    bool transition_to(SYSTEM_STATE_E new_state);
    SYSTEM_STATE_E get_state() const { return m_eState; }
    bool is_ready() const { return m_eState == SYSTEM_STATE_E::READY; }
    bool is_busy() const { return m_eState == SYSTEM_STATE_E::BUSY; }
    uint16_t get_status_code() const;

private:
    SYSTEM_STATE_E m_eState = SYSTEM_STATE_E::INIT;
    bool is_valid_transition(SYSTEM_STATE_E from, SYSTEM_STATE_E to) const;
};

} // namespace zlens
