// App/Src/system_manager.cpp
#include "system_manager.hpp"

namespace zlens {

void SystemManager::init() {
    m_eState = SYSTEM_STATE_E::INIT;
}

bool SystemManager::is_valid_transition(SYSTEM_STATE_E from, SYSTEM_STATE_E to) const {
    // Any state can go to ERROR
    if (to == SYSTEM_STATE_E::ERROR_STATE) return true;

    switch (from) {
    case SYSTEM_STATE_E::INIT:       return to == SYSTEM_STATE_E::SELF_TEST;
    case SYSTEM_STATE_E::SELF_TEST:  return to == SYSTEM_STATE_E::HOMING || to == SYSTEM_STATE_E::READY;
    case SYSTEM_STATE_E::HOMING:     return to == SYSTEM_STATE_E::READY;
    case SYSTEM_STATE_E::READY:      return to == SYSTEM_STATE_E::BUSY || to == SYSTEM_STATE_E::HOMING || to == SYSTEM_STATE_E::SELF_TEST;
    case SYSTEM_STATE_E::BUSY:       return to == SYSTEM_STATE_E::READY;
    case SYSTEM_STATE_E::ERROR_STATE: return to == SYSTEM_STATE_E::INIT;
    }
    return false;
}

bool SystemManager::transition_to(SYSTEM_STATE_E new_state) {
    if (!is_valid_transition(m_eState, new_state)) return false;
    m_eState = new_state;
    return true;
}

uint16_t SystemManager::get_status_code() const {
    switch (m_eState) {
    case SYSTEM_STATE_E::READY: return 0x0000; // stopped
    case SYSTEM_STATE_E::BUSY:  return 0x0001; // running
    case SYSTEM_STATE_E::ERROR_STATE: return 0x0002; // stalled (generic)
    default: return 0x0000;
    }
}

} // namespace zlens
