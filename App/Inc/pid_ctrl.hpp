#pragma once
#include <cstdint>
#include <cmath>

namespace zlens {

class PidCtrl {
public:
    static constexpr float KP = 0.50f;
    static constexpr float KI = 0.003f;
    static constexpr float KD = 0.00f;
    static constexpr float INTEGRAL_MAX = 192.0f;
    static constexpr int32_t INTEGRAL_SEPARATION = 32;
    // Ki escalation multipliers (used when motor is stuck)
    static constexpr float KI_MULT_TIER1 = 3.0f;   // 500ms stuck
    static constexpr float KI_MULT_TIER2 = 6.0f;    // 1000ms stuck
    static constexpr uint16_t KI_STUCK_TIER1 = 500;
    static constexpr uint16_t KI_STUCK_TIER2 = 1000;

    int16_t compute(int32_t iError, int32_t iPos, uint16_t iNoMoveCount = 0) {
        float fP = KP * static_cast<float>(iError);

        // Ki escalation: boost when motor is stuck
        float fKi = KI;
        if      (iNoMoveCount >= KI_STUCK_TIER2) fKi = KI * KI_MULT_TIER2;
        else if (iNoMoveCount >= KI_STUCK_TIER1) fKi = KI * KI_MULT_TIER1;

        // Integral with separation and freeze
        if (!m_bIntegralFrozen) {
            if (std::abs(iError) <= INTEGRAL_SEPARATION) {
                m_fIntegral += fKi * static_cast<float>(iError);
                if (m_fIntegral > INTEGRAL_MAX) m_fIntegral = INTEGRAL_MAX;
                if (m_fIntegral < -INTEGRAL_MAX) m_fIntegral = -INTEGRAL_MAX;
            } else {
                m_fIntegral = 0.0f;
            }
        }

        // Freeze trigger: motor just recovered from being stuck
        if (iNoMoveCount == 0 && m_bWasStuck) {
            m_bIntegralFrozen = true;
            m_bWasStuck = false;
        }
        if (iNoMoveCount > 0) m_bWasStuck = true;

        float fD = -KD * static_cast<float>(iPos - m_iLastPos);
        m_iLastPos = iPos;

        float fOutput = fP + m_fIntegral + fD;
        if (fOutput > MAX_OUTPUT) fOutput = MAX_OUTPUT;
        if (fOutput < -MAX_OUTPUT) fOutput = -MAX_OUTPUT;
        return static_cast<int16_t>(fOutput);
    }

    void reset(int32_t iCurrentPos = 0) {
        m_fIntegral = 0;
        m_iLastPos = iCurrentPos;
        m_bIntegralFrozen = false;
        m_bWasStuck = false;
    }

private:
    static constexpr float MAX_OUTPUT = 1280.0f;
    float m_fIntegral = 0;
    int32_t m_iLastPos = 0;
    bool m_bIntegralFrozen = false;
    bool m_bWasStuck = false;
};

} // namespace zlens
