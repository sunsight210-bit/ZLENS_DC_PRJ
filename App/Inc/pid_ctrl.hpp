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

    int16_t compute(int32_t iError, int32_t iPos) {
        float fP = KP * static_cast<float>(iError);

        // Integral with separation
        if (std::abs(iError) <= INTEGRAL_SEPARATION) {
            m_fIntegral += KI * static_cast<float>(iError);
            if (m_fIntegral > INTEGRAL_MAX) m_fIntegral = INTEGRAL_MAX;
            if (m_fIntegral < -INTEGRAL_MAX) m_fIntegral = -INTEGRAL_MAX;
        } else {
            m_fIntegral = 0.0f;
        }

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
    }

private:
    static constexpr float MAX_OUTPUT = 1280.0f;
    float m_fIntegral = 0;
    int32_t m_iLastPos = 0;
};

} // namespace zlens
