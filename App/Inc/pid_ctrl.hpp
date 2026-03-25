#pragma once
#include <cstdint>
#include <cmath>

namespace zlens {

class PidCtrl {
public:
    static constexpr float KP = 0.50f;
    static constexpr float KI = 0.00f;
    static constexpr float KD = 0.10f;
    static constexpr float INTEGRAL_MAX = 500.0f;

    int16_t compute(int32_t iError, int32_t iPos) {
        float fP = KP * static_cast<float>(iError);

        m_fIntegral += KI * static_cast<float>(iError);
        if (m_fIntegral > INTEGRAL_MAX) m_fIntegral = INTEGRAL_MAX;
        if (m_fIntegral < -INTEGRAL_MAX) m_fIntegral = -INTEGRAL_MAX;

        float fD = 0.0f;
        if (iError != 0) {
            fD = -KD * static_cast<float>(iPos - m_iLastPos);
        }
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
    static constexpr float MAX_OUTPUT = 1200.0f;
    float m_fIntegral = 0;
    int32_t m_iLastPos = 0;
};

} // namespace zlens
