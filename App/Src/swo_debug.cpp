// App/Src/swo_debug.cpp
#ifndef BUILD_TESTING

#include "swo_debug.hpp"
#include <cstdarg>
#include <cstdio>
#include <cstdint>

// ITM Stimulus Port 0
#define ITM_STIM0  (*(volatile uint32_t*)0xE0000000U)
#define ITM_TER    (*(volatile uint32_t*)0xE0000E00U)
// Debug Halting Control and Status Register
#define DHCSR      (*(volatile uint32_t*)0xE000EDF0U)
#define DHCSR_C_DEBUGEN  (1UL)

static void itm_send_char(char c) {
    if ((ITM_TER & 1U) == 0) return;
    while ((ITM_STIM0 & 1U) == 0) {}
    ITM_STIM0 = static_cast<uint32_t>(c);
}

void swo_printf(const char* fmt, ...) {
    // No debugger → skip entirely (production: zero overhead after this check)
    if ((DHCSR & DHCSR_C_DEBUGEN) == 0) return;

    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    for (int i = 0; i < len && i < static_cast<int>(sizeof(buf)); ++i) {
        itm_send_char(buf[i]);
    }
}

#endif // BUILD_TESTING
