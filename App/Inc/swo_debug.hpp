// App/Inc/swo_debug.hpp
#pragma once
#include <cstdint>

#ifdef BUILD_TESTING
#include <cstdio>
#include <cstdarg>
inline void swo_printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}
#else
void swo_printf(const char* fmt, ...);
#endif
