#include <ctime>

#include "hydrv_clock.hpp"

extern "C" int _gettimeofday(struct timeval *tv, [[maybe_unused]] void *tz)
{
    const uint32_t time_ms = hydrv::clock::Clock::GetSystemTime();

    tv->tv_sec = static_cast<time_t>(time_ms / 1000U);
    tv->tv_usec = static_cast<long>((time_ms % 1000U) * 1000U);

    return 0;
}
