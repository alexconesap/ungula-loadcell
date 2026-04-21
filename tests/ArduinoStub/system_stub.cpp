// Stubs for platform-dependent functions needed by loadcell library desktop tests

#include <time/time_control.h>

namespace ungula {

    static TimeControl::ms_tick_t fakeMs = 0;
    static TimeControl::us_tick_t fakeUs = 0;

    void TimeControl::delayMs(time_ms_t ms) {
        const time_ms_t advance = (ms > 0) ? ms : 1;
        fakeMs += advance;
        fakeUs += advance * 1000UL;
    }
    void TimeControl::delayUs(time_us_t us) {
        fakeUs += us;
        if (us >= 1000) {
            fakeMs += us / 1000UL;
        }
    }
    TimeControl::ms_tick_t TimeControl::millis() {
        return fakeMs;
    }
    TimeControl::us_tick_t TimeControl::micros() {
        return fakeUs;
    }
    void TimeControl::delayUntilMs(ms_tick_t& ref, time_ms_t periodMs) {
        ref += periodMs;
        fakeMs = ref;
    }
    void TimeControl::delayUntilUs(us_tick_t& ref, time_us_t periodUs) {
        ref += periodUs;
        fakeUs = ref;
    }
    bool TimeControl::hasReachedMs(ms_tick_t now, ms_tick_t target) {
        return now >= target;
    }
    bool TimeControl::hasReachedUs(us_tick_t now, us_tick_t target) {
        return now >= target;
    }

}  // namespace ungula
