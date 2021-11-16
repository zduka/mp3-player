#pragma once

#include <time.h>

/** Date & time down to a second in 4 bytes. 
 */
class DateTime {
public:

    DateTime() {
        setMonth(1);
        setDay(1);
    }

    uint16_t year() const { // 2021..2084 => 6
        return ((raw_ & YEAR_MASK) >> 26) + 2021;
    }

    uint8_t month() const { // 1..12 => 4
        return (raw_ & MONTH_MASK) >> 22;
    }

    uint8_t day() const { // 1..31 => 5
        return (raw_ & DAY_MASK) >> 17;
    }

    uint8_t hour() const { // 0..23 => 5
        return (raw_ & HOUR_MASK) >> 12;
    }

    uint8_t minute() const { // 0..59 => 6
        return (raw_ & MINUTE_MASK) >> 6;
    }

    uint8_t second() const { // 0..59 => 6
        return raw_ & SECOND_MASK;
    }

    /** Returns the day of week for given date. 
     
        Only works for dates in range. Returns 0 for Monday and 6 for Sunday. 
     */
    // https://cs.uwaterloo.ca/~alopez-o/math-faq/node73.html
    uint8_t dayOfWeek() const {
        uint8_t m = month();
        uint8_t d = day();
        uint16_t y = year();
        y -= m<3; 
        return (y + y / 4 - y / 100 +y / 400 + "-bed=pen+mad."[m] + d) % 7;
    }

    void setYear(uint16_t year) {
        //assert(year >= 2021 && year <= 2084);
        raw_ &= ~YEAR_MASK;
        raw_ |= static_cast<uint32_t>(year - 2021) << 26;
    }

    void setMonth(uint8_t month) {
        //assert(month >= 1 && month <= 12);
        raw_ &= ~MONTH_MASK;
        raw_ |= static_cast<uint32_t>(month) << 22;
    }

    void setDay(uint8_t day) {
        //assert(day >= 1 && day <= 31);
        raw_ &= ~DAY_MASK;
        raw_ |= static_cast<uint32_t>(day) << 17;
    }

    void setHour(uint8_t hour) {
        //assert(hour >= 0 && hour <= 23);
        raw_ &= ~HOUR_MASK;
        raw_ |= static_cast<uint32_t>(hour) << 12;
    }

    void setMinute(uint8_t m) {
        //assert(m >= 0 && m <= 59);
        raw_ &= ~MINUTE_MASK;
        raw_ |= static_cast<uint32_t>(m) << 6;
    }

    void setSecond(uint8_t s) {
        //assert(s >= 0 && s <= 59);
        raw_ &= ~SECOND_MASK;
        raw_ |= static_cast<uint32_t>(s);
    }

    void secondTick() {
        if (second() == 59) {
            setSecond(0);
            if (minute() == 59) {
                setMinute(0);
                if (hour() == 23) {
                    setHour(0);
                    if (day() == DaysInMonth(year(), month())) {
                        setDay(1);
                        if (month() == 12) {
                            setMonth(0);
                            // years can overflow
                            setYear(year() == 2084 ? 2021 : (year() + 1));
                        } else {
                            setMonth(month() + 1);
                        }
                    } else {
                        setDay(day() + 1);
                    }
                } else {
                    setHour(hour() + 1);
                }
            } else {
                setMinute(minute() + 1);
            }
        } else {
            setSecond(second() + 1);
        } 
    }

    bool timeEqualTo(DateTime const & other) const {
        return (raw_ & (HOUR_MASK | MINUTE_MASK | SECOND_MASK)) == (other.raw_ & (HOUR_MASK | MINUTE_MASK | SECOND_MASK));
    }

    static uint8_t DaysInMonth(uint16_t year, uint8_t month) {
        switch (month) {
            case 1: // Jan
            case 3: // Mar
            case 5: // May
            case 7: // Jul
            case 8: // Aug
            case 10: // Oct
            case 12: // Dec
                return 31;
            case 2 : // Feb
                // I'm ignoring the every 100 years leap year skip as the code will hopefully not be around for that long:)
                return (year % 4 == 0) ? 29 : 28;
            case 4:
            case 6:
            case 9:
            case 11:
            default: // whatever
                return 30;
        }
    }

    void setFromNTP(time_t const & epoch) {
        tm t;
        gmtime_r(&epoch, & t);
        setSecond(t.tm_sec);
        setMinute(t.tm_min);
        setHour(t.tm_hour);
        setDay(t.tm_mday);
        setMonth(t.tm_mon + 1);
        setYear(t.tm_year + 1900);
    }

#ifdef LOG
    void log() {
        LOG("%u:%u:%u %u/%u/%u", hour(), minute(), second(), day(), month(), year());
    }
#endif

private:

    static constexpr uint32_t YEAR_MASK = UINT32_C(63) << 26;
    static constexpr uint32_t MONTH_MASK = UINT32_C(15) << 22;
    static constexpr uint32_t DAY_MASK = UINT32_C(31) << 17;
    static constexpr uint32_t HOUR_MASK = UINT32_C(31) << 12;
    static constexpr uint32_t MINUTE_MASK = UINT32_C(63) << 6;
    static constexpr uint32_t SECOND_MASK = UINT32_C(63);

    uint32_t raw_ = 0;

} __attribute__((packed)); //DateTime

static_assert(sizeof(DateTime) == 4);

/** Alarm definition. 
 */
class Alarm {
public:
    uint8_t hour() const {
        return h_ & HOUR_MASK;
    }

    Alarm & setHour(uint8_t value) {
        h_ = (h_ & ~HOUR_MASK) | (value & HOUR_MASK);
        return *this;
    }

    uint8_t minute() const {
        return m_ & MINUTE_MASK;
    }

    Alarm & setMinute(uint8_t value) {
        m_ = (m_ & ~MINUTE_MASK) | (value & MINUTE_MASK);
        return *this;
    }

    void snooze(uint8_t by = 5) {
        ctrl_ = 0xff; // enable temporarily all days
        if (minute() + 5 >= 60) {
            setHour((hour() + 1) % 24);
            setMinute((minute() + 5) % 60);
        } else {
            setMinute(minute() + 5);
        }
    }

    bool enabled() const {
        return ctrl_ & ENABLED_MASK;
    }

    /** Returns true if the alarm is active for given day (0 = Monday, 6 = Sunday). 
     */
    bool activeDay(uint8_t dayId) const {
        return ctrl_ & (2 << dayId);
    }

    Alarm & enable(bool value) {
        if (value) 
            ctrl_ |= ENABLED_MASK;
        else
            ctrl_ &= ~ENABLED_MASK;
        return *this;
    }

    Alarm & enable(bool enabled, bool mon, bool tue, bool wed, bool thu, bool fri, bool sat, bool sun) {
        ctrl_ = (enabled ? ENABLED_MASK : 0)
              | (mon ? MON_MASK : 0)
              | (tue ? TUE_MASK : 0)
              | (wed ? WED_MASK : 0)
              | (thu ? THU_MASK : 0)
              | (fri ? FRI_MASK : 0)
              | (sat ? SAT_MASK : 0)
              | (sun ? SUN_MASK : 0);
        return *this;
    }

    bool operator == (DateTime const & other) const {
        if (enabled() && activeDay(other.dayOfWeek()))
            return (other.hour() == hour()) && (other.minute() == minute()) && (other.second() == 0);
        else
            return false;
    }

private:
    static constexpr uint8_t HOUR_MASK = 31;
    
    uint8_t h_; 

    static constexpr uint8_t MINUTE_MASK = 63;
    uint8_t m_; 

    static constexpr uint8_t ENABLED_MASK = 1;
    static constexpr uint8_t MON_MASK = 2;
    static constexpr uint8_t TUE_MASK = 4;
    static constexpr uint8_t WED_MASK = 8;
    static constexpr uint8_t THU_MASK = 16;
    static constexpr uint8_t FRI_MASK = 32;
    static constexpr uint8_t SAT_MASK = 64;
    static constexpr uint8_t SUN_MASK = 128;
    uint8_t ctrl_;

} __attribute__((packed)); // Alarm
