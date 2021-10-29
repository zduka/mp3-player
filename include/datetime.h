#pragma once

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

    void setYear(uint16_t year) {
        //assert(year >= 2021 && year <= 2084);
        raw_ &= ~YEAR_MASK;
        raw_ += (year - 2021) << 26;
    }

    void setMonth(uint8_t month) {
        //assert(month >= 1 && month <= 12);
        raw_ &= ~MONTH_MASK;
        raw_ += month << 22;
    }

    void setDay(uint8_t day) {
        //assert(day >= 1 && day <= 31);
        raw_ &= ~DAY_MASK;
        raw_ += day << 17;
    }

    void setHour(uint8_t hour) {
        //assert(hour >= 0 && hour <= 23);
        raw_ &= ~HOUR_MASK;
        raw_ += hour << 12;
    }

    void setMinute(uint8_t m) {
        //assert(m >= 0 && m <= 59);
        raw_ &= ~MINUTE_MASK;
        raw_ += m << 6;
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

private:

    static constexpr uint32_t YEAR_MASK = UINT32_C(63) << 26;
    static constexpr uint32_t MONTH_MASK = UINT32_C(16) << 22;
    static constexpr uint32_t DAY_MASK = UINT32_C(31) << 17;
    static constexpr uint32_t HOUR_MASK = UINT32_C(31) << 12;
    static constexpr uint32_t MINUTE_MASK = UINT32_C(63) << 6;
    static constexpr uint32_t SECOND_MASK = UINT32_C(63);

    uint32_t raw_ = 0;

} __attribute__((packed));
