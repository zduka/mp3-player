#pragma once

#include "config.h"
#include "state.h"

namespace msg {

    int constexpr COUNTER_OFFSET = __COUNTER__ + 1;

    template<typename T>
    class Message {
    public:
        uint8_t const id;
        Message():
            id{T::Id} {
        }

    } __attribute__((packed)); // msg::Command

#define MESSAGE(NAME, ...) \
    class NAME : public Message<NAME> { \
    public: \
        static uint8_t constexpr Id = __COUNTER__ - COUNTER_OFFSET; \
        __VA_ARGS__ \
    } __attribute__((packed))

    MESSAGE(SetExtendedState,
        uint8_t offset;
        uint8_t size;

        SetExtendedState(uint8_t offset, uint8_t size):
            offset{offset},
            size{size} {
        }
    );

    MESSAGE(PowerOff);

    MESSAGE(SetMode,
        Mode mode;
        SetMode(Mode mode):
            mode{mode} {
        }
    );

    MESSAGE(SetControlRange,
        uint16_t value;
        uint16_t max;
        SetControlRange(uint16_t value, uint16_t max):
            value{value},
            max{max} {
        }
    );

    MESSAGE(SetVolumeRange,
        uint8_t value;
        uint8_t max;
        SetVolumeRange(uint8_t value, uint8_t max):
            value{value},
            max{max} {
        }
    );


    /** \name Lights Control
     */
    //@{
    MESSAGE(LightsFill, 
        Color color;
        uint8_t timeout;

        LightsFill(Color color, uint8_t timeout = SPECIAL_LIGHTS_TIMEOUT):
            color{color},
            timeout{timeout} {
        }
    );

    MESSAGE(LightsPoint, 
        Color color;
        uint16_t value;
        uint16_t max;
        uint8_t timeout;

        LightsPoint(uint16_t value, uint16_t max, Color const & color, uint8_t timeout = SPECIAL_LIGHTS_TIMEOUT):
            color{color},
            value{value},
            max{max},
            timeout{timeout} {
        }
    );

    MESSAGE(LightsBar,
        Color color;
        uint16_t value;
        uint16_t max;
        uint8_t timeout;

        LightsBar(uint16_t value, uint16_t max, Color const & color, uint8_t timeout = SPECIAL_LIGHTS_TIMEOUT):
            color{color},
            value{value},
            max{max},
            timeout{timeout} {
        }
    );

    MESSAGE(LightsBarCentered,
        Color color;
        uint16_t value;
        uint16_t max;
        uint8_t timeout;

        LightsBarCentered(uint16_t value, uint16_t max, Color const & color, uint8_t timeout = SPECIAL_LIGHTS_TIMEOUT):
            color{color},
            value{value},
            max{max},
            timeout{timeout} {
        }
    );

    MESSAGE(LightsColors,
        Color colors[8];
        uint8_t timeout;

        LightsColors(Color l0, Color l1, Color l2, Color l3, Color l4, Color l5, Color l6, Color l7, uint8_t timeout = SPECIAL_LIGHTS_TIMEOUT):
            colors{l0, l1, l2, l3, l4, l5, l6, l7},
            timeout{timeout} { 
        }
    );
    //@}

    /** \name Audio recording & monitoring
     */
    //@{
    MESSAGE(StartRecording);

    MESSAGE(StopRecording);
    //@}



#undef MESSAGE
 

    static_assert(SetExtendedState::Id == 0);

} // namespace msg