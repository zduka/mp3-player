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

    /** Sets the settings bits that from ESP's configuration that AVR needs to. 
     
        Mainly the brightness settings & mode support. 
     */
    MESSAGE(SetSettings,
        uint8_t maxBrightness;
        bool radioEnabled;
        bool discoEnabled;
        bool lightsEnabled;
        uint8_t syncHour;
        SetSettings(uint8_t maxBrightness, bool radioEnabled, bool discoEnabled, bool lightsEnabled, uint8_t syncHour):
            maxBrightness{maxBrightness},
            radioEnabled{radioEnabled},
            discoEnabled{discoEnabled},
            lightsEnabled{lightsEnabled},
            syncHour{syncHour} {
        }
    );

    /** Tells the AVR to go to sleep. 
     
        Before going to sleep, AVR first checks whether synchronization is due and if so pute ESP into Sync mode, otherwise ESP is terminated and AVR goes to sleep immediately. 
     */
    MESSAGE(Sleep);

    /** Instructs AVR to reset itself immediately. 
     
        This translates into a full system reset. 
     */
    MESSAGE(Reset);

    /** Resets the AVR's watchdog. 
     
        Is called each time extended status VCC & temp are obtained (every second). Does not have to be called in ESPBusy and recording modes where AVR has its own timer to reset ESP if necessary. 
     */
    MESSAGE(AVRWatchdogReset);

    MESSAGE(SetMode,
        Mode mode;
        MusicMode musicMode;
        SetMode(Mode mode, MusicMode musicMode):
            mode{mode},
            musicMode{musicMode} {
        }
    );

    static_assert(sizeof(SetMode) == 3);

    /** Turns the player's idle state on/off explicitly.
     
        Also sets the poweroff timeout. 
        
     */
    MESSAGE(SetIdle,
        bool idle;
        uint8_t timeout;
        SetIdle(bool value, uint8_t timeout):
            idle{value},
            timeout{timeout} {
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

    /** Sets the volume range and value. 
     */
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

    MESSAGE(SetWiFiStatus, 
        WiFiStatus status;
        SetWiFiStatus(WiFiStatus status):
            status{status} {
        }
    );

    /** Sets the busy state of the ESP chip. 
     
        When ESP is busy, avr will ignore any user inputs and will not fire the AVR_IRQ flag giving ESP more time to process its stuff.  
     */
    MESSAGE(SetESPBusy,
        bool busy;
        SetESPBusy(bool busy):
            busy{busy} {
        }
    );

#undef MESSAGE
 

    static_assert(SetExtendedState::Id == 0);

} // namespace msg