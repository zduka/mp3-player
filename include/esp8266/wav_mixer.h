#pragma once

#include <AudioFileSourceSD.h>

#include "helpers.h"

/** A very primitive SD card wav file reader that also applies an overlay sound from a different file. 
 
    Does not really care about 
 */
class WavMixer : public AudioFileSourceSD {
public:
    WavMixer() = default;

    ~WavMixer() override {
        if (overlay_)
            overlay_.close();
    }

    void openOverlay(char const * filename) {
        if (overlay_)
            overlay_.close();
        overlay_ = SD.open(filename, FILE_READ);
        overlay_.seek(WAV_HEADER_SIZE);
    }

    /** Reads the source and adds the overlay. 
     
        The overlay is only added *after the header and is recycled if shorter than the actual file
     */
    uint32_t read(void * data, uint32_t len) override {
        uint32_t pos = getPos();
        uint32_t result = AudioFileSourceSD::read(data, len);
        if (overlay_ && pos >= WAV_HEADER_SIZE)
            applyOverlay(pointer_cast<uint8_t*>(data), len);
        return result;
    }

    bool close() override {
        if (overlay_)
            overlay_.close();
        return AudioFileSourceSD::close();
    }

private:

    static constexpr uint32_t WAV_HEADER_SIZE = 44;

    /** Applies the overlay to the data. 
     */
    void applyOverlay(uint8_t * data, uint32_t len) {
        while (true) {
            uint32_t avail = std::min(len, overlay_.size() - overlay_.position());
            for (uint32_t i = 0; i < avail; ++i) {
                *data = *data / 2 + (overlay_.read() / 2);
                ++data;
            }
            len -= avail;
            if (len == 0)
                break;
            overlay_.seek(WAV_HEADER_SIZE);
        }
    }

    File overlay_;
}; 