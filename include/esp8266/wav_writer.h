#include <SD.h>

#include "helpers.h"

/** A simple class that writes 8000hz mono PCM WAV file to an SD card. 
 
    WAV header description from here: https://www.fatalerrors.org/a/detailed-explanation-of-wav-file-format.html
 */
class WavWriter {

public:
    /** Opens given file on the SD card and prepares it for writing the pcm data. 
     
        Technically writes the dummy header. 
     */
    bool begin(char const * filename) {
        // delete the file if it exists to always start fresh
        SD.remove(filename);
        f_ = SD.open(filename, FILE_WRITE);
        if (f_) {
            f_.write(pointer_cast<uint8_t const*>(& Header_), sizeof(Header_));
            min_ = 0xff;
            max_ = 0;
            d_ = 0; // TODO or start at delay?
            return true;
        } else {
            return false;
        }
    }

    /** Patches the dummy header with proper sizes and closes the file. 
     */
    void end() {
        uint32_t size = f_.size();
        uint32_t chunkSize = size - 8;
        f_.seek(4);
        f_.write(static_cast<uint8_t>(chunkSize & 0xff));
        f_.write(static_cast<uint8_t>((chunkSize >> 8) & 0xff));
        f_.write(static_cast<uint8_t>((chunkSize >> 16) & 0xff));
        f_.write(static_cast<uint8_t>((chunkSize >> 24) & 0xff));
        uint32_t dataSize = size - sizeof(Header_);
        f_.seek(40);
        f_.write(static_cast<uint8_t>(dataSize & 0xff));
        f_.write(static_cast<uint8_t>((dataSize >> 8) & 0xff));
        f_.write(static_cast<uint8_t>((dataSize >> 16) & 0xff));
        f_.write(static_cast<uint8_t>((dataSize >> 24) & 0xff));
        f_.close();
    }

    /** Adds given pcm byte to the file. 
     */
    void add(uint8_t pcm) {
        if (min_ > pcm)
            min_ = pcm;
        if (max_ < pcm)
            max_ = pcm;
        if (pcm >= (CENTER + NOISE_FILTER_THRESHOLD) || pcm <= (CENTER - NOISE_FILTER_THRESHOLD)) 
            d_ = NOISE_FILTER_DELAY;
        else if (d_ == 0 || --d_ == 0)
            pcm = CENTER;
        f_.write(pcm);
    }

    uint8_t min() const {
        return min_;
    }

    uint8_t max() const {
        return max_;
    }

    uint8_t amplitude() const {
        return max_ - min_;
    }

private:

    /** The center value of the recording, i.e. the sound of absolute silence. 
     */
    static constexpr uint8_t CENTER = 128;

    /** The minimal amplitude (distance between center and signal) to trigger actual recording. 
     */
    static constexpr uint8_t NOISE_FILTER_THRESHOLD = 5;

    static constexpr uint8_t NOISE_FILTER_DELAY = 20;

    static inline uint8_t const Header_[] PROGMEM = { 
        0x52, 0x49, 0x46, 0x46, // RIFF
        0xca, 0xfe, 0xba, 0xbe, // chunk size
        0x57, 0x41, 0x56, 0x45, // format (WAVE)
        0x66, 0x6d, 0x74, 0x20, // subchunk 1 ID (fmt_)
        0x10, 0x00, 0x00, 0x00, // subchunk size, 16
        0x01, 0x00, // audio format, uncompressed PCM
        0x01, 0x00, // mono (number of channels)
        0x40, 0x1f, 0x00, 0x00, // sample rate (8000)
        0x40, 0x1f, 0x00, 0x00, // byte rate (8000)
        0x01, 0x00, // block align (number of bytes per sample) (1)
        0x08, 0x00, // bits per sample
        0x64, 0x61, 0x74, 0x61, // subchunk 2 ID (data)
        0xca, 0xfe, 0xba, 0xbe // subchunk 2 size 
    };

    File f_;
    uint8_t min_;
    uint8_t max_;
    uint8_t d_;

}; // WawWriter