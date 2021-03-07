#include "AudioFileSourceSD.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2SNoDAC.h"

AudioGeneratorMP3 *mp3;
AudioFileSourceSD *file;
AudioOutputI2S *out;


void haha() {

    file = new AudioFileSourceSD("000/006.mp3");
    out = new AudioOutputI2SNoDAC();
    mp3 = new AudioGeneratorMP3();
    mp3->begin(file, out);    


    // loop

  if (mp3->isRunning()) {
    if (!mp3->loop()) {
      mp3->stop(); 
      LOG("MP3 done");
    }
  }
    
}
