/*
This script uses the SPIFFS file system(https://github.com/espressif/arduino-esp32/tree/master/libraries/SPIFFS)
and assumes that the data folder in the script folder is built as image and uploaded
to the ESP32. In the PlatformIO IDE the following esp32dev Platform project tasks are available
for that:
1. Build Filesystem Image
2. Upload Filesystem Image
*/
#include <ESP_I2S.h>                                  // arduino-core 3.x
#include <SPIFFS.h>
#include "esp32_wiring.h"
#include "mp3_decoder.h"
#include "mp3_player.h"

// MP3
#define FRAMESIZE         1152                        // Max. frame size in bytes(mp3)
#define OUTSIZE           2048                        // Max number of samples per channel(mp3)

// MP3
uint8_t*          mp3buff = nullptr;                  // Space allocated for complete MP3 file
uint8_t*          mp3bpnt;                            // Points into mp3buff
int               mp3bcnt;                            // Number of samples in buffer
bool              searchFrame;                        // True if search for startframe is needed
int16_t           outbuf[OUTSIZE*2];                  // I2S buffer
int               smpwords;                           // Number of 16 bit words for I2S
int16_t           gain = 50;                          // Gain 0..100
i2s_port_t        i2s_num = I2S_NUM_0;                // i2S port number
int               sampleRate = 44100;                 // Sample rate in Hz

i2s_data_bit_width_t bps = I2S_DATA_BIT_WIDTH_16BIT;
i2s_mode_t mode = I2S_MODE_STD;
i2s_slot_mode_t slot = I2S_SLOT_MODE_STEREO;
I2SClass i2s;


void setup_mp3() {
  esp32_wiring_setup();
  i2s.setPins(bclkI2S, lrcI2S, dinI2S);

  // start I2S at the sample rate with 16-bits per sample
  if (!i2s.begin(mode, sampleRate, bps, slot)) {
    Serial.println("Failed to initialize I2S!");
    while (1);  // do nothing
  }
}


void play_mp3(const char filename[]) {
  File           fp;                                       // MP3 input file
  size_t         flen;                                     // Length of MP3 file
  size_t         alocspace = 0;                            // Allocated buffer space
  char buffer[100];                                        // Used for formatted print

  Serial.println("\nStarting I2S playtask..");
  MP3Decoder_AllocateBuffers();                            // Init HELIX buffers
  digitalWrite(internalLED, HIGH);                         // Show activity
  fp = SPIFFS.open(filename, FILE_READ);                   // Open the MP3 file
  if(!fp) {                                                // Open successful?
    snprintf(buffer, sizeof(buffer),
             "File %s does not exist!",                    // No, show and skip request
             filename);
    Serial.println(buffer);
    return;
  }
  flen = fp.size();                                        // Get file length
  snprintf(buffer, sizeof(buffer),
            "Play file %s, length is %d bytes",            // Show file info
            filename, flen);
  Serial.println(buffer);
  if(flen > alocspace) {                                   // Need for more buffer space?
    alocspace = flen;                                      // Yes, set new required space
    mp3buff = (uint8_t*)realloc(mp3buff, alocspace);       // Allocate this space for MP3 file
  }
  if(mp3buff == nullptr) {                                 // Enough space?
    Serial.println("File error!");                         // No, report and skip
    return;
  }     
  fp.read(mp3buff, flen);                                  // Read MP3 data from file
  fp.close();                                              // Close MP3 file
  playBuff(i2s_num, flen);                                 // Play the file in the buffer
  digitalWrite(internalLED, LOW);                          // End of activity
}


void playBuff(i2s_port_t i2s_num, size_t len)
{
  int      newcnt;                               // Number of bytes remaining after decoding
  uint32_t samprate;                             // Sample rate
  int      channels;                             // Number of channels
  int      smpbytes;                             // Number of bytes for I2S
  int      s;                                    // Position of syncword
  bool     once;                                 // To execute part of code once
  int      n;                                    // Number of samples decoded
  size_t   bw;                                   // I2S bytes written
  int      ops = 0;                              // Number of output samples
  char buffer[100];                              // Used for formatted print

  s = MP3FindSyncWord(mp3buff, len);             // Search for first frame, usually at pos=0
  if(s < 0)                                      // Sync found?
  {
    Serial.println("No mp3 word sync");
    return;                                      // Return with no effect
  }
  mp3bpnt = mp3buff + s;                         // Set pointer to start of frame
  len -= s;                                      // Decrement length
  once = true;                                   // Get samplerate once
  snprintf(buffer, sizeof(buffer),
           "Sync found at 0x%04X, len is %d", s, len);
  Serial.println(buffer);
  while(len > 0) {                               // More MP3 data in buffer?
    newcnt = len;                                // Remaining length of buffer
    n = MP3Decode(mp3bpnt, &newcnt,              // Decode the frame
                    outbuf, 0);
    if(n < 0) {                                  // Check if decode is okay
      snprintf(buffer, sizeof(buffer), "MP3Decode error %d", n);
      Serial.println(buffer);
      return;
    }
    if (once) {
      samprate = MP3GetSampRate();               // Get sample rate
      channels = MP3GetChannels();               // Get number of channels
      int br = MP3GetBitrate();                  // Get bit rate
      int bps = MP3GetBitsPerSample();           // Get bits per sample
      snprintf(buffer, sizeof(buffer),
          "Sample rate: %d, channels: %d, bitrate: %d, bits per sample: %d",
          samprate, channels, br, bps);
      Serial.println(buffer);
      if(channels == 1) {                        // For mono, half sample rate
        samprate /= 2;
      }
      i2s.configureTX(samprate, (i2s_data_bit_width_t)bps, slot);
      once = false;                              // No need to set samplerate again
    }
    ops = MP3GetOutputSamps();                   // Get number of output samples
    smpbytes = ops * 2;                          // Number of bytes in outbuf
    mp3bpnt += (len - newcnt);                   // Increment pointer by bytes handled
    len = newcnt;                                // Length of remaining buffer
    {
      for(int i = 0; i < ops; i++)               // Correct amplitude
      {
        outbuf[i] = outbuf[i] * gain / 100;      // Reduce 0..100 percent
      }
    }
    i2s.write((uint8_t *)outbuf, smpbytes);
  }
}
