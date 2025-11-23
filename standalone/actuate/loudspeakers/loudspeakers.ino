/*
This script uses the SPIFFS file system(https://github.com/espressif/arduino-esp32/tree/master/libraries/SPIFFS)
and assumes that the data folder in the script folder is built as image and uploaded
to the ESP32. In the PlatformIO IDE the following esp32dev Platform project tasks are available
for that:
1. Build Filesystem Image
2. Upload Filesystem Image

Current status: runs on standalone esp32-dev but does not produce sound. The same setup with just the
esp32-dev and the PCM510x DAC can produce sound with the simple_tone script.
*/
#include <SPIFFS.h>
#include <driver/i2s.h>
#include "esp32_wiring.h"
#include "mp3_decoder.h"

// MP3
#define I2S_ENABLE        32                             // Enable I2S/Amp if used
#define FRAMESIZE         1152                              // Max. frame size in bytes(mp3)
#define OUTSIZE           2048                              // Max number of samples per channel(mp3)

char buffer[100];  // Used for formatted print
//                                                      // MP3 stuff
uint8_t*          mp3buff = nullptr;                    // Space allocated for complete MP3 file
uint8_t*          mp3bpnt;                              // Points into mp3buff
int               mp3bcnt;                              // Number of samples in buffer
bool              searchFrame;                          // True if search for startframe is needed
int16_t           outbuf[OUTSIZE*2];                    // I2S buffer
int               smpwords;                             // Number of 16 bit words for I2S
int16_t           gain = 50;                            // Gain 0..100
i2s_port_t     i2s_num = I2S_NUM_0;                     // i2S port number



void setup() {
  pinMode(internalLED, OUTPUT);
  Serial.begin(115200);
  Serial.println("Hello!");
  if(!SPIFFS.begin(true)) {
    Serial.println("An Error occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
  showAllFiles();
  install_i2s_driver();
}


void loop() {
  char filename1[] = "/audio/spring.mp3";
  play_mp3(filename1);
  delay(5000);
  char filename2[] = "/audio/rondje.mp3";
  play_mp3(filename2);
  delay(5000);
}


void showAllFiles() {
  File root = SPIFFS.open("/");                       // Open root directory
  File file;                                             // File in directory

  Serial.println("SPIFFS contents:");
  while(file = root.openNextFile())                    // Iterate through files
  {
    snprintf(buffer, sizeof(buffer), "%5d %s", file.size(), file.path());
    Serial.println(buffer);
  }
}


void install_i2s_driver() {
  esp_err_t      pinss_err;                                        // Result of i2s_set_pin
  i2s_config_t   i2s_config;                                       // I2S configuration

  i2s_config.mode                   =(i2s_mode_t)(I2S_MODE_MASTER | // I2S mode(5)
                                          I2S_MODE_TX);
  i2s_config.sample_rate            = 44100;
  i2s_config.bits_per_sample        = I2S_BITS_PER_SAMPLE_16BIT;    //(16)
  i2s_config.channel_format         = I2S_CHANNEL_FMT_RIGHT_LEFT;   //(0)
  #if ESP_ARDUINO_VERSION_MAJOR >= 2                                 // New version?
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;    // Yes, use new definition
  #else
    i2s_config.communication_format =(i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
  #endif

  i2s_config.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;         // High interrupt priority
  i2s_config.dma_buf_count        = 16,
  i2s_config.dma_buf_len          = 64,
  i2s_config.use_apll             = false;
  i2s_config.tx_desc_auto_clear   = false;                        // No clear tx descriptor on underflow
  i2s_config.fixed_mclk           = I2S_PIN_NO_CHANGE;            // No pin for MCLK
  //i2s_config.mclk_multiple        =(i2s_mclk_multiple_t)0;
  //i2s_config.bits_per_chan        =(i2s_bits_per_chan_t)0;
  #ifdef INTERNALDAC
    i2s_config.mode =(i2s_mode_t)(I2S_MODE_MASTER |               // Set I2S mode for internal DAC
                                   I2S_MODE_TX |                   //(4)
                                   I2S_MODE_DAC_BUILT_IN);       // Enable internal DAC(16)
    #if ESP_ARDUINO_VERSION_MAJOR < 2
      i2s_config.communication_format = I2S_COMM_FORMAT_I2S_MSB;
    #endif
  #endif

  if(i2s_driver_install(i2s_num, &i2s_config, 0, NULL) != ESP_OK) {
    Serial.println("I2S install error!");
  }
  #ifdef INTERNALDAC                                               // Use internal(8 bit) DAC?
    Serial.println("Output to internal DAC");                        // Show output device
    pinss_err = i2s_set_pin(i2s_num, NULL);                    // Yes, default pins for internal DAC
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
  #else
    i2s_pin_config_t pin_config;                                  // I2s pin config
    pin_config.bck_io_num    = bclkI2S;                           // This is BCLK pin
    pin_config.ws_io_num     = lrcI2S;                            // This is L(R)CK pin
    pin_config.data_out_num  = dinI2S;                            // This is DATA output pin
    // pin_config.bck_io_num    = 2;                           // From PCB drawing
    // pin_config.ws_io_num     = 33;                          // From PCB drawing
    // pin_config.data_out_num  = 35;                          // From PCB drawing
    pin_config.data_in_num   = I2S_PIN_NO_CHANGE;                 // No input
    #if ESP_ARDUINO_VERSION_MAJOR >= 2
      pin_config.mck_io_num    = I2S_PIN_NO_CHANGE;               // MCK not used
    #endif
    snprintf(buffer, sizeof(buffer),
             "Output to I2S, pins %d, %d and %d",                // Show pins used for output device
             pin_config.bck_io_num,                              // This is the BCK(bit clock) pin
             pin_config.ws_io_num,                               // This is L(R)CK pin
             pin_config.data_out_num);                           // This is DATA output pin
    Serial.println(buffer);
    pinss_err = i2s_set_pin(i2s_num, &pin_config);               // Set I2S pins
  #endif
  if(pinss_err != ESP_OK) {                                      // Check error condition
    Serial.println("I2S setpin error!");                         // Rport bad pins
  }
}


void play_mp3(char filename[]) {
  File           fp;                                               // MP3 input file
  size_t         flen;                                             // Length of MP3 file
  size_t         alocspace = 0;                                    // Allocated buffer space

  Serial.println("Starting I2S playtask..");
  MP3Decoder_AllocateBuffers();                                   // Init HELIX buffers
  i2s_zero_dma_buffer(i2s_num);                                // Zero the buffer
  Serial.println("Playtask started");
  digitalWrite(internalLED, HIGH);                            // Show activity
  fp = SPIFFS.open(filename, FILE_READ);                      // Open the MP3 file
  if(!fp) {                                                   // Open successful?
    snprintf(buffer, sizeof(buffer),
             "File %s does not exist!",                       // No, show and skip request
             filename);
    Serial.println(buffer);
    return;
  }
  flen = fp.size();                                            // Get file length
  snprintf(buffer, sizeof(buffer),
            "Play file %s, length is %d bytes",                // Show file info
            filename, flen);
  Serial.println(buffer);
  if(flen > alocspace) {                                    // Need for more buffer space?
    alocspace = flen;                                       // Yes, set new required space
    mp3buff =(uint8_t*)realloc(mp3buff, alocspace);         // Allocate this space for MP3 file
  }
  if(mp3buff == nullptr) {                                  // Enough space?
    Serial.println("File error!");                          // No, report and skip
    return;
  }     
  fp.read(mp3buff, flen);                                   // Read MP3 data from file
  fp.close();                                               // Close MP3 file
  i2s_start(i2s_num);                                       // Start I2S
  playBuff(i2s_num, flen);                                  // Play the file in the buffer
  i2s_stop(i2s_num);                                        // Stop I2S
  digitalWrite(internalLED, LOW);                           // End of activity
}


void playBuff(i2s_port_t i2s_num, int len)
{
  int             newcnt;                            // Number of bytes remaining after decoding
  static uint32_t samprate;                          // Sample rate
  static int      channels;                          // Number of channels
  static int      smpbytes;                          // Number of bytes for I2S
  int             s;                                 // Position of syncword
  static bool     once;                              // To execute part of code once
  int             n;                                 // Number of samples decoded
  size_t          bw;                                // I2S bytes written
  int             ops = 0;                           // Number of output samples

  s = MP3FindSyncWord(mp3buff, len);                 // Search for first frame, usually at pos=0
  if(s < 0)                                          // Sync found?
  {
    Serial.println("No mp3 word sync");
    return;                                          // Return with no effect
  }
  mp3bpnt = mp3buff + s;                             // Set pointer to start of frame
  len -= s;                                          // Decrement length
  once = true;                                       // Get samplerate once
  snprintf(buffer, sizeof(buffer),
           "Sync found at 0x%04X, len is %d", s, len);
  Serial.println(buffer);
  while(len > 0) {                                   // More MP3 data in buffer?
  
    newcnt = len;                                    // Remaining length of buffer
    n = MP3Decode(mp3bpnt, &newcnt,                  // Decode the frame
                    outbuf, 0);
    if(n == ERR_MP3_NONE)
    {
      if(once)
      {
        samprate  = MP3GetSampRate();                // Get sample rate
        channels  = MP3GetChannels();                // Get number of channels
        //int br  = MP3GetBitrate();                 // Get bit rate
        //int bps = MP3GetBitsPerSample();           // Get bits per sample
      }
    }
    ops = MP3GetOutputSamps();                       // Get number of output samples
    smpbytes = ops * 2;                              // Number of bytes in outbuf
    mp3bpnt +=(len - newcnt);                        // Increment pointer by bytes handled
    len = newcnt;                                    // Length of remaining buffer
    if(n < 0)                                        // Check if decode is okay
    {
      snprintf(buffer, sizeof(buffer), "MP3Decode error %d", n);
      return;
    }
    if(once)
    {
      //Serial.println("Bitrate     is %d", br);        // Show decoder parameters
      //Serial.println("Samprate    is %d", samprate);
      //Serial.println("Channels    is %d", channels);
      //Serial.println("Bitspersamp is %d", bps);
      //Serial.println("Outputsamps is %d", ops);
      if(channels == 1)                            // For mono, half sample rate
      {
        samprate /= 2;
      }
      i2s_set_sample_rates(i2s_num, samprate);       // Set samplerate
      once = false;                                  // No need to set samplerate again
    }
    #ifdef INTERNALDAC
    {
      for(int i = 0; i < ops; i++)               // Correct amplitude end sign
      {
        outbuf[i] = outbuf[i] * gain / 100 +          // Reduce 0..100 percent
                    0x8000;                          // Internal DAC is not signed
      }
    }
    #else
    {
      for(int i = 0; i < ops; i++)               // Correct amplitude
      {
        outbuf[i] = outbuf[i] * gain / 100;          // Reduce 0..100 percent
      }
    }
    #endif
    i2s_write(i2s_num, outbuf, smpbytes, &bw,       // Send to I2S
                100 );
  }
  #ifdef INTERNALDAC
    for(int i = 0; i < OUTSIZE; i++)
    {
      outbuf[i] = 0x8000;
    }
  #else
    memset(outbuf, 0, sizeof(outbuf));            // Fill with silence
  #endif
  while(false)
  {
    i2s_write(i2s_num, outbuf, 128, &bw, 0);       // Send to I2S
    if(bw != 128)
    {
      break;
    }
  }
}

