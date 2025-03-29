//***************************************************************************************************
//*  Main code                                                                                      *
//*  By Ed Smallenburg <https://github.com/Edzelf/>                                                 *
//***************************************************************************************************
//***************************************************************************************************
// Board:            ESP32 Development board.                                                       *
// Partition scheme: Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)                               *
//***************************************************************************************************
// This sketch uses SPIFFS for the web interface files.  For installation of FS tools see:          *
// https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/                   *
//***************************************************************************************************
//***************************************************************************************************
// Wiring.                                                                                          *
// ------------------------------------------------------------------------------------------------ *
// ESP32dev Signal  Connected to                Remarks                                             *
// -------- ------  --------------              ----------------------------------------------      *
// GPIO0    BOOT    Reserved bootloader         Also 2nd start button for test                      *
// GPIO1    TXD0    Reserved serial output                                                          *
// GPIO2            Built-in LED                                                                    *
// GPIO3    RXD0    Reserved serial input                                                           *
// GPIO4            -                                                                               *
// GPIO5            Big button                  (active low)                                        *
// GPIO12           Neopixel panel DIN                                                              *
// GPIO14           I2S BCLK naar bijv. MAX98357A / PCM5102                                         *
// GPIO15           -                                                                               *
// GPIO16   RXD2    -                                                                               *
// GPIO17   TXD2    -                                                                               *
// GPIO18   SCK     -                                                                               *
// GPIO19   MISO    -                                                                               *
// GPIO21           SDA for I2C (DS3132)                                                            *
// GPIO22           SCK for I2C (DS3132)                                                            *
// GPIO23   MOSI    -                                                                               *
// GPIO25   -       audio output van interne DAC.                                                   *
// GPIO26   -       audio output van interne DAC.                                                   *
// GPIO27   -       I2S DIN  naar bijv. MAX98357A / PCM5102                                         *
// GPIO32   -       Enable VIN of PCM5102                                                           *
// GPIO33   -       I2S LRC  naar bijv. MAX98357A / PCM5102                                         *
// GPIO34   -       GPS TX                      (input only pin)                                    *
// GPIO35   -       -                           (input only pin)                                    *
// -------  ------  ----------------                                                                *
// GND      -       Power supply GND                                                                *
// VCC 5 V  -       Power supply                                                                    *
// VCC 5 V  -       Power supply                                                                    *
// 3.3 V    -       GPS Vcc                                                                         *
// EN       -       -                                                                               *
//***************************************************************************************************
// 11-10-2021, ES: First set-up.                                                                    *
// 01-11-2021, ES: Add registratie logging.                                                         *
// 22-11-2021, ES: Output to MAX98357A.                                                             *
// 23-11-2021, ES: Log in required.                                                                 *
// 26-11-2021, ES: Replaced display with Neopixel panels.                                           *
// 03-12-2021, ES: Show IP-address.                                                                 *
// 08-12-2021, ES: Grant access using client IP address.                                            *
// 09-12-2021, ES: Rotate display 90 degrees left.                                                  *
// 16-12-2021, ES: left and rigth arrows to display character set.                                  *
// 09-03-2022, ES: Front and back Neopixel panels.                                                  *
// 10-03-2022, ES: Stoplicht/pijl model voor signalering op neopixel front panel.                   *
// 14-03-2022, ES: Pijlen links/rechts omwisselbaar.                                                *
// 01-04-2022, ES: Audio via PCM5102.                                                               *
// 01-04-2022, ES: Pijlen links/rechts/stop/go altijd rood of groen.                                *
// 05-04-2022, ES: Toon drukknop en geanimeerde pijl op front panel in rust.                        *
// 12-05-2022, ES: Replaced I2C driver with Arduino "Wire".                                         *
// 20-08-2022, ES: Correctie i.v.m. veranderde SPIFF functie.                                       *
// 30-09-2022, ES: Correctie WiFi event logging.                                                    *
//***************************************************************************************************
#define OTA                                                 // OTA should be enabled (or not)
#
#include <Arduino.h>
#include <WiFi.h>
#include <soc/soc.h>                                        // For brown-out detector setting
#include <soc/rtc_cntl_reg.h>                               // For brown-out detector setting
#ifdef OTA
  #include <ArduinoOTA.h>
#endif
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_partition.h>
#include <WiFiMulti.h>
#include <lwip/dns.h>
#include <ESPmDNS.h>
#include <TinyGPS++.h>                                      // For GPS
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <Wire.h>
#include <driver/i2s.h>                                     // Driver for I2S output
#include "mp3_decoder.h"                                    // include libhelix_HMP3DECODER
#include <NeoPixelBus.h>                                    // For NeoPixel output
#include "charset.hpp"                                      // Neopixel partial charset
#define DEBUG_BUFFER_SIZE 150
#define NVSBUFSIZE        150                               // Longest expected string for nvsgetstr
#define MAXKEYS           40                                // Max. number of NVS keys in table
#define BUTTON0           5                                 // Start button, LOW is active
#define BUTTON1           0                                 // Start button 2, LOW is active

//#define INTERNALDAC                                       // Define for internal DAC usage for I2S
                                                            // Needs platform = espressif32@3.1.0 !!!!
// INTERNALDAC does not seem to work with higher versions of framework.  Do a test.
#ifdef INTERNALDAC
  #if ESP_ARDUINO_VERSION_MAJOR >= 2
    #error Internal DAC will not work in this version of the platform.  Use espressif32@3.1.0.
  #endif
#endif


#define NAME        "HIP"                                   // Ident, also used for AP network.
#define VERSION     "Sat, 20 Aug 2022 10:30:00 GMT"

#define FSIF              true                              // Format SPIFFS if not existing
#define QSIZ              10                                // Number of entries in play queue
#define DS3231_ADDRESS    0x68                              // I2C address of RTC
#define DS3231_TIME       0x00                              // Time register
#define DS3231_STATUSREG  0x0F                              // Status register
#define DS3231_TEMPREG    0x11                              // Temperature register 10-bit

// MP3
#define I2S_ENABLE        32                                // Enable I2S/Amp if used
#define I2S_BCK_PIN       14                                // I2S BCLK naar MAX98357A / PCM5102
#define I2S_LCK_PIN       33                                // I2S LRC naar MAX98357A / PCM5102
#define I2S_DIN_PIN       27                                // I2S DIN naar MAX98357A / PCM5102
#define FRAMESIZE         1152                              // Max. frame size in bytes (mp3)
#define OUTSIZE           2048                              // Max number of samples per channel (mp3)

// GPS
#define GPSSERIAL         Serial2                           // Second serial used for GPS

// Neopixel displays (2x3 modules 8x8)
#define NEO_PANELS        8                                 // Number of panels
#define NEOPIX_PIN        12                                // GPIO for Neopixel displays
#define NEOPIX_COUNT      512                               // Total number of pixels

//***************************************************************************************************
// Forward declarations.                                                                            *
//***************************************************************************************************
esp_err_t   nvsclear() ;
bool        nvssearch ( const char* key ) ;
String      nvsgetstr ( const char* key ) ;
esp_err_t   nvssetstr ( const char* key, String val ) ;
uint32_t    a2tohex ( const char* hstr ) ;
uint32_t    a8tohex ( const char* hstr ) ;
String      getContentType ( String filename ) ;
String      httpheader ( String contentstype ) ;
String      readprefs ( bool output ) ;
void        readprefs ( AsyncWebServerRequest *request ) ;
void        writeprefs ( AsyncWebServerRequest *request ) ;
const char* analyzeCmd ( const char* str ) ;
const char* analyzeCmd ( const char* par, const char* val ) ;
void        getTimeTxt() ;
void        setRtcTime ( const long int t ) ;
bool        openRegfile() ;


//***************************************************************************************************
// Various structs and enums.                                                                       *
//***************************************************************************************************
struct WifiInfo_t                                // For list with WiFi info
{
  uint8_t inx ;                                  // Index as in "wifi_00"
  char*   ssid ;                                 // SSID for an entry
  char*   passphrase ;                           // Passphrase for an entry
} ;


struct nvs_entry
{
  uint8_t  Ns ;                                  // Namespace ID
  uint8_t  Type ;                                // Type of value
  uint8_t  Span ;                                // Number of entries used for this item
  uint8_t  Chunkindex ;                          // Reserved, Should be 0xFF for non-blob types
  uint32_t CRC ;                                 // CRC
  char     Key[16] ;                             // Key in Ascii
  uint64_t Data ;                                // Data in entry
} ;

struct nvs_page                                  // For nvs entries
{ // 1 page is 4096 bytes
  uint32_t  State ;
  uint32_t  Seqnr ;
  uint8_t   version ;
  uint8_t   Unused[19] ;
  uint32_t  CRC ;
  uint8_t   Bitmap[32] ;
  nvs_entry Entry[126] ;
} ;

struct keyname_t                                  // For keys in NVS
{
  char      Key[16] ;                             // Mac length is 15 plus delimeter
} ;

struct qdata_struct                               // Structure of date in playqueue
{
  __attribute__((aligned(4))) char filename[64] ; // File to play
  int pause ;                                     // Pause, silence period in msec
} ;

struct qdisp_struct                               // Structure of date in dispqueue
{
  bool hit ;                                      // Hit or not
  int  volgnummer ;
} ;

enum model_t  { STOPLICHT, PIJL } ;               // Mogelijke keuzes hit/nohit signalering
enum lr_t     { RECHTS, LINKS } ;                 // Keuzes rechtsuit / links uit

//**************************************************************************************************
// Global data section.                                                                            *
//**************************************************************************************************
//
int               DEBUG = 1 ;                            // Debug on/off
//
String            userpw = "bloemkool" ;                 // User password
String            adminpw = "supernova" ;                // Admin password
String            notuserpages = "/config.html,"         // Pages that are not accessible by user
                                 "/syslog.html" ;
bool              dsptaskrdy = false ;                   // Signal for dislay task ready 
model_t           model = PIJL ;                         // Keuze signalering, default pijl
lr_t              lr = RECHTS ;                          // Keuze rechts uit / links uit
byte              mac[6] ;                               // WiFi mac address
int               numSsid ;                              // Number of available WiFi networks
WiFiMulti         wifiMulti ;                            // Possible WiFi networks
bool              wifiok = false ;                       // True if WiFi okay
IPAddress         ip ;                                   // Local IP address
char              ipstr[24] ;                            // Text with IP address, including "IP="
char              AP_SSIDstr[20] ;                       // Name of AP network
char              AP_pw[20] ;                            // Password of AP network
AsyncWebServer    cmdserver ( 80 ) ;                     // Instance of embedded webserver, port 80
hw_timer_t*       timer = nullptr ;                      // For timer
bool              secondpast ;                           // One second has passed
bool              NetworkFound = false ;                 // True if WiFi network connected
uint16_t          reconcount = 0 ;                       // Number of WiFi reconnects
String            networks ;                             // Found networks in the surrounding
TaskHandle_t      xplaytask ;                            // Task handle for playtask
TaskHandle_t      xdisptask ;                            // Task handle for disptask
TaskHandle_t      xmaintask ;                            // Taskhandle for main task
int32_t           xmaincount = 0 ;                       // Loop counter for main task
SemaphoreHandle_t dbgsem = NULL ;                        // For exclusive usage of dbgprint
SemaphoreHandle_t i2csem = NULL ;                        // For exclusive usage of I2C bus
QueueHandle_t     playqueue ;                            // Queue for playtask
QueueHandle_t     dispqueue ;                            // Queue for disptask
struct tm         timeinfo = {0,0,0,0,0,0,0,0,0} ;       // Will be filled with current time
char              timetxt[9] = "00:00:00" ;              // Converted time
char              rtctemptxt[9] = "0.0" ;                // Converted temp from RTC clock
uint32_t          seconds = 0 ;                          // Seconds counter, free running
int               numprefs = 0 ;                         // Aantal gevonden preferences
char              cmd[130] ;                             // Command from Serial
bool              cmd_req = false ;                      // Commando ready in cmd
bool              resetreq = false ;                     // Request to reset the ESP32
bool              buttonreq = false ;                    // True if button has been pushed
bool              pwresetreq = false ;                   // True if password have to be reset
uint32_t          clength ;                              // Content length found in http header
uint16_t          errorcount = 0 ;                       // Total errors detected
int8_t            pin_i2c_sda = SDA ;                    // I2C pin data
int8_t            pin_i2c_scl = SCL ;                    // I2C pin Clock
int8_t            pin_tft_cs = 15 ;                      // Display CS pin
int8_t            pin_tft_dc = 4 ;                       // Display DC pin
int8_t            pin_gps_in = 34 ;                      // Input from GPS TX pin
//                                                       // Neopixel stuff
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod>             // Global object
                  neodisp ( NEOPIX_COUNT, NEOPIX_PIN ) ;
RgbColor          n_white = RgbColor ( 255, 255, 255 ) ; // Colors used for Neopixels
RgbColor          n_black = RgbColor ( 0,     0,   0 ) ; // All colors off
RgbColor          n_red   = RgbColor ( 255,   0,   0 ) ; // Red
RgbColor          n_green = RgbColor ( 0,   255,   0 ) ; // Green
RgbColor          n_blue  = RgbColor ( 0,     0, 255 ) ; // Blue
uint8_t           n_brightness = 30 ;                    // Brightness 0..100
//                                                       // MP3 stuff
uint8_t*          mp3buff = nullptr ;                    // Space allocated for complete MP3 file
uint8_t*          mp3bpnt ;                              // Points into mp3buff
int               mp3bcnt ;                              // Number of samples in buffer
bool              searchFrame ;                          // True if search for startframe is needed
int16_t           outbuf[OUTSIZE*2] ;                    // I2S buffer
int               smpwords ;                             // Number of 16 bit words for I2S
int16_t           gain = 50 ;                            // Gain 0..100
//                                                       // Application stuff
int               probability = 20 ;                     // Probability in percent
bool              au_V_ned = true ;                      // Audio volgnummer Nederlands ON/OFF
bool              au_T_ned = true ;                      // Audio tekst Nederlands ON/OFF
bool              au_V_eng = true ;                      // Audio volgnummer Engels ON/OFF
bool              au_T_eng = true ;                      // Audio tekst Engels ON/OFF
int               volgnummer =  0 ;                      // Volgnummer
int               trefnummer =  0 ;                      // Trefnummer
char              treftijd[9]   = "00:00:00" ;           // Tijdstip treffer
//                                                       // GPS stuff
TinyGPSPlus       gps ;                                  // GPS class instance
double            gps_lat = 0.0 ;                        // Latitude
double            gps_lon = 0.0 ;                        // Longitude
String            gps_lat_s ;                            // Latitude als string
String            gps_lon_s ;                            // Longitude as string
int               gps_nsat = 0 ;                         // Aantal satelieten
char              gpsOK = ' ' ;                          // Wordt "G" indien okay
//                                                       // Registration file stuff
char              regfilename[32] ;                      // Name of the registration file
File              regfile ;                              // Set if registration file opened
//                                                       // NVS stuff
uint8_t                 namespace_ID ;                   // Namespace ID
char                    nvs_val[NVSBUFSIZE] ;            // Buffer for value of one NVS key
nvs_page                nvsbuf ;                         // Space for 1 page of NVS info
const esp_partition_t*  nvs = nullptr ;                  // Pointer to NVS partition struct
esp_err_t               nvserr ;                         // Error code from nvs functions
nvs_handle              nvshandle = 0 ;                  // Handle for nvs access
char                    nvskeys[MAXKEYS][16] ;           // Space for NVS keys
std::vector<WifiInfo_t> wifilist ;                       // List with wifi_xx info
std::vector<keyname_t>  keynames ;                       // Keynames in NVS
std::vector<String>     dbglines ;                       // Container for last debug lines
std::vector<uint32_t>   liulist ;                        // List with loggged-in users
std::vector<uint32_t>   lialist ;                        // List with loggged-in admins
const esp_partition_t*  pspiffs = nullptr ;              // Pointer to SPIFFS partition struct

//**************************************************************************************************
// End of global data section.                                                                     *
//**************************************************************************************************

//***************************************************************************************************
// Forward declarations.                                                                            *
//***************************************************************************************************
void      dbgprint ( const char* format, ... ) ;
uint32_t  getRtcTime() ;
void      initRtc() ;


//**************************************************************************************************
//                                          D B G P R I N T                                        *
//**************************************************************************************************
// Send a line of info to serial output.  Works like vsprintf(), but checks the DEBUG flag.        *
// Debug lines will be added to dbglines, a buffer holding the last debuglines.                    *
// Print only if DEBUG flag is true.                                                               *
//**************************************************************************************************
void dbgprint ( const char* format, ... )
{
  static char sbuf[DEBUG_BUFFER_SIZE] ;                // For debug lines
  va_list varArgs ;                                    // For variable number of params
  String   dbgline = "" ;                              // Resulting line

  if ( xSemaphoreTake ( dbgsem, 200 ) != pdTRUE  )     // Claim resource
  {
    return ;                                           // Not available
  }
  va_start ( varArgs, format ) ;                       // Prepare parameters
  vsnprintf ( sbuf, sizeof(sbuf), format, varArgs ) ;  // Format the message
  va_end ( varArgs ) ;                                 // End of using parameters
  if ( DEBUG )                                         // DEBUG on?
  {
    getTimeTxt() ;                                     // Get current time
    dbgline = String ( timetxt ) + 
              String ( " - " ) +
              String ( sbuf ) ;                        // Info to a String
    Serial.println ( dbgline.c_str() ) ;               // and the info
    if ( ESP.getFreeHeap() > 100000 )                  // Memory shortage due to debug lines?
    {
      dbglines.push_back ( dbgline ) ;                 // Add to buffer with debug lines
    }
  }
  xSemaphoreGive ( dbgsem ) ;                          // Release resource
}


//**************************************************************************************************
//                                    P L A Y B U F F                                              *
//**************************************************************************************************
// Play the len bytes from MP3 buffer.                                                             *
//**************************************************************************************************
void playBuff ( i2s_port_t i2s_num, int len )
{
  int             newcnt ;                            // Number of bytes remaining after decoding
  static uint32_t samprate ;                          // Sample rate
  static int      channels ;                          // Number of channels
  static int      smpbytes ;                          // Number of bytes for I2S
  int             s ;                                 // Position of syncword
  static bool     once ;                              // To execute part of code once
  int             n ;                                 // Number of samples decoded
  size_t          bw ;                                // I2S bytes written
  int             ops = 0 ;                           // Number of output samples

  s = MP3FindSyncWord ( mp3buff, len ) ;              // Search for first frame, usually at pos=0
  if ( s < 0 )                                        // Sync found?
  {
    return ;                                          // Return with no effect
  }
  mp3bpnt = mp3buff + s ;                             // Set pointer to start of frame
  len -= s ;                                          // Decrement length
  once = true ;                                       // Get samplerate once
  //dbgprint ( "Sync found at 0x%04X, len is %d",
  //           s, len ) ;
  while ( len > 0 )                                   // More MP3 data in buffer?
  {
    newcnt = len ;                                    // Remaining length of buffer
    n = MP3Decode ( mp3bpnt, &newcnt,                 // Decode the frame
                    outbuf, 0 ) ;
    if ( n == ERR_MP3_NONE )
    {
      if ( once )
      {
        samprate  = MP3GetSampRate() ;                // Get sample rate
        channels  = MP3GetChannels() ;                // Get number of channels
        //int br  = MP3GetBitrate() ;                 // Get bit rate
        //int bps = MP3GetBitsPerSample() ;           // Get bits per sample
      }
    }
    ops = MP3GetOutputSamps() ;                       // Get number of output samples
    smpbytes = ops * 2 ;                              // Number of bytes in outbuf
    mp3bpnt += ( len - newcnt ) ;                     // Increment pointer by bytes handled
    len = newcnt ;                                    // Length of remaining buffer
    if ( n < 0 )                                      // Check if decode is okay
    {
      dbgprint ( "MP3Decode error %d", n ) ;
      return ;
    }
    if ( once )
    {
      //dbgprint ( "Bitrate     is %d", br ) ;        // Show decoder parameters
      //dbgprint ( "Samprate    is %d", samprate ) ;
      //dbgprint ( "Channels    is %d", channels ) ;
      //dbgprint ( "Bitspersamp is %d", bps ) ;
      //dbgprint ( "Outputsamps is %d", ops ) ;
      if ( channels == 1 )                            // For mono, half sample rate
      {
        samprate /= 2 ;
      }
      i2s_set_sample_rates ( i2s_num, samprate ) ;    // Set samplerate
      once = false ;                                  // No need to set samplerate again
    }
    #ifdef INTERNALDAC
    {
      for ( int i = 0 ; i < ops ; i++ )               // Correct amplitude end sign
      {
        outbuf[i] = outbuf[i] * gain / 100 +          // Reduce 0..100 percent
                    0x8000 ;                          // Internal DAC is not signed
      }
    }
    #else
    {
      for ( int i = 0 ; i < ops ; i++ )               // Correct amplitude
      {
        outbuf[i] = outbuf[i] * gain / 100 ;          // Reduce 0..100 percent
      }
    }
    #endif
    i2s_write ( i2s_num, outbuf, smpbytes, &bw,       // Send to I2S
                100  ) ;
  }
  #ifdef INTERNALDAC
    for ( int i = 0 ; i < OUTSIZE ; i++ )
    {
      outbuf[i] = 0x8000 ;
    }
  #else
    memset ( outbuf, 0, sizeof(outbuf) ) ;            // Fill with silence
  #endif
  while ( false )
  {
    i2s_write ( i2s_num, outbuf, 128, &bw, 0 ) ;       // Send to I2S
    if ( bw != 128 )
    {
      break ;
    }
  }
}


//**************************************************************************************************
//                                      G E T T I M E T X T                                        *
//**************************************************************************************************
// Result is stored as a string in timetxt.                                                        *
// Not re-entrant.                                                                                 *
//**************************************************************************************************
void getTimeTxt()
{
  uint8_t    sec ;                                     // Seconds in timeinfo

  do
  {
    sec = timeinfo.tm_sec ;                            // Copy to detect change
    sprintf ( timetxt, "%02d:%02d:%02d",               // Format time to a string
              timeinfo.tm_hour,
              timeinfo.tm_min,
              timeinfo.tm_sec ) ;                      // Fill timeinfo
  } while ( sec != timeinfo.tm_sec ) ;                 // Maybe time changed by interrupt
}



//**************************************************************************************************
//                              B C D 2 B I N and B I N 2 B C D                                    *
//**************************************************************************************************
// Conversion for RTC values.                                                                      *
//**************************************************************************************************
uint8_t bcd2bin ( uint8_t val )
{
  return val - 6 * ( val >> 4 ) ;
}

uint8_t bin2bcd ( uint8_t val )
{
  return val + 6 * ( val / 10 ) ;
}


//**************************************************************************************************
//                                       I N I T R T C                                             *
//**************************************************************************************************
// Init the RTC.                                                                                   *
//**************************************************************************************************
void initRtc()
{
  uint8_t     b ;                                        // Status byte from RTC
  const long  ut = 1627053420 ;                          // 23-07-2021 15:17

  Wire.beginTransmission ( DS3231_ADDRESS ) ;            // Begin transmission
  Wire.write ( DS3231_STATUSREG ) ;                      // Prepare to read from status register
  Wire.endTransmission() ;                               // Close channel
  Wire.requestFrom ( DS3231_ADDRESS, 1 ) ;               // Request status register
  b = Wire.read() ;                                      // Read the status byte
  if ( b & BIT7 )                                        // Bit 7 is "power loss"
  {
    dbgprint ( "RTC lost power! Init time" ) ;           // Power los, report
    setRtcTime ( ut ) ;                                  // And set time to usable time
  }
}


//**************************************************************************************************
//                                      G E T R T C T I M E                                        *
//**************************************************************************************************
// Read time from RTC and convert.                                                                 *
// Result is also stored as a string in timetxt.                                                   *
//**************************************************************************************************
uint32_t getRtcTime()
{
  int     year ;    	                                        // Year
  uint8_t tb[7] ;                                             // 7 time registers

  if ( xSemaphoreTake ( i2csem, 200 ) == pdTRUE  )            // Claim resource
  {
    Wire.beginTransmission ( DS3231_ADDRESS ) ;               // Connect to RTC
    Wire.write ( DS3231_TIME ) ;                              // Prtepare to read time registers
    Wire.endTransmission() ;                                  // Close channel
    Wire.requestFrom ( DS3231_ADDRESS , 7 ) ;                 // Prepare to read from 7 time registers
    Wire.readBytes ( tb, sizeof(tb) ) ;                       // Read 6 time registers
    timeinfo.tm_sec  = bcd2bin ( tb[0] ) ;
    timeinfo.tm_min  = bcd2bin ( tb[1] ) ;
    timeinfo.tm_hour = bcd2bin ( tb[2] ) ;
    timeinfo.tm_wday = tb[3] - 1 ;                            // Day 1..7 to 0..6
    timeinfo.tm_mday = bcd2bin ( tb[4] ) ;
    timeinfo.tm_mon  = bcd2bin ( tb[5] & 0x7F ) - 1 ;         // Month 0..11
    year             = bcd2bin ( tb[6] ) ;                    // Years 00..99
    timeinfo.tm_year = year ;                                 // Years since 1900
    if ( tb[5] & BIT7 )                                       // century bit?
    {
      timeinfo.tm_year += 100 ;                               // Yes, add 100 years
    }
    timeinfo.tm_isdst = -1 ;                                  // DST not available
    xSemaphoreGive ( i2csem ) ;                               // Release resource
  }
  getTimeTxt() ;                                              // Format new time to a string
  return mktime ( &timeinfo ) ;                               // Return current time
}


//**************************************************************************************************
//                                      S E T R T C T I M E                                        *
//**************************************************************************************************
// Set RTC time.                                                                                   *
//**************************************************************************************************
void  setRtcTime ( const long int t )
{
  uint8_t    tb[7] ;                                     // 7 time registers
  uint8_t    mc = 0 ;                                    // Century data
  int        year ;    	                                 // Year
  uint8_t    statreg = 0 ;                               // Status register
  struct tm* x ;                                         // Time as tm struct 

  x = localtime ( &t ) ;                                 // Get time as a struct
  year = x->tm_year ;                                    // Get number of years since 1900
  if ( year >= 100 )                                     // 2000 or later?  Probably later.
  {
    year -= 100 ;                                        // Yes overflow in century bit
    mc = 0x80 ;                                          // Set overflow bit
  }
  tb[0] = bin2bcd ( x->tm_sec ) ;
  tb[1] = bin2bcd ( x->tm_min ) ;
  tb[2] = bin2bcd ( x->tm_hour ) ;
  // The RTC must know the day of the week for the weekly alarms to work.
  tb[3] = bin2bcd ( x->tm_wday ) ;
  tb[4] = bin2bcd ( x->tm_mday ) ;
  tb[5] = bin2bcd ( x->tm_mon + 1 ) | mc ;               // Month 1..11 plus century
  tb[6] = bin2bcd ( year ) ;                             // Year mod 100
  if ( xSemaphoreTake ( i2csem, 200 ) == pdTRUE  )       // Claim resource
  {
    Wire.beginTransmission ( DS3231_ADDRESS ) ;          // Connect to RTC
    Wire.write ( DS3231_TIME ) ;                         // Prepare to write to time registers
    Wire.write ( tb, sizeof(tb) ) ;                      // Write time registers
    Wire.endTransmission() ;                             // Close channel
    Wire.beginTransmission ( DS3231_ADDRESS ) ;          // Connect again to RTC
    Wire.write ( DS3231_STATUSREG ) ;                    // Prepare to read 1 status reg
    Wire.endTransmission() ;                             // Close channel
    Wire.requestFrom ( DS3231_ADDRESS, 1 ) ;             // Request the status register
    statreg = Wire.read() ;                              // Read the status byte
    Wire.endTransmission() ;                             // Close channel
    statreg &= ~BIT7 ;                                   // Clear OSF bit
    Wire.beginTransmission ( DS3231_ADDRESS ) ;          // Connect again to RTC
    Wire.write ( DS3231_STATUSREG ) ;                    // Prepare to write to status register
    Wire.write ( statreg ) ;                             // Write the status byte
    Wire.endTransmission() ;                             // Close channel
    xSemaphoreGive ( i2csem ) ;                          // Release resource
  }
  dbgprint ( "RTC status is 0x%02X", statreg ) ;         // Show status
  dbgprint ( "Set %02X, %02X, %02X, %02X,"               // For debugging RTC
             " %02X, %02X, %02X",
              tb[0], tb[1], tb[2], tb[3],
              tb[4], tb[5], tb[6] ) ;
}


//**************************************************************************************************
//                                      G E T R T C T E M P                                        *
//**************************************************************************************************
// Read temperature from RTC and convert.                                                          *
//**************************************************************************************************
float getRtcTemp()
{
  uint8_t temp[2] ;                                    // Temperature in 2 bytes
  float   tf ;                                         // Temperature from RTC
  String  ts ;                                         // Temperature as string

  if ( xSemaphoreTake ( i2csem, 200 ) != pdTRUE  )     // Claim resource
  {
    return 0.0 ;                                       // Not available
  }
  Wire.beginTransmission ( DS3231_ADDRESS ) ;          // Connect to RTC
  Wire.write ( DS3231_TEMPREG ) ;                      // Prepare to read temp register
  Wire.endTransmission() ;                             // Close channel
  Wire.requestFrom ( DS3231_ADDRESS, sizeof(temp ) ) ; // Prepare to read from temp. register
  Wire.readBytes ( temp, sizeof(temp) ) ;              // Read 2 temp bytes
  xSemaphoreGive ( i2csem ) ;                          // Release resource
  tf = (float)temp[0] + ( temp[1] >> 6 ) * 0.25f ;
  ts = String ( tf, 1 ) ;
  sprintf ( rtctemptxt, "%s", ts.c_str() ) ;           // Format new temp to a string
  return tf ;
}


//**************************************************************************************************
//                                         C H O M P                                               *
//**************************************************************************************************
// Do some filtering on de inputstring:                                                            *
//  - String comment part (starting with "#").                                                     *
//  - Strip trailing CR.                                                                           *
//  - Strip leading spaces.                                                                        *
//  - Strip trailing spaces.                                                                       *
//**************************************************************************************************
void chomp ( String &str )
{
  int   inx ;                                         // Index in de input string

  if ( ( inx = str.indexOf ( "#" ) ) >= 0 )           // Comment line or partial comment?
  {
    str.remove ( inx ) ;                              // Yes, remove
  }
  str.trim() ;                                        // Remove spaces and CR
}


//**************************************************************************************************
//                                           O T A S T A R T                                       *
//**************************************************************************************************
// Update via WiFi has been started by PlatformIO IDE.                                             *
//**************************************************************************************************
#ifdef OTA
void otastart()
{
  dbgprint ( "OTA update Started" ) ;
}
#endif


//**************************************************************************************************
//                            S C A N _ C O N T E N T _ L E N G T H                                *
//**************************************************************************************************
// If the line contains content-length information: set clength (content length counter).          *
//**************************************************************************************************
void scan_content_length ( const char* metalinebf )
{
  if ( strstr ( metalinebf, "Content-Length" ) )        // Line contains content length
  {
    clength = atoi ( metalinebf + 15 ) ;                // Yes, set clength
    dbgprint ( "Content-Length is %d", clength ) ;      // Show for debugging purposes
  }
}


//**************************************************************************************************
//                                          T I M E R 1 S E C                                      *
//**************************************************************************************************
// Timing.  Called every second.                                                                   *
// Note that calling timely procedures within this routine or in called functions will             *
// cause a crash!                                                                                  *
//**************************************************************************************************
void IRAM_ATTR timer1sec()
{
  seconds++ ;                                           // Update seconds counter
  if ( ++timeinfo.tm_sec >= 60 )                        // Update number of seconds
  {
    timeinfo.tm_sec = 0 ;                               // Wrap after 60 seconds
    if ( ++timeinfo.tm_min >= 60 )
    {
      timeinfo.tm_min = 0 ;                             // Wrap after 60 minutes
      if ( ++timeinfo.tm_hour >= 24 )
      {
        timeinfo.tm_hour = 0 ;                          // Wrap after 24 hours
        timeinfo.tm_mday++ ;                            // Update day of month
        timeinfo.tm_wday++ ;                            // Update weekday
        timeinfo.tm_wday %= 7 ;                         // wrap if necessary
        timeinfo.tm_yday++ ;                            // Update days since 1 jan
      }
    }
  }
}


//**************************************************************************************************
//                                          T I M E R 1 0 0                                        *
//**************************************************************************************************
// Called every 100 msec on interrupt level, so must be in IRAM and no lengthy operations          *
// allowed.                                                                                        *
//**************************************************************************************************
void IRAM_ATTR timer100()
{
  static int16_t   countsec = 0 ;                 // Counter for activatie 10 seconds process

  if ( ++countsec == 10 )                         // One second over?
  {
    timer1sec() ;                                 // Yes, call 1 second function
    countsec = 0 ;                                // Reset counter
  }
}


//**************************************************************************************************
//                            B U B B L E S O R T K E Y S                                          *
//**************************************************************************************************
// Bubblesort the nvskeys.                                                                         *
//**************************************************************************************************
void bubbleSortKeys ( uint16_t n )
{
  uint16_t i, j ;                                             // Indexes in nvskeys
  char     tmpstr[32] ;                                       // Temp. storage for a key

  for ( i = 0 ; i < n - 1 ; i++ )                             // Examine all keys
  {
    for ( j = 0 ; j < n - i - 1 ; j++ )                       // Compare to following keys
    {
      if ( strcmp ( nvskeys[j], nvskeys[j + 1] ) > 0 )        // Next key out of order?
      {
        strcpy ( tmpstr, nvskeys[j] ) ;                       // Save current key a while
        strcpy ( nvskeys[j], nvskeys[j + 1] ) ;               // Replace current with next key
        strcpy ( nvskeys[j + 1], tmpstr ) ;                   // Replace next with saved current
      }
    }
  }
}


//**************************************************************************************************
//                                      F I L L K E Y L I S T                                      *
//**************************************************************************************************
// File the list of all keys in NVS.                                                               *
// The keys will be sorted.                                                                        *
//**************************************************************************************************
void fillkeylist()
{
  esp_err_t    result = ESP_OK ;                                // Result of reading partition
  uint32_t     offset = 0 ;                                     // Offset in nvs partition
  uint16_t     i ;                                              // Index in Entry 0..125.
  uint8_t      bm ;                                             // Bitmap for an entry
  uint16_t     nvsinx = 0 ;                                     // Index in nvskey table

  keynames.clear() ;                                            // Clear the list
  while ( offset < nvs->size )
  {
    result = esp_partition_read ( nvs, offset,                  // Read 1 page in nvs partition
                                  &nvsbuf,
                                  sizeof(nvsbuf) ) ;
    if ( result != ESP_OK )
    {
      dbgprint ( "Error reading NVS!" ) ;
      break ;
    }
    i = 0 ;
    while ( i < 126 )
    {
      bm = ( nvsbuf.Bitmap[i / 4] >> ( ( i % 4 ) * 2 ) ) ;      // Get bitmap for this entry,
      bm &= 0x03 ;                                              // 2 bits for one entry
      if ( bm == 2 )                                            // Entry is active?
      {
        int c = nvsbuf.Entry[i].Key[0] ;
        if ( ( nvsbuf.Entry[i].Ns == namespace_ID ) &&          // Namespace right?
             ( c >= 'a' ) &&                                    // Key okay?
             ( c <= 'z' ) )
        {
          //dbgprint ( "Ns is %d, key is %s", nvsbuf.Entry[i].Ns,
          //           nvsbuf.Entry[i].Key ) ;
          strcpy ( nvskeys[nvsinx], nvsbuf.Entry[i].Key ) ;     // Yes, save in table
          if ( ++nvsinx == MAXKEYS )
          {
            nvsinx-- ;                                          // Prevent excessive index
          }
        }
        if ( nvsbuf.Entry[i].Span == 0 )                        // may not be 0
        {
          offset = nvs->size ;                                  // Leave outer loop as well
          break ;
        }
        i += nvsbuf.Entry[i].Span ;                             // Next entry
      }
      else
      {
        i++ ;
      }
    }
    offset += sizeof(nvs_page) ;                                // Prepare to read next page in nvs
  }
  nvskeys[nvsinx][0] = '\0' ;                                   // Empty key at the end
  dbgprint ( "Read %d keys from NVS", nvsinx ) ;
  bubbleSortKeys ( nvsinx ) ;                                   // Sort the keys
}


//**************************************************************************************************
//                                  A D M I N A C C E S S                                          *
//**************************************************************************************************
// Check if the client has admin access.                                                           *
//**************************************************************************************************
bool adminaccess ( AsyncWebServerRequest *request )
{
  uint32_t rip ;                                              // Remote IP (client)

  rip = request->client()->getRemoteAddress() ;               // Get remote IP
  for ( int i = 0 ; i < lialist.size() ; i++ )                // Search in admin list
  {
    if ( lialist[i] == rip )                                  // Match?
    {
      return true ;                                           // Yes, access okay
    }
  }
  return false ;                                              // No match, no access
}


//**************************************************************************************************
//                                   U S E R A C C E S S                                           *
//**************************************************************************************************
// Check if the client has user access.                                                            *
// Note that an "admin" has always access.                                                         *
//**************************************************************************************************
bool useraccess ( AsyncWebServerRequest *request )
{
  uint32_t rip ;                                              // Remote IP (client)

  rip = request->client()->getRemoteAddress() ;               // Get remote IP
  if ( adminaccess ( request ) )                              // User has admin rights?
  {
    return true ;                                             // Yes, access is okay
  }
  for ( int i = 0 ; i < liulist.size() ; i++ )                // Search in user list
  {
    if ( liulist[i] == rip )                                  // Match?
    {
      return true ;                                           // Yes, access okay
    }
  }
  return false ;                                              // No match, no access
}


//**************************************************************************************************
//                                  H A N D L E F I L E R E A D                                    *
//**************************************************************************************************
// Transfer file van SPIFFS naar webserver client.                                                 *
//**************************************************************************************************
void handleFileRead ( AsyncWebServerRequest *request )
{
  String       ct ;                                   // Content type
  String       path = "/login.html" ;                 // Filename for SPIFFS
  String       reqpath = request->url() ;             // Requested path

  if ( reqpath.indexOf ( ".html" ) < 0 )              // Non-html is always allowed
  {
    path = reqpath ;                                  // So use this page
  }
  else if ( adminaccess ( request ) )                 // Full access?
  {
    path = reqpath ;                                  // Yes, set path
  }
  else if ( useraccess ( request ) )                  // User has access?
  {
    if ( notuserpages.indexOf ( reqpath ) < 0 )       // Alowed for user?
    {
      path = reqpath ;                                // Yes, set path
    }
  }
  //dbgprint ( "Handle file read, path is %s",
  //           path.c_str() ) ;
  ct = getContentType ( path ) ;                      // Get content type
  if ( SPIFFS.exists ( path ) )                       // Does it exist in SPIFFS?
  {
    //dbgprint ( "File exists" ) ;
    request->send ( SPIFFS, path, ct ) ;              // Send to client
  }
  else
  {
    //dbgprint ( "File does not exists!" ) ;
    request->send ( 404 ) ;                           // Page not found
  }
}


//**************************************************************************************************
//                                     S C A N S E R I A L                                         *
//**************************************************************************************************
// Listen to commands on the Serial inputline.                                                     *
//**************************************************************************************************
void scanserial()
{
  static String serialcmd ;                      // Command from Serial input
  char          c ;                              // Input character
  const char*   reply = "" ;                     // Reply string from analyzeCmd
  uint16_t      len ;                            // Length of input string

  while ( Serial.available() )                   // Any input seen?
  {
    c =  (char)Serial.read() ;                   // Yes, read the next input character
    //Serial.write ( c ) ;                       // Echo
    len = serialcmd.length() ;                   // Get the length of the current string
    if ( ( c == '\n' ) || ( c == '\r' ) )
    {
      if ( len )
      {
        strncpy ( cmd, serialcmd.c_str(),
                  sizeof(cmd) ) ;
        reply = analyzeCmd ( cmd ) ;             // Analyze command and handle it
        dbgprint ( reply ) ;                     // Result for debugging
        serialcmd = "" ;                         // Prepare for new command
      }
    }
    if ( c >= ' ' )                              // Only accept useful characters
    {
      serialcmd += c ;                           // Add to the command
    }
    if ( len >= ( sizeof(cmd) - 2 )  )           // Check for excessive length
    {
      serialcmd = "" ;                           // Too long, reset
    }
  }
}


//**************************************************************************************************
//                                        W R I T E P R E F S                                      *
//**************************************************************************************************
// Update the preferences.  Called from the web interface.                                         *
// Parameter is a string with multiple HTTP key/value pairs.                                       *
//**************************************************************************************************
void writeprefs ( AsyncWebServerRequest *request )
{
  int        numargs ;                            // Number of arguments
  int        i ;                                  // Index in arguments
  String     key ;                                // Name of parameter i
  String     contents ;                           // Value of parameter i

  nvsclear() ;                                    // Remove all preferences
  numargs = request->params() ;                   // Haal aantal parameters
  for ( i = 0 ; i < numargs ; i++ )               // Scan de parameters
  {
    key = request->argName ( i ) ;                // Get name (key)
    contents = request->arg ( i ) ;               // Get value
    chomp ( key ) ;                               // Remove leading/trailing spaces
    chomp ( contents ) ;                          // Remove leading/trailing spaces
    if ( key == "version" )                       // Skip de "version" parameter
    {
      continue ;
    }
    dbgprint ( "Handle NVS setting %s = %s",
               key.c_str(), contents.c_str() ) ;  // Show key/value pair
    nvssetstr ( key.c_str(), contents ) ;         // Save new pair
  }
  nvs_commit( nvshandle ) ;
  fillkeylist() ;                                 // Update list with keys
}


//**************************************************************************************************
//                                       R E A D P R E F S                                         *
//**************************************************************************************************
// Read the preferences and interpret the commands.                                                *
// If output == true, the key / value pairs are returned to the caller as a String.                *
// The number of found preferences will be stored in numprefs.                                     *
//**************************************************************************************************
String readprefs ( bool output )
{
  uint16_t    i ;                                           // Loop control
  String      val ;                                         // Contents of preference entry
  String      cmd ;                                         // Command for analyzCmd
  String      outstr = "" ;                                 // Outputstring
  char*       key ;                                         // Point to nvskeys[i]

  fillkeylist() ;                                           // Update list with keys
  i = 0 ;
  while ( *( key = nvskeys[i] ) )                           // Loop trough all available keys
  {
    val = nvsgetstr ( key ) ;                               // Read value of this key
    cmd = String ( key ) +                                  // Yes, form command
          String ( " = " ) +
          val ;
    if ( output )
    {
      outstr += String ( key ) +                            // Add to outstr
                String ( " = " ) +
                val +
                String ( "\n" ) ;                           // Add newline
    }
    else
    {
      chomp ( val ) ;                                       // Do not show comments
      if ( cmd.startsWith ( "wifi_" ) )                     // Hide WiFi network and passwors
      {
        val = String ( "*****/*****" ) ;                    // by replacing with garbage
      }
      dbgprint ( "Config %-15s = %s",                       // Preferences to syslog
                 key, val.c_str() ) ;
      analyzeCmd ( cmd.c_str() ) ;                          // Analyze it
    }
    i++ ;                                                   // Next key
  }
  if ( i == 0 )
  {
    outstr = String ( "No preferences found.\n" ) ;
  }
  numprefs = i ;                                            // Save number of keys found
  return outstr ;
}


//**************************************************************************************************
//                                     A N A L Y Z E C M D                                         *
//**************************************************************************************************
// Handling of the various commands from remote webclient or Serial.                               *
// Version for handling string with: <parameter>=<value>                                           *
//**************************************************************************************************
const char* analyzeCmd ( const char* str )
{
  char*        value ;                           // Points to value after equalsign in command
  const char*  res ;                             // Result of analyzeCmd

  value = strstr ( str, "=" ) ;                  // See if command contains a "="
  if ( value )
  {
    *value = '\0' ;                              // Separate command from value
    res = analyzeCmd ( str, value + 1 ) ;        // Analyze command and handle it
    *value = '=' ;                               // Restore equal sign
  }
  else
  {
    res = analyzeCmd ( str, "0" ) ;              // No value, assume zero
  }
  return res ;
}


//**************************************************************************************************
//                                     A N A L Y Z E C M D                                         *
//**************************************************************************************************
// Handling of the various commands from remote webclient or serial.                               *
// par holds the parametername and val holds the value.                                            *
// Examples with available parameters:                                                             *
//   trefkans                     // Trefkans in procenten                                         *
//   brightness                   // Helderheid van display 0..100                                 *
//   gain                         // Versterking 0..100                                            *
//   userpw                       // User password                                                 *
//   adminpw                      // Admin password                                                *
//   au_v_ned                     // Audio volgnummer Nederlands 1/0                               *
//   au_t_ned                     // Audio teksten Nederlands 1/0                                  *
//   au_v_eng                     // Audio volgnummer Engels 1/0                                   *
//   au_t_eng                     // Audio teksten Engels 1/0                                      *
//   model                        // "pijl" of "stoplicht" of                                      *
//   lrrev                        // "rechts" of "links" (uitgang no hit)                          *
//   test                         // For test purposes                                             *
//   reset                        // Restart the ESP32                                             *
//**************************************************************************************************
const char* analyzeCmd ( const char* par, const char* val )
{
  String       argument ;                               // Argument as string
  String       value ;                                  // Value of an argument as a string
  int          ivalue ;                                 // Value of an argument as an integer
  static char  reply[180] ;                             // Reply to client, will be returned
  String       tmpstr ;                                 // Temporary for value

  strcpy ( reply, "Command accepted" ) ;                // Default reply
  argument = String ( par ) ;                           // Get the argument
  chomp ( argument ) ;                                  // Remove comment and useless spaces
  if ( argument.length() == 0 )                         // Lege commandline (comment)?
  {
    return reply ;                                      // Ignore
  }
  argument.toLowerCase() ;                              // Force to lower case
  value = String ( val ) ;                              // Get the specified value
  ivalue = value.toInt() ;                              // Convert to integer
  chomp ( value ) ;                                     // Remove comment and extra spaces
  //ivalue = value.toInt() ;                            // Get argument as integer
  if ( value.length() )
  {
    tmpstr = value ;                                    // Make local copy of value
  }
  else
  {
    dbgprint ( "Command: %s (without parameter)",
               argument.c_str() ) ;
  }
  // Behandle de parameters
  if ( argument.startsWith ( "trefkans" ) )             // Trefkans?
  {
    probability = ivalue ;                              // Yes, set new value
  }
  else if ( argument == "bright" )                      // Brightness setting?
  {
    n_brightness = ivalue ;                             // Yes set brightness
  }
  else if ( argument == "gain" )                        // Versterking (0..100)?
  {
    gain = ivalue ;                                     // Yes set gain
  }
  else if ( argument == "au_v_ned" )                    // Audio Nederlands volgnummer 1/0?
  {
    au_V_ned = ( ivalue != 0 ) ;                        // Yes set/reset option
  }
  else if ( argument == "au_t_ned" )                    // Audio Nederlands teksten 1/0?
  {
    au_T_ned = ( ivalue != 0 ) ;                        // Yes set/reset option
  }
  else if ( argument == "au_v_eng" )                    // Audio Engels volgnummer 1/0?
  {
    au_V_eng = ( ivalue != 0 ) ;                        // Yes set/reset option
  }
  else if ( argument == "au_t_eng" )                    // Audio Engelse teksten 1/0?
  {
    au_T_eng = ( ivalue != 0 ) ;                        // Yes set/reset option
  }
  else if ( argument == "userpw" )                      // User password setting?
  {
    userpw = value ;                                    // Yes set user password
  }
  else if ( argument == "adminpw" )                     // Admin password setting?
  {
    adminpw = value ;                                   // Yes set admin password
  }
  else if ( argument == "wifipw" )                      // WiFi password setting?
  {
    sprintf ( AP_pw, value.c_str() ) ;                  // Yes set WiFi password
  }
  else if ( argument.startsWith ( "reset" ) )           // Reset request?
  {
    resetreq = true ;                                   // Reset all
  }
  else if ( argument == "model" )                       // Model command?
  {
    if ( value == "pijl" )                              // Ja, model is "pijl"?
    {
      model = PIJL ;                                    // Ja, set model
    }
    else                                                // Model is "stoplicht"
    {
      model = STOPLICHT ;                               // Ja, set model STOPLICHT
    }
  }
  else if ( argument == "lrrev" )                       // lrrev command?
  {
    if ( value == "links" )                             // Ja, l/R mode instellen op links?
    {
      lr = LINKS ;                                      // Ja, wijzigen
    }
    else                                                // Anders l/R mode instellen op rechts
    {
      lr = RECHTS ;                                     // Ja, wijzigen
    }
  } 
  else if ( argument == "test" )                        // Test command
  {
    dbgprint ( "Free memory is %d",                     // Print task info
               ESP.getFreeHeap() ) ;
    dbgprint ( "Stack playtask is %d",
               uxTaskGetStackHighWaterMark ( xplaytask ) ) ;
    dbgprint ( "Stack disptask is %d",
               uxTaskGetStackHighWaterMark ( xdisptask ) ) ;
    dbgprint ( "Errorcount is %d", errorcount ) ;
    dbgprint ( "WiFi re-connect count is %d", reconcount ) ;
    dbgprint ( "GPS satellieten: %d",                   // Aantal satellieten in gebruik
               gps_nsat ) ;
    dbgprint ( "GPS N is %s", gps_lat_s.c_str() ) ;     // Show position for test
    dbgprint ( "GPS E is %s", gps_lon_s.c_str() ) ;
    dbgprint ( "GPS tijd is %02d:%02d:%02d",            // And current time
               gps.time.hour(),
               gps.time.minute(),
               gps.time.second() ) ;
  }
  else
  {
    sprintf ( reply, "%s called with illegal parameter: %s",
              NAME, argument.c_str() ) ;
  }
  return reply ;                                        // Return reply to the caller
}


//**************************************************************************************************
//                                     H T T P H E A D E R                                         *
//**************************************************************************************************
// Set http headers to a string.                                                                   *
//**************************************************************************************************
String httpheader ( String contentstype )
{
  return String ( "HTTP/1.1 200 OK\nContent-type:" ) +
         contentstype +
         String ( "\n"
                  "Server: " NAME "\n"
                  "Cache-Control: " "max-age=3600\n"
                  "Last-Modified: " VERSION "\n\n" ) ;
}


//**************************************************************************************************
//                                     G E T C O N T E N T T Y P E                                 *
//**************************************************************************************************
// Returns the contenttype of a file to send.                                                      *
//**************************************************************************************************
String getContentType ( String filename )
{
  if      ( filename.endsWith ( ".html" ) ) return "text/html" ;
  else if ( filename.endsWith ( ".log"  ) ) return "text/plain" ;
  else if ( filename.endsWith ( ".csv"  ) ) return "text/plain" ;
  else if ( filename.endsWith ( ".png"  ) ) return "image/png" ;
  else if ( filename.endsWith ( ".gif"  ) ) return "image/gif" ;
  else if ( filename.endsWith ( ".jpg"  ) ) return "image/jpeg" ;
  else if ( filename.endsWith ( ".bmp"  ) ) return "image/bmp" ;
  else if ( filename.endsWith ( ".ico"  ) ) return "image/x-icon" ;
  else if ( filename.endsWith ( ".css"  ) ) return "text/css" ;
  else if ( filename.endsWith ( ".zip"  ) ) return "application/x-zip" ;
  else if ( filename.endsWith ( ".gz"   ) ) return "application/x-gzip" ;
  else if ( filename.endsWith ( ".mp3"  ) ) return "audio/mpeg" ;
  else if ( filename.endsWith ( ".js"   ) ) return "application/javascript" ;
  else if ( filename.endsWith ( ".pw"   ) ) return "" ;              // Passwords are secret
  return "text/html" ;
}


//**************************************************************************************************
//                                        C B  _ S Y S L O G                                       *
//**************************************************************************************************
// Callback function for handle_syslog, will be called for every chunk to send to client.          *
// If no more data is available, this function will return 0.                                      *
//**************************************************************************************************
size_t cb_syslog ( uint8_t *buffer, size_t maxLen, size_t index )
{
  static int   i ;                                  // Index in dbglines
  static int   nrl ;                                // Mumber of lines in dbglines
  static char  linebuf[DEBUG_BUFFER_SIZE + 20] ;    // Holds one debug line
  static char* p_in ;                               // Pointer in linebuf
  char*        p_out = (char*)buffer ;              // Fill pointer for output buffer
  String       s ;                                  // Single line from dbglines
  size_t       len = 0 ;                            // Number of bytes filled in buffer

  //Serial.printf ( "Syslog chunk of max %d characters\n",
  //                maxLen ) ;
  if ( index == 0 )                                 // First call for this page?
  {
    i = 0 ;                                         // Yes, set index
    nrl = dbglines.size() ;                         // Number of lines in dbglines
    p_in = linebuf ;                                // Set linebuf to empty
    *p_in = '\0' ;
  }
  while ( maxLen-- > 0 )                            // Space for another character?
  {
    if ( *p_in == '\0' )                            // Input buffer end?
    {
      if ( i == nrl )                               // Yes, is there another line?
      {
        break ;                                     // No, end of text
      }
      s = dbglines[i++] ;                           // Yes, get next line from container
      strcpy ( linebuf, s.c_str() ) ;               // Fill linebuf
      strcat ( linebuf, "\n" ) ;                    // Add a break
      p_in = linebuf ;                              // Pointer to start of line
    }
    *p_out++ = *p_in++ ;                            // Copy next character
    len++ ;                                         // Increase result length
  }
  // We come here if output buffer is completely full or if end of dbglines is reached
  //Serial.printf ( "Return syslog chunk, l is %d,"
  //                " line %d/%d\n",
  //                len, i, nrl ) ;
  return len ;                                      // Return filled length of buffer
}


//**************************************************************************************************
//                                    H A N D L E _ S Y S L O G                                    *
//**************************************************************************************************
// Called from syslog page to list the syslog in dbglines.                                         *
// It will handle the chunks for the client.  The buffer is filled by the callback routine.        *
//**************************************************************************************************
void handle_syslog ( AsyncWebServerRequest *request )
{
  AsyncWebServerResponse *response ;

  dbgprint ( "HTTP syslog request" ) ;
  if ( adminaccess ( request ) )                  // Check access
  {
    response = request->beginChunkedResponse ( "text/plain", cb_syslog ) ;
    response->addHeader ( "Server", NAME ) ;
    request->send ( response ) ;
  }
  else
  {
    request->send ( 200, "text/plain",            // Access not allowed
                    "No access" ) ;
  }
}


//**************************************************************************************************
//                                    H A N D L E _ R E G I S                                      *
//**************************************************************************************************
// Called from registraties page to list the registration files.                                   *
//**************************************************************************************************
void handle_regis ( AsyncWebServerRequest *request )
{
  File   root ;                                          // File handle root directory
  File   file ;                                          // File in root
  String reply ;                                         // Reply to client

  dbgprint ( "HTTP regis request" ) ;
  root = SPIFFS.open ( "/" ) ;                           // Open root directory
  while ( ( file = root.openNextFile() ) )               // Iterate through files
  {
    //if ( ! adminaccess ( request ) )
    //{
    //  break ;                                          // No access
    //}
    if ( strstr ( file.path(), ".csv" ) == nullptr )     // Is it a registration file?
    {
      continue ;                                         // No, continue
    }
    if ( ! reply.isEmpty() )                             // Separator needed?
    {
      reply += String ( "|" ) ;                          // Yes, add "|"
    }
    reply += String ( file.path() + 1 ) ;                // File name without "/"
  }
  request->send ( 200, "text/plain", reply ) ;           // Send to client
}


//**************************************************************************************************
//                                  H A N D L E _ E X P O R T                                      *
//**************************************************************************************************
// Called from registraties page to export a registration file.                                    *
//**************************************************************************************************
void handle_export ( AsyncWebServerRequest *request )
{
  String                 reply = "Bad request!" ;          // Reply to client
  String                 fnam ;                            // Value of first argument as String
  AsyncWebServerResponse *response ;                       // Response for client

  if ( useraccess ( request ) &&                           // Check access
       request->hasArg ( "file" ) )                        // File defined?
  {
    fnam = request->getParam(0)->value() ;                 // Get value of first argument (file name)
    dbgprint ( "HTTP export request for %s",
               fnam.c_str() ) ;
    if ( fnam.endsWith ( ".csv" ) )
    {
      response = request->beginResponse ( SPIFFS, fnam,    // Fill response
                                          String(), true ) ;
      request->send ( response ) ;                         // Send response
      return ;
    }
  }
  request->send ( 200, "text/plain", reply ) ;
}


//**************************************************************************************************
//                                    H A N D L E _ S T A T U S                                    *
//**************************************************************************************************
// Called from main page to list the current settings/data.                                        *
//**************************************************************************************************
void handle_status ( AsyncWebServerRequest *request )
{
  static char statusstr[80] ;                       // Will be status

  //dbgprint ( "HTTP status request" ) ;
  getTimeTxt() ;                                    // Get time into timetxt
  sprintf ( statusstr, "%s|%d|%d|%d|%s|%s|%s|%d|%d|%d|%d|%d|%d|%d|%d",
            timetxt,
            probability,
            volgnummer,
            trefnummer,
            treftijd,
            gps_lat_s.c_str(),
            gps_lon_s.c_str(),
            au_V_ned,
            au_T_ned,
            au_V_eng,
            au_T_eng,
            gain,
            n_brightness,
            model,
            lr ) ;
  //dbgprint ( "status %s", statusstr ) ;
  request->send ( 200, "text/plain", statusstr ) ;  // Send to client
}


//**************************************************************************************************
//                                    H A N D L E _ R E S E T                                      *
//**************************************************************************************************
// Called from config page to reset the CPU.                                                       *
//**************************************************************************************************
void handle_reset ( AsyncWebServerRequest *request )
{
  resetreq = adminaccess ( request ) ;                     // Set flag to reset CPU if user=admin
  dbgprint ( "HTTP reset request %d", resetreq ) ;
  request->send ( 200, "text/plain", "Command accepted" ) ;
}


//**************************************************************************************************
//                              H A N D L E _ S E T T I N G S                                      *
//**************************************************************************************************
// Called from main web page to set the audio/model options.                                       *
//**************************************************************************************************
void handle_settings ( AsyncWebServerRequest *request )
{
  String argn ;                                           // First key (=key in NVS)
  String argv ;                                           // Value of first argument as String
  String reply = "No access, log in!" ;

  argn = request->getParam(0)->name() ;                   // Get name of first argument, is key
  if ( useraccess ( request ) )                           // Check access
  {
    if ( ( argn.startsWith ( "au_" ) ) ||                 // Must be an audio command
         ( argn.startsWith ( "model" ) ) ||               // Or a "model" command
         ( argn.startsWith ( "lrrev" ) ) )                // Or a "lrrev" command
    {
      argv = request->getParam(0)->value() ;              // Get value of first argument
      dbgprint ( "HTTP, set %s = %s",                     // Show debug info
                 argn.c_str(), argv.c_str() ) ;
      nvssetstr ( argn.c_str(), argv ) ;                  // Save in NVS
      reply = analyzeCmd ( argn.c_str(), argv.c_str() ) ; // Activate setting
    }
    else
    {
      reply = String ( "Bad command!" ) ;                 // Unknown command
    }
  }
  request->send ( 200, "text/plain", reply ) ;
}


//**************************************************************************************************
//                                   H A N D L E _ L O G I N                                       *
//**************************************************************************************************
// Called from main web page to log in.                                                           *
//**************************************************************************************************
void handle_login ( AsyncWebServerRequest *request )
{
  String   argv0 ;                                       // Value of first argument as String
  String   argv1 ;                                       // Value of second argument as String
  uint32_t rip ;                                         // Remote IP (client)

  rip = request->client()->getRemoteAddress() ;          // Get remote IP
  argv0 = request->getParam(0)->value() ;                // Get value of first argument (user)
  argv1 = request->getParam(1)->value() ;                // Get value of second argument (password)
  //dbgprint ( "Log-in %s/%s/%s", argv0.c_str(),
  //           argv1.c_str(), adminpw.c_str() ) ; 
  if ( argv0 == "admin" )                                // Is it admin?
  {
    if ( ! adminaccess ( request ) )                     // Yes, need for log-in?
    {
      if ( argv1 == adminpw )                            // Yes, password okay?
      {
        lialist.push_back ( rip ) ;                      // Yes, add IP to logged-in admin list
        dbgprint ( "Admin logged in" ) ;                 // Show success
      }
    }
  }
  else                                                   // No admin.  User log-in
  {
    if ( ! useraccess ( request ) )                      // Yes, need for log-in?
    {
      if ( argv1 == userpw )                             // No, user. Paswsword okay?
      {
        liulist.push_back ( rip ) ;                      // Yes, add IP to logged-in user list
        dbgprint ( "User %s logged in",                  // Show success
                   argv0.c_str() ) ;
      }
    }
  }
  request->send ( 200 ) ;                                // Always send "okay"
}


//**************************************************************************************************
//                                   H A N D L E _ L O G O U T                                     *
//**************************************************************************************************
// Called from main web page to log out.                                                           *
//**************************************************************************************************
void handle_logout ( AsyncWebServerRequest *request )
{
  String   newpage = "/login.html" ;
  uint32_t rip ;                                              // Remote IP (client)

  rip = request->client()->getRemoteAddress() ;               // Get remote IP
  for ( int i = 0 ; i < lialist.size() ; i++ )                // Remove from admin list
  {
    if ( lialist[i] == rip )                                  // Match?
    {
      lialist[i] = 0 ;                                        // Yes, clear entry
    }
  }
  for ( int i = 0 ; i < liulist.size() ; i++ )                // Remove from user list
  {
    if ( liulist[i] == rip )                                  // Match?
    {
      liulist[i] = 0 ;                                        // Yes, clear entry
    }
  }
  request->send ( SPIFFS, String ( "/login.html" ),
                  getContentType ( newpage ) ) ;              // Send to client
}


//**************************************************************************************************
//                                    H A N D L E _ P R O B                                        *
//**************************************************************************************************
// Called from main web page to set the probability.                                               *
//**************************************************************************************************
void handle_prob ( AsyncWebServerRequest *request )
{
  String argt ;                                           // Value of first argument as String

  argt = request->getParam(0)->value() ;                  // Get value of first argument
  if ( useraccess ( request ) )                           // Check access
  {
    probability = argt.toInt() ;                          // Convert to int
    nvssetstr ( "trefkans", String ( probability ) ) ;
    openRegfile() ;                                       // Make sure registration file is open
    regfile.printf ( "P,%d\n", probability ) ;            // Add probability
    dbgprint ( "HTTP probability set to %d",
               probability ) ;
  }
  request->send ( 200, "text/plain", "Command accepted" ) ;
}


//**************************************************************************************************
//                              H A N D L E _ B R I G H T N E S S                                  *
//**************************************************************************************************
// Called from main web page to set the neopixel brightness.                                       *
//**************************************************************************************************
void handle_brightness ( AsyncWebServerRequest *request )
{
  String argt ;                                           // Value of first argument as String

  argt = request->getParam(0)->value() ;                  // Get value of first argument
  if ( useraccess ( request ) )                           // Check access
  {
    n_brightness = argt.toInt() ;                         // Convert to int
    nvssetstr ( "brightness", String ( n_brightness ) ) ;
    openRegfile() ;                                       // Make sure registration file is open
    dbgprint ( "HTTP brightness set to %d",
               n_brightness ) ;
  }
  request->send ( 200, "text/plain", "Command accepted" ) ;
}


//**************************************************************************************************
//                                  H A N D L E _ G A I N                                          *
//**************************************************************************************************
// Called from main web page to set the gain.                                                      *
//**************************************************************************************************
void handle_gain ( AsyncWebServerRequest *request )
{
  String argt ;                                           // Value of first argument as String

  argt = request->getParam(0)->value() ;                  // Get value of first argument
  if ( useraccess ( request ) )                           // Check access
  {
    gain = argt.toInt() ;                                 // Convert to int
    nvssetstr ( "gain", String ( gain ) ) ;
    openRegfile() ;                                       // Make sure registration file is open
    dbgprint ( "HTTP gain set to %d", gain ) ;
  }
  request->send ( 200, "text/plain", "Command accepted" ) ;
}


//**************************************************************************************************
//                                   F I N D N S I D                                               *
//**************************************************************************************************
// Find the namespace ID for the namespace passed as parameter.                                    *
//**************************************************************************************************
uint8_t FindNsID ( const char* ns )
{
  esp_err_t                 result = ESP_OK ;                 // Result of reading partition
  uint32_t                  offset = 0 ;                      // Offset in nvs partition
  uint8_t                   i ;                               // Index in Entry 0..125
  uint8_t                   bm ;                              // Bitmap for an entry
  uint8_t                   res = 0xFF ;                      // Function result

  while ( offset < nvs->size )
  {
    result = esp_partition_read ( nvs, offset,                // Read 1 page in nvs partition
                                  &nvsbuf,
                                  sizeof(nvsbuf) ) ;
    if ( result != ESP_OK )
    {
      dbgprint ( "Error reading NVS!" ) ;
      break ;
    }
    i = 0 ;
    while ( i < 126 )
    {
      bm = ( nvsbuf.Bitmap[i / 4] >> ( ( i % 4 ) * 2 ) ) ;    // Get bitmap for this entry,
      bm &= 0x03 ;                                            // 2 bits for one entry
      if ( bm != 2 )                                          // 3: free entry, 0: discarded
      {
        i++ ;
        continue ;
      }
      //dbgprint ( "bm is %d, i is %d, NS is %d", bm, i, nvsbuf.Entry[i].Ns ) ;
      //if ( ( nvsbuf.Entry[i].Ns == 0 ) || ( nvsbuf.Entry[i].Key[0] != 0 ) )
      //{
      //  dbgprint ( "Key is %s", nvsbuf.Entry[i].Key ) ;
      //}
      if ( ( bm == 2 ) &&
           ( nvsbuf.Entry[i].Ns == 0 ) &&
           ( strcmp ( ns, nvsbuf.Entry[i].Key ) == 0 ) )
      {
        res = nvsbuf.Entry[i].Data & 0xFF ;                   // Return the ID
        offset = nvs->size ;                                  // Stop outer loop as well
        break ;
      }
      else
      {
        if ( bm == 2 )
        {
          if ( nvsbuf.Entry[i].Span == 0 )                    // Cannot be zero
          {
            return res ;                                      // Return bad result
          }
          i += nvsbuf.Entry[i].Span ;                         // Next entry
        }
        else
        {
          i++ ;
        }
      }
    }
    offset += sizeof(nvs_page) ;                              // Prepare to read next page in nvs
  }
  return res ;
}


//**************************************************************************************************
//                                      N V S I N I T                                              *
//**************************************************************************************************
bool nvsinit()
{
  esp_err_t err = nvs_flash_init() ;
  
  if ( err == ESP_ERR_NVS_NO_FREE_PAGES ||
       err == ESP_ERR_NVS_NEW_VERSION_FOUND )
  {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    dbgprint ( "NVS flash init NOT okay!" ) ;
    err = nvs_flash_erase() ;
    if ( err != ESP_OK )
    {
      dbgprint ( "nvs_flash_erase error!" ) ;
    } 
    err = nvs_flash_init() ;
    if ( err != ESP_OK )
    {
      dbgprint ( "nvs_flash_init error!" ) ;
    } 
  }
  return ( err == ESP_OK ) ;
}


//**************************************************************************************************
//                                      N V S O P E N                                              *
//**************************************************************************************************
// Open Preferences with my-app namespace. Each application module, library, etc.                  *
// has to use namespace name to prevent key name collisions. We will open storage in               *
// RW-mode (second parameter has to be false).                                                     *
//**************************************************************************************************
bool nvsopen()
{
  bool res = true ;                                          // Assume correct NVS

  if ( ! nvshandle )                                         // Opened already?
  {
    nvserr = nvs_open ( NAME, NVS_READWRITE, &nvshandle ) ;  // No, open nvs
    if ( nvserr != ESP_OK )
    {
      dbgprint ( "nvs_open failed!" ) ;
      res = false ;                                          // Bad result
    }
    else
    {
      // Be sure there will be a valid namespace in NVS
      nvs_set_str ( nvshandle, "version", VERSION ) ;
      nvs_commit ( nvshandle ) ;
    }
  }
  return res ;                                               // Return result
}


//**************************************************************************************************
//                                      N V S C L O S E                                            *
//**************************************************************************************************
// Close Preferences with my-app namespace.                                                        *
//**************************************************************************************************
void nvsclose()
{
  if ( nvshandle )                                           // Opened already?
  {
    nvs_close ( nvshandle ) ;                                // Yes, close it
    nvshandle = 0 ;                                          // Handle invalid
  }
}


//**************************************************************************************************
//                                      N V S C L E A R                                            *
//**************************************************************************************************
// Clear all preferences.                                                                          *
//**************************************************************************************************
esp_err_t nvsclear()
{
  esp_err_t res ;                                     // Result erase

  nvsopen() ;                                         // Be sure to open nvs
  res = nvs_erase_all ( nvshandle ) ;                 // Clear all keys
  if ( res == ESP_OK )
  {
    res = nvs_commit ( nvshandle ) ;                  // Commit
  }
  nvsclose() ;                                        // Maybe best to close
  return res ;
}


//**************************************************************************************************
//                                      N V S G E T S T R                                          *
//**************************************************************************************************
// Read a string from nvs.                                                                         *
//**************************************************************************************************
String nvsgetstr ( const char* key )
{
  size_t        len = NVSBUFSIZE ;          // Max length of the string, later real length

  nvsopen() ;                               // Be sure to open nvs
  nvs_val[0] = '\0' ;                       // Return empty string on error
  nvserr = nvs_get_str ( nvshandle, key, nvs_val, &len ) ;
  if ( nvserr )
  {
    dbgprint ( "nvs_get_str failed %X for key %s, keylen is %d, len is %d!",
               nvserr, key, strlen ( key), len ) ;
    dbgprint ( "Contents: %s", nvs_val ) ;
  }
  return String ( nvs_val ) ;
}


//**************************************************************************************************
//                                      N V S S E T S T R                                          *
//**************************************************************************************************
// Put a key/value pair in nvs.  Length is limited to allow easy read-back.                        *
// No writing if no change.                                                                        *
//**************************************************************************************************
esp_err_t nvssetstr ( const char* key, String val )
{
  String curcont ;                                         // Current contents
  bool   wflag = true  ;                                   // Assume update or new key

  dbgprint ( "Setstring for %s: %s", key, val.c_str() ) ;
  if ( val.length() >= NVSBUFSIZE )                        // Limit length of string to store
  {
    dbgprint ( "nvssetstr length failed!" ) ;
    return ESP_ERR_NVS_NOT_ENOUGH_SPACE ;
  }
  if ( nvssearch ( key ) )                                 // Already in nvs?
  {
    curcont = nvsgetstr ( key ) ;                          // Read current value
    wflag = ( curcont != val ) ;                           // Value change?
  }
  if ( wflag )                                             // Update or new?
  {
    dbgprint ( "nvssetstr update value" ) ;
    nvserr = nvs_set_str ( nvshandle, key, val.c_str() ) ; // Store key and value
    if ( nvserr )                                          // Check error
    {
      dbgprint ( "nvssetstr failed!" ) ;
    }
    else
    {
      nvs_commit ( nvshandle ) ;
    }
  }
  return nvserr ;
}


//**************************************************************************************************
//                                      N V S C H K E Y                                            *
//**************************************************************************************************
// Change a keyname in in nvs.                                                                     *
//**************************************************************************************************
void nvschkey ( const char* oldk, const char* newk )
{
  String curcont ;                                         // Current contents

  if ( nvssearch ( oldk ) )                                // Old key in nvs?
  {
    curcont = nvsgetstr ( oldk ) ;                         // Read current value
    nvs_erase_key ( nvshandle, oldk ) ;                    // Remove key
    nvssetstr ( newk, curcont ) ;                          // Insert new
    nvs_commit ( nvshandle ) ;
  }
}


//**************************************************************************************************
//                                      N V S S E A R C H                                          *
//**************************************************************************************************
// Check if key exists in nvs.                                                                     *
//**************************************************************************************************
bool nvssearch ( const char* key )
{
  size_t        len = NVSBUFSIZE ;                      // Length of the string

  nvsopen() ;                                           // Be sure to open nvs
  nvserr = nvs_get_str ( nvshandle, key, NULL, &len ) ; // Get length of contents
  //dbgprint ( "nvssearch for key %s, result %d",
  //           key, nvserr) ;
  return ( nvserr == ESP_OK ) ;                         // Return true if found
}


//**************************************************************************************************
//                                     G E T E N C R Y P T I O N T Y P E                           *
//**************************************************************************************************
// Read the encryption type of the network and return as a 4 byte name                             *
//**************************************************************************************************
const char* getEncryptionType ( wifi_auth_mode_t thisType )
{
  switch ( thisType )
  {
    case WIFI_AUTH_OPEN:
      return "OPEN" ;
    case WIFI_AUTH_WEP:
      return "WEP" ;
    case WIFI_AUTH_WPA_PSK:
      return "WPA_PSK" ;
    case WIFI_AUTH_WPA2_PSK:
      return "WPA2_PSK" ;
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "WPA_WPA2_PSK" ;
    case WIFI_AUTH_MAX:
      return "MAX" ;
    default:
      break ;
  }
  return "????" ;
}


//**************************************************************************************************
//                                        L I S T N E T W O R K S                                  *
//**************************************************************************************************
// List the available networks.                                                                    *
// Acceptable networks are those who have an entry in the preferences.                             *
// SSIDs of available networks will be saved for use in webinterface.                              *
//**************************************************************************************************
void listNetworks()
{
  WifiInfo_t       winfo ;            // Entry from wifilist
  wifi_auth_mode_t encryption ;       // TKIP(WPA), WEP, etc.
  const char*      acceptable ;       // Netwerk is acceptable for connection
  int              i, j ;             // Loop control

  dbgprint ( "Scan Networks" ) ;                         // Scan for nearby networks
  numSsid = WiFi.scanNetworks() ;
  dbgprint ( "WiFi scan completed" ) ;
  if ( numSsid <= 0 )
  {
    dbgprint ( "Couldn't get a wifi connection" ) ;
    return ;
  }
  // print the list of networks seen:
  dbgprint ( "Number of available networks: %d",
             numSsid ) ;
  // Print the network number and name for each network found and
  for ( i = 0 ; i < numSsid ; i++ )
  {
    acceptable = "" ;                                    // Assume not acceptable
    for ( j = 0 ; j < wifilist.size() ; j++ )            // Search in wifilist
    {
      winfo = wifilist[j] ;                              // Get one entry
      if ( WiFi.SSID(i).indexOf ( winfo.ssid ) == 0 )    // Is this SSID acceptable?
      {
        acceptable = "Acceptable" ;
        break ;
      }
    }
    encryption = WiFi.encryptionType ( i ) ;
    dbgprint ( "%2d - %-25s Signal: %3d dBm, Encryption %4s, %s",
               i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i),
               getEncryptionType ( encryption ),
               acceptable ) ;
    // Remember this network for later use
    networks += WiFi.SSID(i) + String ( "|" ) ;
  }
  dbgprint ( "End of list" ) ;
}


//**************************************************************************************************
//                                           M K _ L S A N                                         *
//**************************************************************************************************
// Make al list of acceptable networks in preferences.                                             *
// Will be called only once by setup().                                                            *
// The result will be stored in wifilist.                                                          *
// Not that the last found SSID and password are kept in common data.  If only one SSID is         *
// defined, the connect is made without using wifiMulti.  In this case a connection will           *
// be made even if de SSID is hidden.                                                              *
//**************************************************************************************************
void  mk_lsan()
{
  uint8_t     i ;                                        // Loop control
  char        key[10] ;                                  // For example: "wifi_03"
  String      buf ;                                      // "SSID/password"
  String      lssid, lpw ;                               // Last read SSID and password from nvs
  int         inx ;                                      // Place of "/"
  WifiInfo_t  winfo ;                                    // Element to store in list

  dbgprint ( "Create list with acceptable WiFi networks" ) ;
  for ( i = 0 ; i < 20 ; i++ )                           // Examine wifi_00 .. wifi_19
  {
    sprintf ( key, "wifi_%02d", i ) ;                    // Form key in preferences
    if ( nvssearch ( key  ) )                            // Does it exists?
    {
      buf = nvsgetstr ( key ) ;                          // Get the contents
      inx = buf.indexOf ( "/" ) ;                        // Find separator between ssid and password
      if ( inx > 0 )                                     // Separator found?
      {
        lpw = buf.substring ( inx + 1 ) ;                // Isolate password
        lssid = buf.substring ( 0, inx ) ;               // Holds SSID now
        dbgprint ( "Added %s to list of networks",
                   lssid.c_str() ) ;
        winfo.inx = i ;                                  // Create new element for wifilist ;
        winfo.ssid = strdup ( lssid.c_str() ) ;          // Set ssid of element
        winfo.passphrase = strdup ( lpw.c_str() ) ;
        wifilist.push_back ( winfo ) ;                   // Add to list
        wifiMulti.addAP ( winfo.ssid,                    // Add to wifi acceptable network list
                          winfo.passphrase ) ;
      }
    }
  }
}


//**************************************************************************************************
//                                  W I F I _ E V E N T                                            *
//**************************************************************************************************
// Print WiFi events voor test.                                                                    *
// Set or reset "wifiok".                                                                          *
//**************************************************************************************************
void wifi_event ( WiFiEvent_t event )
{
  const char* p = "WiFi event" ;            // Default debug string

  switch(event)
  {
    case ARDUINO_EVENT_WIFI_READY:
      p = "WiFI ready" ;
      wifiok = true ;                       // WiFi is okay
      break ;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
      p= "SYSTEM_EVENT_SCAN_DONE" ;
      break ;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      p = "SYSTEM_EVENT_STA_CONNECTED" ;
      wifiok = true ;                       // WiFi is okay
      break ;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      p = "SYSTEM_EVENT_STA_DISCONNECTED" ;
      wifiok = false ;                     // WiFi is not okay amymore
      break ;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      p = "STA_GOT_IP" ;
      wifiok = true ;                      // WiFi is okay
      break ;
    case ARDUINO_EVENT_WIFI_STA_START:
      p = "STA_START" ;
      break ;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      p = "STA_STOP" ;
      break ;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      p = "AP_STA IP ASSIGNED" ;
      break ;
    case ARDUINO_EVENT_WIFI_AP_START:
      p = "AP started." ;
      wifiok = true ;                       // WiFi is okay
      break ;
    case ARDUINO_EVENT_WIFI_AP_STOP:
      p = "AP Stopped" ;
      wifiok = false ;                      // WiFi is not okay amymore
      break ;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      p = "WiFi Client Connected" ;
      wifiok = true ;                       // WiFi is okay
      break ;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      p = "WiFi Client Disconnected" ;
      wifiok = false ;                      // WiFi is not okay amymore
      break ;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      p = "WiFi Lost IP" ;
      wifiok = false ;                     // WiFi is not okay amymore
      break ;
    default:
      dbgprint ( "Unhandled WiFi Event: %d", event ) ;
      return ;
  }
  dbgprint ( p ) ;
}


//**************************************************************************************************
//                                       C O N N E C T W I F I                                     *
//**************************************************************************************************
// Connect to WiFi using the SSID's available in wifiMulti.                                        *
// If only one AP if found in preferences (i.e. wifi_00) the connection is made without            *
// using wifiMulti.                                                                                *
// If connection fails, an AP is created and the function returns false.                           *
//**************************************************************************************************
bool connectwifi()
{
  bool       localAP = false ;                          // True if only local AP is left
  WifiInfo_t winfo ;                                    // Entry from wifilist
  String     ipaddress ;                                // Own IP-address

  reconcount++ ;                                        // Count number of (re)connections
  if ( wifilist.size()  )                               // Any AP defined?
  {
    WiFi.mode ( WIFI_STA ) ;                            // ESP32 is een "station"
    WiFi.disconnect() ;                                 // Voorkom router probleem
    vTaskDelay ( 100 / portTICK_PERIOD_MS ) ;           // 100 ms delay
    if ( wifilist.size() == 1 )                         // Just one AP defined in preferences?
    {
      winfo = wifilist[0] ;                             // Get this entry
      WiFi.begin ( winfo.ssid, winfo.passphrase ) ;     // Connect to single SSID found in wifi_xx
      dbgprint ( "Try WiFi %s", winfo.ssid ) ;          // Message to show during WiFi connect
    }
    else                                                // More AP to try
    {
      wifiMulti.run() ;                                 // Connect to best network
    }
    if (  WiFi.waitForConnectResult() != WL_CONNECTED ) // Try to connect
    {
      localAP = true ;                                  // Error, setup own AP
    }
  }
  else
  {
    dbgprint ( "No known WiFi networks!" ) ;
    localAP = true ;                                    // Not even a single AP defined
  }
  if ( localAP )                                        // Must setup local AP?
  {
    dbgprint ( "WiFi Failed!  Trying to setup AP "
               "with name %s and password %s.",
               AP_SSIDstr, AP_pw ) ;
    WiFi.mode ( WIFI_AP ) ;                             // ESP32 wordt een access point
    vTaskDelay ( 100 / portTICK_PERIOD_MS ) ;           // 100 ms delay
    WiFi.softAP ( AP_SSIDstr, AP_SSIDstr ) ;            // This ESP will be an AP
    ip = WiFi.softAPIP() ;                              // Get AP IP address
  }
  else
  {
    ip = WiFi.localIP() ;                               // GET STA IP address
  }
  sprintf ( ipstr, "IP=%s",                             // String to display for debug
            ip.toString().c_str() ) ;
  dbgprint ( ipstr ) ;                                  // Address for AP
  return ( localAP == false ) ;                         // Return result of connection
}


//**************************************************************************************************
//                                    I N I T P R E F S                                            *
//**************************************************************************************************
// Vul NVS met defaults voor de configureerbare parameters.                                        *
// Dit gebeurt als er helemaal niets in NVS zit.                                                   *
//**************************************************************************************************
void initprefs()
{
  nvsclear() ;                                                          // Erase all keys
  nvssetstr ( "wifi_00",      "ssid/password"                      ) ;
}


//**************************************************************************************************
//                                    H A N D L E _ G E T P R E F S                                *
//**************************************************************************************************
// Called from config page to display configuration data.                                          *
//**************************************************************************************************
void handle_getprefs ( AsyncWebServerRequest *request )
{
  String prefs ;
  
  dbgprint ( "HTTP get preferences" ) ;                // Show request
  if ( adminaccess ( request ) )
  {
    prefs = readprefs ( true ) ;                       // Read preference values
  }
  request->send ( 200, "text/plain", prefs ) ;         // Send the reply
}


//**************************************************************************************************
//                                    H A N D L E _ S A V E P R E F S                              *
//**************************************************************************************************
// Called from config page to save configuration data.                                             *
//**************************************************************************************************
void handle_saveprefs ( AsyncWebServerRequest *request )
{
  String reply = "Configuratie opgeslagen" ;           // Default reply
  
  dbgprint ( "HTTP save preferences" ) ;               // Show request
  if ( adminaccess ( request ) )
  {
    writeprefs ( request ) ;                           // Write to NVS
  }
  request->send ( 200, "text/plain", reply ) ;         // Send the reply
}


//**************************************************************************************************
//                                    H A N D L E _ M T S Y N C                                    *
//**************************************************************************************************
// Called from config page to synhronize the RTC.                                                  *
// example:  POST /mtsync?t=1589191159                                                             *
//**************************************************************************************************
void handle_mtsync ( AsyncWebServerRequest *request )
{
  String      argt ;                                    // Value of first argument
  //uint32_t    c_Epoch32OfOriginYear = 946684800 ;       // Seconds betweeen 1970 and 2000
  uint32_t    rtcnow ;                                  // Time from RTC, sec. since 2000-01-01
  uint32_t    sec ;                                     // Seconds since Epoch
  uint32_t    delta ;                                   // Time difference
  String      res ;                                     // Result of setting
  char        str[70] ;                                 // For formatting result

  argt = request->getParam(0)->value() ;                // Get value of first argument
  sec = argt.toInt() ;                                  // Convert to int
  rtcnow = getRtcTime() ;                               // Get RTC time, seconds since 1-1-1970
  dbgprint ( "HTTP clock sync to %d", sec  ) ;          // Show request
  dbgprint ( "RTCnow is %d", rtcnow  ) ;                // Show current RTC time
  if ( useraccess ( request ) )                         // Only if logged-in
  {
    setRtcTime ( sec ) ;                                // Set new RTC time
  }
  //sec -= c_Epoch32OfOriginYear ;                      // Adjust for Epoch (1970)
  if ( sec > rtcnow )
  {
    delta = sec - rtcnow ;                              // Compute clock off time
  }
  else
  {
    delta = rtcnow - sec ;
  }
  getRtcTime() ;                                        // Read back and set timetxt
  sprintf ( str, "RTC synced to "
            "%s, "                                      // Show new new time
            "delta is %u seconds",
            timetxt,
            delta ) ;
  res = String ( str ) ;                                // Use for reply
  dbgprint ( str ) ;                                    // Also in debug log
  request->send ( 200, "text/plain", res ) ;            // Send the reply
}


//******************************************************************************
//                              H E X                                          *
//******************************************************************************
// Convert 1 character (0..9, A..F, a..f) to its hexadecimal value.            *
//******************************************************************************
uint8_t hex ( char h )
{
  uint8_t res = 0x0F ;                    // Function result
  
  if ( h >= '0' && h <= '9' )             // Easy conversion for '0'...'9'
  {
    res = h - '0' ;
  }
  else if ( h >= 'A' && h <= 'F' )        // Conversion for 'A'...'f'
  {
    res = h - 'A' + 10 ;
  }
  else if ( h >= 'a' && h <= 'f' )        // Conversion for 'a'...'f'
  {
    res = h - 'a' + 10 ;
  }
  return res ;                            // Return the result
}


//**************************************************************************************************
//                                           A 8 T O H E X                                         *
//**************************************************************************************************
// Convert a string of 8 haxadecimal characters (0..9, A..F, a..f) to its                          *
// hexadecimal value.                                                                              *
//**************************************************************************************************
uint32_t a8tohex ( const char* hstr )
{
  uint8_t  i ;                                             // Loop control
  uint32_t res = 0 ;                                       // 32 bits result
  
  for ( i = 0 ; i < 8 ; i++ )                              // Handle 8 nibbles
  {
    res = ( res << 4 ) | hex ( *hstr++ ) ;                 // Shift next four bits in
  }
  return res ;
}


//**************************************************************************************************
//                                           A 2 T O H E X                                         *
//**************************************************************************************************
// Convert a string of 2 haxadecimal characters (0..9, A..F, a..f) to its                          *
// hexadecimal value.                                                                              *
//**************************************************************************************************
uint32_t a2tohex ( const char* hstr )
{
  uint8_t  i ;                                             // Loop control
  uint32_t res = 0 ;                                       // 32 bits result
  
  for ( i = 0 ; i < 2 ; i++ )                              // Handle 2 nibbles
  {
    res = ( res << 4 ) | hex ( *hstr++ ) ;                 // Shift next four bits in
  }
  return res ;
}


//**************************************************************************************************
//                                   S H O W A L L F I L E S                                       *
//**************************************************************************************************
// Show files in the file system.  Only files in root-directory are printed.                       *
//**************************************************************************************************
void showAllFiles()
{
  File root = SPIFFS.open ( "/" ) ;                      // Open root directory
  File file ;                                            // File in directory

  dbgprint ( "SPIFFS contents:") ;
  while ( ( file = root.openNextFile() ) )               // Iterate through files
  {
    dbgprint ( "%5d %s", file.size(),                    // Show file info
               file.path() ) ;                           // File name with "/"
  }
}


//**************************************************************************************************
//                                       I S R _ B U T T O N                                       *
//**************************************************************************************************
// Will be called if the button is pushed (falling edge).                                          *
//**************************************************************************************************
void IRAM_ATTR ISR_button()
{
  buttonreq = true ;                              // Set request flag
}


//**************************************************************************************************
//                                     P L A Y T E X T S                                           *
//**************************************************************************************************
// Genrate MP3 output.  Sequence number and text in 0, 1 or 2 languages.                           *
// Parameter is "hit" or no hit.                                                                   *
//**************************************************************************************************
void playtexts ( bool hit )
{
  qdata_struct  qentry ;                                // Data to queue
  bool          startout = false ;
  int           v ;                                     // Volgnr modulo 100000..10
  int           divider ;                               // Divider to get nextdigit
  int           digit ;
  const char*   lang = "ne" ;                           // Available languages
  int           i ;                                     // Index inb lang
  bool          doplay ;                                // Play or not
  char          langstr[]= "_x.mp3" ;                   // Language as a string

  for ( i = 0 ; i < 2 ; i++ )                           // 2 loops for 2 languages
  {
    langstr[1] = lang[i] ;                              // Make language string
    // First handle the sequence number
    doplay  = ( lang[i] == 'n' ) && ( au_V_ned ) ;      // Play dutch?
    doplay |= ( lang[i] == 'e' ) && ( au_V_eng ) ;      // Play english?
    if ( doplay )
    {
      strcpy ( qentry.filename, "/cijfers/0_e.mp3" ) ;  // Filename of sound to play
      qentry.pause = 0 ;                                // Normally no pause
      qentry.filename[11] = lang[i] ;                   // Digit to play
      v = volgnummer ;                                  // Set sequence number
      divider = 100000 ;                                // Set first devider
      startout = false ;                                // Wait for first nonzero digit
      while ( divider )
      {
        digit = v / divider ;
        startout |= ( digit > 0 ) ;
        if ( startout )
        {
          qentry.filename[9] = (char)(digit + '0' ) ;
          if ( divider == 10  )                         // Last digit?
          {
            qentry.pause = 500 ;                        // Pause after last digit
          }
          xQueueSend ( playqueue, &qentry, 200 ) ;      // Send to queue
        }
        v %= divider ;
        divider /= 10 ;                                 // For next digit
      }
    }
    // Now the text
    doplay  = ( lang[i] == 'n' ) && ( au_T_ned ) ;      // Play dutch?
    doplay |= ( lang[i] == 'e' ) && ( au_T_eng ) ;      // Play english?
    if ( doplay )
    {
      strcpy ( qentry.filename, "/audio/" ) ;           // Directory of sound to play
      if ( model == PIJL )                              // Sound depends on model
      {
        strcat ( qentry.filename, "pijl/" ) ;           // Sound from "pijl" subdirectory
      }
      else
      {
        strcat ( qentry.filename, "stoplicht/" ) ;      // Sound from "pijl" subdirectory
      }
      if ( hit )
      {
        strcat ( qentry.filename, "treffer" ) ;         // Partial filename of sound to play
      }
      else
      {
        strcat ( qentry.filename, "doorgaan" ) ;        // Rartial filename of sound to play
      }
      strcat ( qentry.filename, langstr ) ;             // Add language and ".mp3"
      qentry.pause = 500 ;                              // Pause after text
      xQueueSend ( playqueue, &qentry, 200 ) ;          // Send to queue
    }
  }
}


//**************************************************************************************************
//                                     H A N D L E _ B U T T O N                                   *
//**************************************************************************************************
// Handle the big button.                                                                          *
//**************************************************************************************************
void handle_button()
{
  bool             hit ;                          // True if ...
  qdisp_struct     dispdata ;                     // Queue element for disptask

  dbgprint ( "Button pushed" ) ;
  volgnummer++ ;                                  // Update sequence number
  hit = ( random ( 100 ) < probability ) ;        // Hit if value between 0 and 99 less than p
  if ( hit )                                      // Hit?
  {
    trefnummer = volgnummer ;                     // Yes, save ID for registration/web interface
    dbgprint ( "Sequence nr %d",
               trefnummer ) ;
    strcpy ( treftijd, timetxt ) ;                // Remember timestamp of hit
    openRegfile() ;                               // Make sure regfile is open
    regfile.printf ( "H,%d,%s\n",                 // Log hit
                     trefnummer, treftijd ) ;
    regfile.flush() ;                             // Make sure output is written
  }
  playtexts ( hit ) ;                             // Generate mp3 output
  dispdata.hit = hit ;                            // Fill queue element
  dispdata.volgnummer = volgnummer ;
  xQueueSend ( dispqueue, &dispdata, 200 ) ;      // Send to queue
}
  


//**************************************************************************************************
// Functions for disptask.                                                                         *
//**************************************************************************************************

//**************************************************************************************************
//                                       S H O W C H A R                                           *
//**************************************************************************************************
// Show a digit (or special character) on the display.  Pos is the position ( 0..NEO_PANEL-1 ).    *
// dig is the digit(0..9) for decimal digits. C_X shows "X", C_DOWN shows "V", C_UP shows "^",     *
// C_RIGHT shows ">" and C_LEFT shows "<".                                                         *
// tcolor is the text color, br is the brightness (0..100).                                        *
// Characters are separated by 2 black pixels                                                      *
//**************************************************************************************************
void showchar ( uint8_t pos, uint8_t dig, RgbColor &tcolor, uint8_t br )
{
  uint64_t patt  = charset[dig] ;                         // Get pattern for this digit
  uint64_t mask  = 0x8000000000000000 ;
  uint16_t pix   = pos * 80 ;                             // Pixel to change
  RgbColor tc    = tcolor.Dim ( br * 255 / 100 ) ;        // Scale text color for brightness

  if ( pos >= 3 )                                         // Backside panel (3..6)?
  {
    pix += 16 ;                                           // Yes, correction backside
  }
  while ( mask )                                          // Loop for 64 bits
  {
    if ( patt & mask )                                    // Bit set?
    {
      neodisp.SetPixelColor ( pix, tc ) ;                 // Yes, apply text color
    }
    else
    {
      neodisp.SetPixelColor ( pix, n_black ) ;           // No, set to background color
    }
    mask >>= 1 ;                                         // Shift mask
    pix++ ;                                              // Next pixel
  }
  neodisp.Show() ;                                       // Output pattern
}


//**************************************************************************************************
//                                 S H O W N E O B U T T O N                                       *
//**************************************************************************************************
// Show an iconic button on the frontside of the panel.                                            *
// The top 2 slots (160 pixels) are reserver for the down arrow (animated), the 3rd slot, pixels   *
// 160..239 will be filled by animated down arrow.                                                 *
//**************************************************************************************************
void showNeoButton()
{
  RgbColor  color = RgbColor(128) ;               // Color to display, mostly grey
  RgbColor  bcolor = n_red ;
  RgbColor  rc ;                                  // Scaled red
  uint16_t  pix ;                                 // Pixel to change color to red

  showchar ( 2, C_BUT, color, n_brightness ) ;    // Show button in grey
  rc = bcolor.Dim ( n_brightness * 255 / 100 ) ;  // Scale red color for brightness
  for ( pix = 160 ; pix < 192 ; pix++ )           // Change color to red upper half
  {
    if ( neodisp.GetPixelColor ( pix ) != 0 )     // Only if not black
    {
      neodisp.SetPixelColor ( pix, rc ) ;         // Change one pixel from grey to red
    }
  }
  showchar ( 0, C_DOWN, n_white, n_brightness ) ; // Show arrow at top
  showchar ( 1, C_DOWN, n_white, 0 ) ;            // And blank in middle slot
}


//**************************************************************************************************
//                                 A N I M A T E A R R O W                                         *
//**************************************************************************************************
// Animate the top 2 slots of the frontside panel.                                                 *
//**************************************************************************************************
void animateArrow()
{
  neodisp.RotateRight ( 8, 0, 159 ) ;              // Rotate 16 rows
  neodisp.Show() ;
}


//**************************************************************************************************
//                                     S H O W N E O N U M                                         *
//**************************************************************************************************
// Show a 3 digit number on 3 backside panels.                                                     *
//**************************************************************************************************
void showNeoNum ( int v, RgbColor tcolor )
{
  for ( uint8_t i = 5 ; i >= 3 ; i-- )       // Display on 3 panels: 5, 4, 3
  {
    showchar ( i, v % 10,                    // Show red/green character
               tcolor, n_brightness ) ;
    v = v / 10 ;                             // Next digit
    if ( v == 0 )                            // Is next digit significant?
    {
      tcolor = n_black ;                     // No, force invisible character
    }
  }
}


//**************************************************************************************************
//                                     S H O W N E O S Y M                                         *
//**************************************************************************************************
// Show hit or no-hit symbol on 3 panels.                                                          *
//**************************************************************************************************
void showNeoSym ( bool hit )
{

  RgbColor          color = n_white ;                           // Color to display, assume white
  uint8_t           symbol ;                                    // Symbol to display

  if ( hit )                                                    // A hit?
  {
    color = n_red ;                                             // Color will be red
    if ( model == PIJL )                                        // Yes, PIJL model?
    {
      if ( lr == RECHTS )                                       // Right out on no-hit?
      {
        symbol = C_LEFT ;                                       // Yes, symbol will be "<"
      }
      else
      {
        symbol = C_RIGHT ;                                      // right out on hit
      }
    }
    else
    {
      symbol = C_X ;                                            // STOPLICHT, show cross
    }
  }
  else                                                           // No hit
  {
    color = n_green ;                                           // Color will be green
    if ( model == PIJL )                                        // No hit, mode is PIJL?
    {
      if ( lr == RECHTS )                                       // Right out on no-hit?
      {
        symbol = C_RIGHT ;                                      // Yes, symbol will be ">"
      }
      else
      {
        symbol = C_LEFT ;                                       // Left out on no hit
      }
    }
    else
    {
      symbol = C_V ;                                            // STOPLICHT, show "V"
    }
  }
  for ( uint8_t i = 0 ; i < 3  ; i++ )                          // Display on 3 front panels
  {
    showchar ( i, symbol, color, n_brightness ) ;               // Show red/green character
  }
}


//**************************************************************************************************
//                                     D I S P T A S K                                             *
//**************************************************************************************************
// Handle the diplay on the 3 Neopixel panels.                                                     *
//**************************************************************************************************
void disptask ( void * parameter )
{
  qdisp_struct      dispdata ;                                  // Data from queue
  int               v = 0 ;                                     // Sequence number
  RgbColor          color ;                                     // Color to display, assume green
  uint8_t           resetpwcount = 0 ;                          // Counter for password reset function
  bool              newbutton = true ;                          // Button drawing or not

  dbgprint ( "Starting neopixel displaytask.." ) ;
  neodisp.Begin() ;                                             // Initialize NeoPixel
  neodisp.Show() ;                                              // Output blank pattern
  // Show IP-address on backside display
  for ( int i = 0 ; i < 4 ; i++ )
  {
    showNeoNum ( ip[i], n_blue ) ;                              // Show next byte of IP-address
    vTaskDelay ( 2000 / portTICK_PERIOD_MS ) ;                  // For 2 seconds
    showNeoNum ( 0, n_black ) ;                                 // Blank display
    vTaskDelay ( 200 / portTICK_PERIOD_MS ) ;                   // Blank for short time
    if ( ( digitalRead ( BUTTON0 ) == LOW ) ||                  // Check button(s)
         ( digitalRead ( BUTTON1 ) == LOW ) )
    {
       resetpwcount++ ;                                         // Count number of times
    }
  }
  if ( resetpwcount >= 3 )                                      // Seen button long enough?
  {
    pwresetreq = true ;                                         // Yes, set pw reset request
  }
  dsptaskrdy = true ;                                           // Task is ready for actions
  neodisp.ClearTo ( 0 ) ;                                       // Blank screen
  neodisp.Show() ;
  while ( true )
  {
    if ( xQueueReceive ( dispqueue, &dispdata, 100 ) )          // New display request?
    {
      neodisp.ClearTo ( 0 ) ;                                   // Clear display
      showNeoSym ( dispdata.hit ) ;                             // Yes, show hit/no hit
      color = n_green ;                                         // Color to display, assume green
      if ( dispdata.hit )                                       // Was it a hit?
      {
        color = n_red ;                                         // Yes, color will be red
      }
      else
      {
        color = n_green ;                                       // No, color will be green
      }
      v = dispdata.volgnummer ;                                 // Sequence number
      showNeoNum ( v, color ) ;                                 // Show sequence at backside panel
      vTaskDelay ( 5000 / portTICK_PERIOD_MS ) ;                // Active for 5 second
      newbutton = true ;                                        // Activate button icon on front panel
    }
    else                                                        // No new request
    {                                                           // Inactive
      if ( newbutton )                                          // Need to show button?
      {
        showNeoButton() ;                                       // Show button on front panel
        newbutton = false ;                                     // Next loop: just animate
      }
      else
      {
        animateArrow() ;                                        // Scroll the downarrow
      }
    }
  }
}

//**************************************************************************************************
//                                  P L A Y T A S K I 2 S                                          *
//**************************************************************************************************
// Play stream data from input queue. Version for I2S output or output to internal DAC.            *
// I2S output is suitable for a PCM5102A DAC.                                                      *
// Internal ESP32 DAC (pin 25 and 26) is used when no pin BCK is configured.                       *
// Note that the naming of the data pin is somewhat confusing.  The data out pin in the pin        *
// configuration is called data_out_num, but this pin should be connected to the "DIN" pin of the  *
// external DAC.  The variable used to configure this pin is therefore called "i2s_din_pin".       *
// If no pin for i2s_bck is configured, output will be sent to the internal DAC.                   *
// Task will stop on OTA update.                                                                   *
//**************************************************************************************************
void playtaskI2S ( void * parameter )
{
  esp_err_t      pinss_err ;                                        // Result of i2s_set_pin
  i2s_port_t     i2s_num = I2S_NUM_0 ;                              // i2S port number
  i2s_config_t   i2s_config ;                                       // I2S configuration
  qdata_struct   playdata ;                                         // Data from queue
  File           fp ;                                               // MP3 input file
  size_t         flen ;                                             // Length of MP3 file
  size_t         alocspace = 0 ;                                    // Allocated buffer space

  vTaskDelay ( 3000 / portTICK_PERIOD_MS ) ;                        // Start delay
  dbgprint ( "Starting I2S playtask.." ) ;
  MP3Decoder_AllocateBuffers() ;                                    // Init HELIX buffers
  i2s_config.mode                   = (i2s_mode_t)(I2S_MODE_MASTER | // I2S mode (5)
                                          I2S_MODE_TX) ;
  i2s_config.sample_rate            = 44100 ;
  i2s_config.bits_per_sample        = I2S_BITS_PER_SAMPLE_16BIT ;    // (16)
  i2s_config.channel_format         = I2S_CHANNEL_FMT_RIGHT_LEFT ;   // (0)
  #if ESP_ARDUINO_VERSION_MAJOR >= 2                                 // New version?
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S ;    // Yes, use new definition
  #else
    i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB) ;
  #endif
  i2s_config.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1 ;         // High interrupt priority
  i2s_config.dma_buf_count        = 16,
  i2s_config.dma_buf_len          = 64,
  i2s_config.use_apll             = false ;
  i2s_config.tx_desc_auto_clear   = false ;                        // No clear tx descriptor on underflow
  i2s_config.fixed_mclk           = I2S_PIN_NO_CHANGE ;            // No pin for MCLK
  //i2s_config.mclk_multiple        = (i2s_mclk_multiple_t)0 ;
  //i2s_config.bits_per_chan        = (i2s_bits_per_chan_t)0 ;
  #ifdef INTERNALDAC
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER |               // Set I2S mode for internal DAC
                                   I2S_MODE_TX |                   // (4)
                                   I2S_MODE_DAC_BUILT_IN ) ;       // Enable internal DAC (16)
    #if ESP_ARDUINO_VERSION_MAJOR < 2
      i2s_config.communication_format = I2S_COMM_FORMAT_I2S_MSB ;
    #endif
  #endif
  dbgprint ( "Starting I2S playtask.." ) ;
  MP3Decoder_AllocateBuffers() ;                                   // Init HELIX buffers
  if ( i2s_driver_install ( i2s_num, &i2s_config, 0, NULL ) != ESP_OK )
  {
    dbgprint ( "I2S install error!" ) ;
  }
  #ifdef INTERNALDAC                                               // Use internal (8 bit) DAC?
    dbgprint ( "Output to internal DAC" ) ;                        // Show output device
    pinss_err = i2s_set_pin ( i2s_num, NULL ) ;                    // Yes, default pins for internal DAC
    i2s_set_dac_mode ( I2S_DAC_CHANNEL_BOTH_EN ) ;
  #else
    i2s_pin_config_t pin_config ;                                  // I2s pin config
    pin_config.bck_io_num    = I2S_BCK_PIN ;                       // This is BCK pin
    pin_config.ws_io_num     = I2S_LCK_PIN ;                       // This is L(R)CK pin
    pin_config.data_out_num  = I2S_DIN_PIN ;                       // This is DATA output pin
    pin_config.data_in_num   = I2S_PIN_NO_CHANGE ;                 // No input
    #if ESP_ARDUINO_VERSION_MAJOR >= 2
      pin_config.mck_io_num    = I2S_PIN_NO_CHANGE ;               // MCK not used
    #endif
    dbgprint ( "Output to I2S, pins %d, %d and %d",                // Show pins used for output device
               pin_config.bck_io_num,                              // This is the BCK (bit clock) pin
               pin_config.ws_io_num,                               // This is L(R)CK pin
               pin_config.data_out_num ) ;                         // This is DATA output pin
    if ( ( I2S_BCK_PIN + I2S_LCK_PIN + I2S_DIN_PIN ) > 2 )         // Check for legal pins
    {
      pinss_err = i2s_set_pin ( i2s_num, &pin_config ) ;           // Set I2S pins
    }
  #endif
  i2s_zero_dma_buffer ( i2s_num ) ;                                // Zero the buffer
  if ( pinss_err != ESP_OK )                                       // Check error condition
  {
    dbgprint ( "I2S setpin error!" ) ;                             // Rport bad pis
    while ( true)                                                  // Forever..
    {
      xQueueReceive ( playqueue, &playdata, 500 ) ;                // Ignore all chunk from queue
    }
  }
  dbgprint ( "Playtask started" ) ;
  while ( true )
  {
    if ( xQueueReceive ( playqueue, &playdata, 500 ) )              // New play request?
    {
      digitalWrite ( LED_BUILTIN, HIGH ) ;                          // Show activity
      fp = SPIFFS.open ( playdata.filename, FILE_READ ) ;           // Open the MP3 file
      if ( !fp )                                                    // Open successful?
      {
        dbgprint ( "File %s does not exist!",                       // No, show and skip request
                   playdata.filename ) ;
        continue ;
      }
      flen = fp.size() ;                                            // Get file length
      dbgprint ( "Play file %s, length is %d bytes",                // Show file info
                 playdata.filename, flen ) ;
      if ( flen > alocspace )                                       // Need for more buffer space?
      {
        alocspace = flen ;                                          // Yes, set new required space
        mp3buff = (uint8_t*)realloc ( mp3buff, alocspace ) ;        // Allocate this space for MP3 file
      }
      if ( mp3buff == nullptr )                                     // Enough space?
      {
        dbgprint ( "File error!" ) ;                                // No, report and skip
        continue ;
      }     
      fp.read ( mp3buff, flen ) ;                                   // Read MP3 data from file
      fp.close() ;                                                  // Close MP3 file
      i2s_start ( i2s_num ) ;                                       // Start I2S
      playBuff ( i2s_num, flen ) ;                                  // Play the file in the buffer
      i2s_stop ( i2s_num ) ;                                        // Stop I2S
      vTaskDelay ( playdata.pause / portTICK_PERIOD_MS ) ;          // silence for a while
      digitalWrite ( LED_BUILTIN, LOW ) ;                           // End of activity
    }
  }
}


//**************************************************************************************************
//                                S C A N G P S S E R I A L                                        *
//**************************************************************************************************
// Listen to input on the GPS serial line.                                                         *
//**************************************************************************************************
void scanGPSserial()
{
  bool           validGPS = false ;              // True if GPS fixed
  char           c ;                             // Character read from serial input
  static char    gpsline[80] ;                   // Buffer for debug output
  static uint8_t inx = 0 ;                       // Index in buffer
  static int     dbgcount = 40 ;                 // Limit debug output
  int            x ;                             // Idem, nieuw aantal
  
  while ( GPSSERIAL.available() )                // Input on serial line?
  {
    c = GPSSERIAL.read() ;                       // Yes, get next injput character
    validGPS = gps.encode ( c ) ;                // Let TinyGPS do the parsing
    if ( c < ' ' )                               // Control character?
    {
      gpsline[inx] = '\0' ;                      // Delimit input line
      if ( ( dbgcount > 0 ) &&                   // Check debug limit
           ( strlen ( gpsline ) > 3 ) )          // Serious input?
      {
        if ( --dbgcount == 0 )                   // Count number of debug lines
        {
          dbgprint ( "GPS input logging ends" ) ;
        }
        else
        {
          dbgprint ( "GPS input <%s>",          // Yes, show debug info
                    gpsline ) ;
        }
      }
      inx = 0 ;                                  // For next input line
    }
    else
    {
      gpsline[inx++] = c ;                       // Store normal characters
    }
  }
  if ( validGPS )
  {
    x = gps.satellites.value() ;                 // Aantal satelieten
    if ( x != gps_nsat )
    {
      gps_nsat = x ;                             // Is veranderd
      //dbgprint ( "GPS satellieten: %d",        // Aantal satellieten in gebruik
      //           gps_nsat ) ;
      if ( gps_nsat > 0 )                        // GPS okay?
      {
        gpsOK = 'G' ;                            // Ja, "G" op display
      } 
      else
      {
        gpsOK = ' ' ;                            // Anders spatie
      }
    }
    if ( gps.location.isUpdated() )
    {
      //dbgprint ( "GPS update" ) ;
      gps_lat = gps.location.lat() ;             // Latitude ophalen
      gps_lon = gps.location.lng() ;             // Longitude ophalen
      gps_lat_s = String ( gps_lat, 6 ) ;        // Latitude als string
      gps_lon_s = String ( gps_lon, 6 ) ;        // Longitude als string
    }
  }
}


//**************************************************************************************************
//                               C L E A N R E G F I L E S                                         *
//**************************************************************************************************
// Clean-up regfiles to free the occupied space.                                                   *
// If less than 100 kB is available, the oldest regfiles are removed.                              *
// If parameter "all" is true: clean-up ALL regfiles.                                              *
//**************************************************************************************************
void cleanRegfiles ( bool all )
{
  size_t dfree ;                                           // Free SPIFFS space
  File   root ;                                            // Root directory
  File   file  ;                                           // Next file 

  while ( true )
  {
    dfree = SPIFFS.totalBytes() - SPIFFS.usedBytes() ;     // Get free space
    if ( dfree > 100000 )                                  // Enough space?
    {
      if ( ! all )
      {
        break ;                                            // Yes, stop cleaning
      }
    }
    root = SPIFFS.open ( "/" ) ;                           // Open root directory
    while ( ( file = root.openNextFile() ) )               // Iterate through files
    {
      if ( strstr ( file.path(), ".csv" ) )                // Is this a reg file?
      {
        dbgprint ( "Remove file %s", file.path() ) ;       // Yes, report
        SPIFFS.remove ( file.path() ) ;                    // And delete oldest regfile
        if ( ! all )                                       // Delete all logs?
        {
          break ;                                          // No, delete just one at the time
        }
      }
    }
  }
}


//**************************************************************************************************
//                               O P E N R E G F I L E                                             *
//**************************************************************************************************
// Open file for registration if not already open.                                                 *
// Filename is the current date.  A sequence number is appended to guarantee unique name.          *
//**************************************************************************************************
bool openRegfile()
{
  int seq = 1 ;                                         // Sequense nr

  if ( regfile )                                        // Regfile already open?
  {
    return true ;                                       // Yes, do not bother
  }
  while ( ! regfile )                                   // Try to open file with unique name
  {
    sprintf ( regfilename, "/%d-%02d-%02d_%03d.csv",    // Format the file name with sequence number
              timeinfo.tm_year + 1900,
              timeinfo.tm_mon + 1,
              timeinfo.tm_mday,
              seq ) ;
    if ( ! SPIFFS.exists ( regfilename) )
    {
      break ;                                           // Unique file name found
    }
    seq++ ;                                             // File exists, try next sequence
  }
  regfile = SPIFFS.open ( regfilename, FILE_WRITE ) ;
  if ( !regfile )
  {
    dbgprint ( "Errr opening reg file!" ) ;
    return false ;
  }
  dbgprint ( "Opened new reg file %s", regfilename ) ;
  // First line in regfile is date and GPS position (if any)
  regfile.printf ( "G,%02d-%02d-%d,%.6f,%.6f\n",
                    timeinfo.tm_mday, timeinfo.tm_mon + 1,
                    timeinfo.tm_year+1900,
                    gps_lat, gps_lon ) ;
  regfile.printf ( "P,%d\n", probability ) ;            // Log probability
  regfile.flush() ;                                     // Make sure output is written

  return true ;
}


//**************************************************************************************************
//                                           S E T U P                                             *
//**************************************************************************************************
// Setup for the program.                                                                          *
//**************************************************************************************************
void setup()
{
  const char*               partname = "nvs" ;           // Partition with NVS info
  esp_partition_iterator_t  pi ;                         // Iterator for partition_find()
  const esp_partition_t*    ps ;                         // Pointer to partition struct
  String                    tmps ;                       // Temporary string
  bool                      i2cok ;                      // I2C initialization result
  uint32_t                  s ;                          // Timestamp used as randomseed
  char                      hash = 0 ;                   // Hash of mac address

  xmaintask = xTaskGetCurrentTaskHandle() ;              // My taskhandle
  dbgsem  = xSemaphoreCreateMutex() ;                    // Semaphore for exclusive use of dbgprint
  i2csem  = xSemaphoreCreateMutex() ;                    // Semaphore for exclusive use of I2C bus
  i2cok = Wire.begin() ;                                 // Init I2C driver for RTC
  pinMode ( NEOPIX_PIN, OUTPUT ) ;                       // Needed?
  delay ( 4000 ) ;                                       // Wait for terminal to start
  Serial.begin ( 115200 ) ;                              // For debug
  Serial.println() ;
  nvsinit() ;                                            // Init NVS
  GPSSERIAL.begin ( 9600, SERIAL_8N1, pin_gps_in ) ;     // Define GPS serial inpout
  WRITE_PERI_REG ( RTC_CNTL_BROWN_OUT_REG, 0 ) ;         // Disable brownout detector
  pinMode ( LED_BUILTIN, OUTPUT ) ;                      // Enable built-in LED
  digitalWrite ( LED_BUILTIN, HIGH ) ;                   // Show activity
  pinMode ( BUTTON0, INPUT_PULLUP ) ;                    // Enable Start button
  pinMode ( BUTTON1, INPUT_PULLUP ) ;                    // Enable second Start button
  attachInterrupt ( BUTTON0, ISR_button, FALLING ) ;     // Link ISR to start button
  attachInterrupt ( BUTTON1, ISR_button, FALLING ) ;     // Link ISR to second start button
  if ( ! i2cok )                                         // Check I2C status
  {
    dbgprint ( "I2C initialization error!" ) ;           // Report if error
  }
  Serial.printf ( "\nStarting " NAME ", running on CPU %d "
                  "at %d MHz.\nVersion %s.\n"
                  "Compiled at %s %s local time\n"
                  "Free memory %d\n\n",
                  xPortGetCoreID(),
                  ESP.getCpuFreqMHz(),
                  VERSION,
                  __DATE__, __TIME__,                    // Compile date/time
                  ESP.getFreeHeap() ) ;                  // Free RAM memory
  initRtc() ;                                            // Init RTC
  WiFi.macAddress ( mac ) ;                              // Get mac-adress
  hash = mac[3] ^ mac[2] ^ mac[1] ^ mac[0] ;             // Hash it
  sprintf ( AP_SSIDstr, "%s-NET-%02X", NAME, hash ) ;    // Generate "unique" AP name
  strcpy ( AP_pw, AP_SSIDstr ) ;                         // Password is the same
  getRtcTemp() ;                                         // Read temperature
  s = getRtcTime() ;                                     // Read from RTC
  randomSeed ( s ) ;                                     // Init random generator
  dbgprint ( "Time (RTC) is now %02d:%02d:%4d %s,"
             " temperature is %s °C ",
             timeinfo.tm_mday,
             timeinfo.tm_mon + 1,
             timeinfo.tm_year + 1900,
             timetxt, rtctemptxt ) ;
  if ( !SPIFFS.begin ( FSIF ) )                          // Mount and test SPIFFS
  {
    dbgprint ( "SPIFFS Mount Fout!" ) ;                  // A pity...
  }
  else
  {
    dbgprint ( "SPIFFS is okay, space %d, used %d",      // Show available SPIFFS space
               SPIFFS.totalBytes(),
               SPIFFS.usedBytes() ) ;
    showAllFiles() ;                                     // Show all files on SPIFFS
    cleanRegfiles ( false ) ;                            // Remove Registration files if not enough space
  }
  pi = esp_partition_find ( ESP_PARTITION_TYPE_DATA,     // Get partition iterator for
                            ESP_PARTITION_SUBTYPE_ANY,   // All data partitions
                            nullptr ) ;
  while ( pi )
  {
    ps = esp_partition_get ( pi ) ;                      // Get partition struct
    dbgprint ( "Found partition '%-8.8s' "               // Show partition
               "at offset 0x%06X "
               "with size %8d",
               ps->label, ps->address, ps->size ) ;
    if ( strcmp ( ps->label, partname ) == 0 )           // Is this the NVS partition?
    {
      nvs = ps ;                                         // Yes, remember NVS partition
    }
    else if ( strcmp ( ps->label, "spiffs" ) == 0 )      // Is this the SPIFFS partition?
    {
      pspiffs = ps ;                                     // Yes, remember SPIFFS partition
    }
    pi = esp_partition_next ( pi ) ;                     // Find next
  }
  //esp_partition_iterator_release ( pi ) ;              // Release the iterator (but pi = NULL??)
  if ( nvs == nullptr )
  {
    dbgprint ( "Partition %s not found!", partname ) ;   // Very unlikely...
    while ( true ) ;                                     // Impossible to continue
  }
  if ( pspiffs == nullptr )
  {
    dbgprint ( "Partition spiffs not found!" ) ;         // Very unlikely...
    while ( true ) ;                                     // Impossible to continue
  }
  namespace_ID = FindNsID ( NAME ) ;                   // Find ID of our namespace in NVS
  dbgprint ( "FindNsID result is 0x%02X",
             namespace_ID ) ;
  if ( namespace_ID == 0xFF )                            // Namespace found?
  {
     dbgprint ( "NVS not okay!" ) ;
  }
  readprefs ( false ) ;                                  // Read preferences
  if ( numprefs == 0 )                                   // Waren er preferences?
  {
    initprefs() ;                                        // Nee, vul met defaults
  }
  WiFi.onEvent ( wifi_event ) ;
  mk_lsan() ;                                            // Make al list of acceptable networks
  listNetworks() ;                                       // Search for WiFi networks
  tcpip_adapter_set_hostname ( TCPIP_ADAPTER_IF_STA,
                               NAME ) ;
  dbgprint ( "Connect to WiFi" ) ;                       // Show progress
  NetworkFound = connectwifi() ;                         // Connect to WiFi network
  dbgprint ( "Start web server for commands" ) ;
  cmdserver.on ( "/getprefs",   handle_getprefs ) ;      // Handle get preferences
  cmdserver.on ( "/saveprefs",  handle_saveprefs ) ;     // Handle save preferences
  cmdserver.on ( "/syslog",     handle_syslog ) ;        // Handle show syslog
  cmdserver.on ( "/regis",      handle_regis ) ;         // Handle list filenames
  cmdserver.on ( "/export",     handle_export ) ;        // Handle export of registration file 
  cmdserver.on ( "/reset",      handle_reset ) ;         // Handle update software button
  cmdserver.on ( "/mtsync",     handle_mtsync ) ;        // Handle sync RTC
  cmdserver.on ( "/prob",       handle_prob ) ;          // Handle probability setting
  cmdserver.on ( "/gain",       handle_gain ) ;          // Handle gain setting
  cmdserver.on ( "/brightness", handle_brightness ) ;    // Handle brightness setting
  cmdserver.on ( "/status",     handle_status ) ;        // Handle status request
  cmdserver.on ( "/settings",   handle_settings ) ;      // Handle audio/model setting request
  cmdserver.on ( "/plogin",     handle_login ) ;         // Handle log in request
  cmdserver.on ( "/logout",     handle_logout ) ;        // Handle log out request
  cmdserver.onNotFound ( handleFileRead ) ;              // Just handle a simple page/file
  cmdserver.begin() ;                                    // Start http server
  // Start DNS and OTA only if in STA mode (with home network)
  if ( NetworkFound )
  {
    dbgprint ( "Network found" ) ;
    #ifdef OTA
      ArduinoOTA.setHostname ( NAME ) ;                  // Set the hostname
      ArduinoOTA.onStart ( otastart ) ;
      ArduinoOTA.begin() ;                               // Allow update over the air
    #endif
    if ( MDNS.begin ( AP_SSIDstr ) )                     // Start MDNS transponder
    {
      dbgprint ( "MDNS responder started" ) ;
    }
    else
    {
      dbgprint ( "Error setting up MDNS responder!" ) ;
    }
  }
  playqueue = xQueueCreate ( QSIZ,                       // Create queue for communication
                             sizeof ( qdata_struct ) ) ;
  dispqueue = xQueueCreate ( QSIZ,                       // Create queue for disptask
                             sizeof ( qdisp_struct ) ) ;
  xTaskCreatePinnedToCore (
    playtaskI2S,                                         // Task to play file in playqueue to I2S
    "Playtask",                                          // Name of task
    6000,                                                // Stack size of task
    NULL,                                                // parameter of the task
    2,                                                   // priority of the task
    &xplaytask,                                          // Task handle to keep track of created task
    0 ) ;                                                // Run on CPU 0
  xTaskCreatePinnedToCore (
    disptask,                                            // Task to handle display on panels
    "Disptask",                                          // name of task
    2500,                                                // Stack size of task
    NULL,                                                // parameter of the task
    2,                                                   // priority of the task
    &xdisptask,                                          // Task handle to keep track of created task
    0 ) ;                                                // Run on CPU 0
  timer = timerBegin ( 0, 80, true ) ;                   // User 1st timer with prescaler 80
  timerAttachInterrupt ( timer, &timer100, false ) ;     // Call timer100() on timer alarm
  timerAlarmWrite ( timer, 100000, true ) ;              // Alarm every 100 msec
  timerAlarmEnable ( timer ) ;                           // Enable the timer
  while ( GPSSERIAL.available() )                        // Flush all GPS input
  {
    GPSSERIAL.read() ;                                   // by reading input character
  }
  dbgprint ( "Wait for dsptask" )  ;
  while ( ! dsptaskrdy )                                 // Wait until dislay task ready 
  {
    vTaskDelay ( 500 / portTICK_PERIOD_MS ) ;            // Pause
  }
  dbgprint ( "End Wait for dsptask" )  ;
  buttonreq = false ;                                    // Reset possible button push
  dbgprint ( "End of setup" ) ;
  digitalWrite ( LED_BUILTIN, LOW ) ;                    // End activity
}


//**************************************************************************************************
//                                           L O O P                                               *
//**************************************************************************************************
// Main loop of the program.                                                                       *
//**************************************************************************************************
void loop()
{
  delay ( 100 ) ;
  if ( buttonreq )                                        // Button pushed?
  {
    handle_button() ;                                     // Yes, handle the button
    vTaskDelay ( 5000 / portTICK_PERIOD_MS ) ;            // 5 seconds dead period
    buttonreq = false ;
  }
  if ( resetreq )                                         // Reset requested?
  {
    delay ( 1000 ) ;                                      // Yes, wait some time
    ESP.restart() ;                                       // Reboot
  }
  if ( pwresetreq )                                       // Reset all passwords?
  {
    pwresetreq = false ;                                  // Yes, clear request
    dbgprint ( "Password reset requested" ) ;             // Show it
    nvs_erase_key ( nvshandle, "userpw" ) ;               // Yes, reset user password
    nvs_erase_key ( nvshandle, "adminpw" ) ;              // and admin password
    nvs_erase_key ( nvshandle, "wifipw" ) ;               // and WiFi password
  }
  scanserial() ;                                          // Handle serial input
  scanGPSserial() ;                                       // Listen for GPS info
  if ( errorcount >= 1000 )                               // Too much errors?
  {
    dbgprint ( "Errorcount too high!  Reset.." ) ;
    resetreq = true ;                                     // Yes, force reset
  }
  #ifdef OTA
    ArduinoOTA.handle() ;                                 // Check for OTA
  #endif
  xmaincount++ ;                                          // Count number of loops
}
