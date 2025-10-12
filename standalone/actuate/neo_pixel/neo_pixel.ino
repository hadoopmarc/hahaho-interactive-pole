#include <NeoPixelBus.h> 

// Neopixel displays (2x3 modules 8x8)
#define NEO_PANELS        8                                 // Number of panels
#define NEOPIX_PIN        12                                // GPIO for Neopixel displays
#define NEOPIX_COUNT      512                               // Total number of pixels

// Neopixel stuff
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> neodisp(NEOPIX_COUNT, NEOPIX_PIN);
RgbColor          n_white = RgbColor ( 255, 255, 255 ) ; // Colors used for Neopixels
RgbColor          n_black = RgbColor ( 0,     0,   0 ) ; // All colors off
RgbColor          n_red   = RgbColor ( 255,   0,   0 ) ; // Red
RgbColor          n_green = RgbColor ( 0,   255,   0 ) ; // Green
RgbColor          n_blue  = RgbColor ( 0,     0, 255 ) ; // Blue
uint8_t           n_brightness = 30 ;                    // Brightness 0..100


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


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
