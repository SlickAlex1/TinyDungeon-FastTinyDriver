#include <Arduino.h>
#include "TinyJoypadUtils.h"

#if defined(__AVR_ATtiny85__)
  #include "FastTinyDriver.h"
#else
  #include <Adafruit_SSD1306.h>
  Adafruit_SSD1306 display( 128, 64, &Wire, -1 );
  uint8_t *adafruitBuffer;
  #ifdef _ENABLE_SERIAL_SCREENSHOT_
    #include "SerialHexTools.h"
  #endif
#endif

uint16_t analogJoystickX;
uint16_t analogJoystickY;

/*-------------------------------------------------------*/
void InitTinyJoypad()
{
#if defined(__AVR_ATtiny85__)
  SOUND_PORT_DDR &= ~( ( 1 << PB5) | ( 1 << PB3 ) | ( 1 << PB1 ) );
  SOUND_PORT_DDR |= ( 1 << SOUND_PIN );
#else
  pinMode( LEFT_RIGHT_BUTTON, INPUT );
  pinMode( UP_DOWN_BUTTON, INPUT );
  pinMode( FIRE_BUTTON, INPUT );
  pinMode( SOUND_PIN, OUTPUT );
  Serial.begin( 115200 );
#endif
}

bool isLeftPressed()   { uint16_t inputX = analogRead( LEFT_RIGHT_BUTTON ); return( ( inputX >= ANALOG_UPPER_LIMIT_MIN ) && ( inputX < ANALOG_UPPER_LIMIT_MAX ) ); }
bool isRightPressed()  { uint16_t inputX = analogRead( LEFT_RIGHT_BUTTON ); return( ( inputX > ANALOG_LOWER_LIMIT_MIN ) && ( inputX < ANALOG_LOWER_LIMIT_MAX ) ); }
bool isUpPressed()     { uint16_t inputY = analogRead( UP_DOWN_BUTTON );    return( ( inputY > ANALOG_LOWER_LIMIT_MIN ) && ( inputY < ANALOG_LOWER_LIMIT_MAX ) ); }
bool isDownPressed()   { uint16_t inputY = analogRead( UP_DOWN_BUTTON );    return( ( inputY >= ANALOG_UPPER_LIMIT_MIN ) && ( inputY < ANALOG_UPPER_LIMIT_MAX ) ); }
bool isFirePressed()   { return( digitalRead( FIRE_BUTTON ) == 0 ); }
void waitUntilButtonsReleased() { while( isLeftPressed() || isRightPressed() || isUpPressed() || isDownPressed() || isFirePressed() ); }
void waitUntilButtonsReleased(const uint8_t delayTime) { waitUntilButtonsReleased(); _delay_ms( delayTime ); }
void readAnalogJoystick() { analogJoystickX = analogRead( LEFT_RIGHT_BUTTON ); analogJoystickY = analogRead( UP_DOWN_BUTTON ); }
bool wasLeftPressed()  { return( ( analogJoystickX >= ANALOG_UPPER_LIMIT_MIN ) && ( analogJoystickX < ANALOG_UPPER_LIMIT_MAX ) ); }
bool wasRightPressed() { return( ( analogJoystickX > ANALOG_LOWER_LIMIT_MIN ) && ( analogJoystickX < ANALOG_LOWER_LIMIT_MAX ) ); }
bool wasUpPressed()    { return( ( analogJoystickY > ANALOG_LOWER_LIMIT_MIN ) && ( analogJoystickY < ANALOG_LOWER_LIMIT_MAX ) ); }
bool wasDownPressed()  { return( ( analogJoystickY >= ANALOG_UPPER_LIMIT_MIN ) && ( analogJoystickY < ANALOG_UPPER_LIMIT_MAX ) ); }
uint16_t getAnalogValueX() { return( analogJoystickX ); }
uint16_t getAnalogValueY() { return( analogJoystickY ); }

void __attribute__ ((noinline)) _variableDelay_us( uint8_t delayValue )
{
  while ( delayValue-- != 0 ) { _delay_us( 1 ); }
}

void Sound( const uint8_t freq, const uint8_t dur )
{
  for ( uint8_t t = 0; t < dur; t++ )
  {
#if defined(__AVR_ATtiny85__)
    if ( freq != 0 ){ SOUND_PORT = SOUND_PORT | ( 1 << SOUND_PIN ); }
    _variableDelay_us( 255 - freq );
    SOUND_PORT = SOUND_PORT & ~( 1 << SOUND_PIN );
    _variableDelay_us( 255 - freq );
#else
    if ( freq != 0 ){ digitalWrite( SOUND_PIN, 1 ); }
    _variableDelay_us( 255 - freq );
    digitalWrite( SOUND_PIN, 0 );
    _variableDelay_us( 255 - freq );
#endif
  }
}

/*-------------------------------------------------------*/
void InitDisplay()
{
#if defined(__AVR_ATtiny85__)
  TinyOLED_init();
  for (uint8_t y = 0; y < 8; y++) {
    ssd1306_selectPage(y);
    for (uint8_t x = 0; x < 128; x++) i2c_write(0x00);
    i2c_stop();
  }
#else
  if( !display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed - 1024 bytes for frame buffer required!")); for(;;);
  }
#endif
}

/*-------------------------------------------------------*/
void PrepareDisplayRow( uint8_t y )
{
#if defined(__AVR_ATtiny85__)
  ssd1306_selectPage(y);
#else
  adafruitBuffer = display.getBuffer() + ( y * 128 );
#endif
}

/*-------------------------------------------------------*/
void SendPixels( uint8_t pixels )
{
#if defined(__AVR_ATtiny85__)
  i2c_write( pixels );
#else
  *adafruitBuffer++ = pixels;
#endif
}

/*-------------------------------------------------------*/
void FinishDisplayRow()
{
#if defined(__AVR_ATtiny85__)
  i2c_stop();
#endif
}

/*-------------------------------------------------------*/
void DisplayBuffer()
{
#if !defined(__AVR_ATtiny85__)
  display.display();
  #if defined(_VARIANT_ARDUINO_ZERO_)
    delay(100);
  #endif
  #ifndef _SERIAL_SCREENSHOT_NO_AUTO_SHOT_
    CheckForSerialScreenshot();
  #endif
#endif
}

/*-------------------------------------------------------*/
void SerialScreenshot()
{
#if !defined(__AVR_ATtiny85__)
  #ifdef _ENABLE_SERIAL_SCREENSHOT_
    Serial.println( F("\r\nThis is a TinyJoypad screenshot. Output is one hex byte per pixel. To get the actual image perform the following steps:") );
    Serial.println( F("(1) The output can be converted to binary with 'https://tomeko.net/online_tools/hex_to_file.php?lang=en' online.") );
    Serial.println( F("(2) Then import the file with IrfanView (https://www.irfanview.com/): Open as -> RAW file...") );
    Serial.println( F("(3) Set Image width to 64 and Image height to 128, 8 BPP -> OK") );
    Serial.println( F("(4) Rotate and mirror the result as needed :)\r\n") );
    Serial.println( F("Hint: If you only get partial screenshots, try using a terminal program to capture the serial output.") );
    printScreenBufferToSerial( display.getBuffer(), 128, 8 );
  #endif
#endif
}

/*-------------------------------------------------------*/
void CheckForSerialScreenshot()
{
#if !defined(__AVR_ATtiny85__)
  #ifdef _ENABLE_SERIAL_SCREENSHOT_
    if ( _SERIAL_SCREENSHOT_TRIGGER_CONDITION_ )
    {
      SerialScreenshot();
    }
  #endif
#endif
}

/*-------------------------------------------------------*/
void serialPrint( const char *text )
{
#ifdef USE_SERIAL_PRINT
  Serial.print( text );
#endif
}

void serialPrintln( const char *text )
{
#ifdef USE_SERIAL_PRINT
  Serial.println( text );
#endif
}

void serialPrint( const __FlashStringHelper *text )
{
#ifdef USE_SERIAL_PRINT
  Serial.print( text );
#endif
}

void serialPrintln( const __FlashStringHelper *text )
{
#ifdef USE_SERIAL_PRINT
  Serial.println( text );
#endif
}

void serialPrint( const unsigned int number )
{
#ifdef USE_SERIAL_PRINT
  Serial.print( number );
#endif
}

void serialPrintln( const unsigned int number )
{
#ifdef USE_SERIAL_PRINT
  Serial.println( number );
#endif
}

void serialPrint( const int number )
{
#ifdef USE_SERIAL_PRINT
  Serial.print( number );
#endif
}

void serialPrintln( const int number )
{
#ifdef USE_SERIAL_PRINT
  Serial.println( number );
#endif
}