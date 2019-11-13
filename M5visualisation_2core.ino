#include <atomic>                      // std::atomic, std::memory_order_relaxed
#include "arduinoFFT.h"
#include <M5Stack.h>
//#include <TFT_eSPI.h>
#include <M5Display.h>                 // This library is a lot faster! But also a lot larger.

#define TFT_CPU_CORE            0      // Select which core the tft task will run on.

#define microphone              34
#define speaker                 25
#define backlight               32

#define bufferSize              256

#define spectrumWidth           140
#define spectrumHeight          90

#define waveWidth               bufferSize
#define waveHeight              90

#define SAMPLING_FREQUENCY      48000

double vReal[bufferSize];
double vImag[bufferSize];

/* shared between ISR and main program */
std::atomic<std::uint16_t> currentSample{0};
std::atomic<bool>          bufferFilled;

static hw_timer_t *        sampleTimer = NULL;

unsigned int sampling_period_us = round( 1000000 * ( 1.0 / SAMPLING_FREQUENCY ) );

arduinoFFT  FFT      = arduinoFFT();
TFT_eSPI    tft      = TFT_eSPI();
TFT_eSprite spectrum = TFT_eSprite(&tft);
TFT_eSprite waveform = TFT_eSprite(&tft);

static void IRAM_ATTR _sampleISR() {
  if ( bufferFilled ) return;
  vReal[currentSample] = analogRead( microphone );
  vImag[currentSample] = 0;
  ++currentSample;
  if ( currentSample == bufferSize ) {
    bufferFilled = true;
  }
}

void setup() {
  pinMode( microphone, INPUT );
  dacWrite( speaker, 0 );

  ledcAttachPin( backlight, 0);
  ledcSetup( 0, 1300, 16 );
  ledcWrite( 0, 0xFFFF  / 16  ); //dimming the backlight will produce more base noise

  sampleTimer = timerBegin( 0, 80, true );
  timerAttachInterrupt( sampleTimer, &_sampleISR, true );
  timerAlarmWrite( sampleTimer, sampling_period_us, true );
  timerAlarmEnable( sampleTimer );

  //esp_wifi_set_mode(WIFI_MODE_NULL);

  bufferFilled = false;
  xTaskCreatePinnedToCore(
    updateTFT,                       /* Function to implement the task */
    "updateTFT",                     /* Name of the task */
    3000,                           /* Stack size in words */
    NULL,                           /* Task input parameter */
    5,                              /* Priority of the task */
    NULL,                           /* Task handle. */
    TFT_CPU_CORE);
}

void loop() {
  vTaskDelete(NULL);
}

uint16_t fps;
time_t previous;

void updateTFT( void * pvParameters ) {
  tft.init();
  tft.setRotation( 1 );
  tft.fillScreen( TFT_BLUE );
  tft.setTextColor( TFT_WHITE, TFT_BLUE );
  tft.setTextSize( 2 );
  tft.setCursor( 75, 105 );

  spectrum.createSprite( spectrumWidth, spectrumHeight );
  spectrum.setColorDepth( 8 );
  waveform.createSprite( waveWidth, waveHeight );
  waveform.setColorDepth( 8 );

  while ( true ) {
    if ( bufferFilled ) {
      waveform.fillSprite( TFT_BLACK );
      for ( uint16_t counter = 1; counter < waveWidth; counter++ ) {
        waveform.drawLine( counter - 1,
                           map( vReal[counter - 1], 0, 4096, 0, waveHeight ),
                           counter,
                           map( vReal[counter], 0, 4096, 0, waveHeight ),
                           ILI9341_WHITE );
      }
      waveform.pushSprite( 32, 130 );

      spectrum.fillScreen( TFT_BLACK );
      FFT.Windowing( vReal, bufferSize, FFT_WIN_TYP_HAMMING, FFT_FORWARD) ;
      FFT.Compute( vReal, vImag, bufferSize, FFT_FORWARD );
      FFT.ComplexToMagnitude( vReal, vImag, bufferSize );
      // The binning code was copied from https://github.com/G6EJD/ESP32-8266-Audio-Spectrum-Display/blob/master/ESP32_Spectrum_Display_02.ino
      for ( int i = 2; i < ( bufferSize / 2); i++ ) {
        if (i <= 2 )             displayBand(0, (int)vReal[i]); // 125Hz
        if (i > 2   && i <= 4 )   displayBand(1, (int)vReal[i]); // 250Hz
        if (i > 4   && i <= 7 )   displayBand(2, (int)vReal[i]); // 500Hz
        if (i > 7   && i <= 15 )  displayBand(3, (int)vReal[i]); // 1000Hz
        if (i > 15  && i <= 40 )  displayBand(4, (int)vReal[i]); // 2000Hz
        if (i > 40  && i <= 70 )  displayBand(5, (int)vReal[i]); // 4000Hz
        if (i > 70  && i <= 288 ) displayBand(6, (int)vReal[i]); // 8000Hz
        if (i > 288           ) displayBand(7, (int)vReal[i]); // 16000Hz
      }

      currentSample.store( 0, std::memory_order_relaxed );
      bufferFilled = false;
      spectrum.pushSprite( 90, 10 );
    }
    static time_t previous;
    static uint16_t fps;
    if ( time( NULL ) != previous ) {
      tft.setCursor( 35, 110);
      tft.printf( "%3.1fkHz Core:%i %3.ifps", SAMPLING_FREQUENCY / 1000.0, TFT_CPU_CORE, fps );
      previous = time( NULL );
      fps = 0;
    }
    fps++;

    delay(1);
  }
}

static inline void displayBand( const int band, const int dsize ) {
  spectrum.fillRect( band * 20 + 5, spectrumHeight - dsize / ( bufferSize  * 2.5 ) + 10 , 10, spectrumHeight, TFT_GREEN );
}
