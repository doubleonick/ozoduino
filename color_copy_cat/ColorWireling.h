#ifndef COLORWIRELING_H
#define COLORWIRELING_H

#include <Arduino.h>
#include <Wire.h>         // For I2C communication with sensor
#include "Adafruit_TCS34725.h"  // The library used for the Color Sensor
#include <Wireling.h>     // For interfacing with Wirelings

class ColorWireling
{
  private:
    /* Initialise with specific int time and gain values */
    Adafruit_TCS34725 _tcs;

    byte _port;

    // Variables to hold the values the sensor reads
    uint16_t _r;
    uint16_t _g; 
    uint16_t _b;
    uint16_t _c;
    uint16_t _colorTemp;
    uint16_t _lux;

    float _h;
    float _s;
    float _v;

  public:
    ColorWireling();
    ColorWireling(byte port);
    void init(void);

    void getColorData(int port);

    void rgbToHsv(int r, int g, int b);

    uint16_t getR();

    uint16_t getG();

    uint16_t getB();

    uint16_t getC();

    uint16_t getColorTemp();

    uint16_t getLux();

    float getH();
    
    float getS();

    float getV();
    /*
    // Turn Wireling LEDs on
    void LEDon();

    // Turn Wireling LEDs off
    void LEDoff();
    */
};

#endif
