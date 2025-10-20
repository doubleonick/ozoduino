#include "ColorWireling.h"

ColorWireling::ColorWireling(){}

ColorWireling::ColorWireling(byte port)
{
  _port = port;
}

void ColorWireling::init(void)
{
  // Wire.begin();
  // Wireling.begin();

  //The port is the number on the Adapter board where the sensor is attached
  Wireling.selectPort(_port);

  _tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

  if (_tcs.begin()) {
    // SerialMonitorInterface.println("Found sensor");
  } else {
    // SerialMonitorInterface.println("No TCS34725 found ... check your connections");
    while (1);
  }

  // Turn Wireling LEDs on 
  // LEDon(); 
  //Light sensor... turn OFF LEDs
  // LEDoff();
}

void ColorWireling::getColorData(int port) 
{
  Wireling.selectPort(port);

  // LEDon();
  _tcs.getRawData(&_r, &_g, &_b, &_c);
  _colorTemp = _tcs.calculateColorTemperature(_r, _g, _b);
  _lux = _tcs.calculateLux(_r, _g, _b);

  // SerialMonitorInterface.print("Color Temp: "); SerialMonitorInterface.print(_colorTemp); SerialMonitorInterface.print(" K, ");
  //  SerialMonitorInterface.print("Lux: "); SerialMonitorInterface.print(_lux, DEC); SerialMonitorInterface.print(", ");
  // SerialMonitorInterface.print("R: "); SerialMonitorInterface.print(_r, DEC); SerialMonitorInterface.print(", ");
  // SerialMonitorInterface.print("G: "); SerialMonitorInterface.print(_g, DEC); SerialMonitorInterface.print(", ");
  // SerialMonitorInterface.print("B: "); SerialMonitorInterface.print(_b); SerialMonitorInterface.print(", ");
  // SerialMonitorInterface.print("Clr: "); SerialMonitorInterface.print(_c, DEC);
  // SerialMonitorInterface.println(" ");

  rgbToHsv(_r, _g, _b);

  delay(100);
}

void ColorWireling::rgbToHsv(int r, int g, int b)
{
  float rPrime, gPrime, bPrime;
  float cMax, cMin, cDelta;
  // float h, s, v;

  rPrime = r/255;
  gPrime = g/255;
  bPrime = b/255;

  cMax   = max(max(rPrime, gPrime), bPrime);
  cMin   = min(min(rPrime, gPrime), bPrime);
  cDelta = cMax - cMin;
  //Hue
  if(cDelta == 0)
  {
    _h = 0;
  }
  else if(cMax == rPrime)
  {
    _h = fmod(((gPrime - bPrime)/cDelta), 6.0);
  }
  else if(cMax == gPrime)
  {
    _h = ((bPrime - rPrime)/cDelta + 2.0);
  }
  else if(cMax == bPrime)
  {
    _h = (rPrime - gPrime)/cDelta + 4.0;
  }

  _h = _h * 60;

  //Saturation
  if(cMax == 0)
  {
    _s = 0;
  }
  else if(cMax != 0)
  {
    _s = cDelta / cMax;
  }

  //Value
  _v = cMax;

  // SerialMonitorInterface.print("h: ");
  // SerialMonitorInterface.print(h);
  // SerialMonitorInterface.print(" , s: ");
  // SerialMonitorInterface.print(s);
  // SerialMonitorInterface.print(", v: ");
  // SerialMonitorInterface.println(v);
}

uint16_t ColorWireling::getR()
{
  return _r;
}

uint16_t ColorWireling::getG()
{
  return _g;
}

uint16_t ColorWireling::getB()
{
  return _b;
}

uint16_t ColorWireling::getC()
{
  return _c;
}

uint16_t ColorWireling::getColorTemp()
{
  return _colorTemp;
}

uint16_t ColorWireling::getLux()
{
  return _lux;
}

float ColorWireling::getH()
{
  return _h;
}

float ColorWireling::getS()
{
  return _s;
}

float ColorWireling::getV()
{
  return _v;
}
/*
// Turn Wireling LEDs on
void ColorWireling::LEDon() {
  _tcs.setInterrupt(true);
}

// Turn Wireling LEDs off
void ColorWireling::LEDoff() {
  _tcs.setInterrupt(false);
}*/
