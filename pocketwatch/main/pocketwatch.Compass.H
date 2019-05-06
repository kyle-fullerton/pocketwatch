#ifndef POCKETWATCH_COMPASS_H
#define POCKETWATCH_COMPASS_H

#include <Wire.h>

namespace pocketwatch
{

typedef unsigned long time_t;

#define COMPASS_I2C_ADDRESS 0x1E

class Compass
{
public:
  Compass();

  void start(time_t startTime, time_t updatePer);
  void process(time_t currentTime);

  float getHeading() const;
  int16_t getX() const { return x; }
  int16_t getY() const { return y; }
  int16_t getZ() const { return z; }

private:

  void requestData();
  int16_t read16Bits();
  
  time_t updatePeriod;

  time_t lastUpdateTime;
  int16_t x;
  int16_t y;
  int16_t z;

};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Compass::Compass() : updatePeriod(1000),
                     lastUpdateTime(0),
                     x(0),
                     y(0),
                     z(0)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Compass::start(time_t startTime, time_t updatePer)
{
  lastUpdateTime = startTime;
  updatePeriod = updatePer;
  
  Wire.begin();

  // Set configuration register 1
  Wire.beginTransmission(COMPASS_I2C_ADDRESS);
  Wire.write(0x00);
  // Binary 0 00 100 00 = Hexadecimal 10
  //        0  : Reserved
  //        00 : Average 2^this number of samples for each reading
  //        100: Take 15 samples per second
  //        00 : Don't put a positive or negative bias on the sensors
  Wire.write(0x10);
  Wire.endTransmission();
  
  // Set configuration register 2
  Wire.beginTransmission(COMPASS_I2C_ADDRESS);
  Wire.write(0x01);
  // Binary 001 00000 = Hexadecimal 20
  //        001  : Default sensor gain
  //        00000: Must all be 0
  Wire.write(0x20);
  Wire.endTransmission();
  
  // Set mode register
  Wire.beginTransmission(COMPASS_I2C_ADDRESS);
  Wire.write(0x02);
  // Binary 0 00000 00 = Hexadecimal 00
  //        0    : Don't take high-speed samples
  //        00000: Must all be 0
  //        00   : Take samples repeatedly (01 for only on command; 1x for never)
  Wire.write(0x00);
  Wire.endTransmission();
  
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Compass::process(time_t currentTime)
{
  if (currentTime - lastUpdateTime > updatePeriod)
  {
    lastUpdateTime = currentTime;

    requestData();

    x = read16Bits();
    z = read16Bits();
    y = read16Bits();
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
float Compass::getHeading() const
{
  if ((x == 0) && (y == 0))
  {
    return 0.0f;
  }
  else
  {
    return atan2(y, x);
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Compass::requestData()
{
  // Command it to send back data starting with register 0x03 (high bits of X)
  Wire.beginTransmission(COMPASS_I2C_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  
  // Request 6 bytes of data
  Wire.requestFrom(COMPASS_I2C_ADDRESS, 6);

  while (Wire.available() < 6) {}

}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int16_t Compass::read16Bits()
{
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();

  return (int16_t)(lo | (int16_t)hi << 8);
}

} // end namespace pocketwatch

#endif
