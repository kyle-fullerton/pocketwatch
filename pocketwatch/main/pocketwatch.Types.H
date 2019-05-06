#ifndef POCKETWATCH_TYPES_H
#define POCKETWATCH_TYPES_H

#define EARTH__RADIUS 6371000.0

namespace pocketwatch
{

typedef unsigned long time_t;

namespace types
{

struct SensorData
{
public:

  uint8_t selection;

  float heading;
  int16_t compassX;
  int16_t compassY;
  int16_t compassZ;

  uint8_t yearSince2000;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  double latitude;
  double longitude;

  float groundSpeedKnots;
  float trackAngle;
  float altitudeMeters;
  
  double fastWaypointLatitude;
  double fastWaypointLongitude;
  double slowWaypointLatitude;
  double slowWaypointLongitude;
  
};

struct Hands
{
  uint8_t bigHand;
  uint8_t mediumHand;
  uint8_t smallHand;
};

struct MotorConfig
{
  uint8_t pinA1;
  uint8_t pinA2;
  uint8_t pinB1;
  uint8_t pinB2;
};


} // end namespace types
} // end namespace pocketwatch

#endif
