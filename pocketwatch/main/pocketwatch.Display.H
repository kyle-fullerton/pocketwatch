#ifndef POCKETWATCH_DISPLAY_H
#define POCKETWATCH_DISPLAY_H

#include <Adafruit_NeoPixel.h>

#include "pocketwatch.Motor.H"
#include "pocketwatch.Types.H"

#define POCKETWATCH__DISPLAY__NUMPOSITIONS 120.0f
#define PIN 6
#define Fast 'F'
#define Slow 'S'

namespace pocketwatch
{

typedef unsigned long time_t;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(17, PIN, NEO_GRB + NEO_KHZ800);
class Display
{
  public:
    Display();

    void start(time_t startTime, time_t refreshDur, time_t handMotionDur, uint8_t nPositions, const types::MotorConfig& bigMotorConfig);
    void process(time_t currentTime, const types::SensorData& data);


  private:


    void calculateTimeOfDay(types::Hands& handPositions, const types::SensorData& data);
    void calculateTravelingData(types::Hands& handPositions, const types::SensorData& data);
    void calculateWaypointReturnData(types::Hands& handPositions, const types::SensorData& data);
    void calculateHomeReturnData(types::Hands& handPositions, const types::SensorData& data);

    uint8_t calculateNorth(const types::SensorData& data);
    uint8_t calculateSpeed(const types::SensorData& data);
    uint8_t calculateAltitude(const types::SensorData& data);
    void calculateDistanceAndDirection(uint8_t& dist, uint8_t& dir, const types::SensorData& data, char waypoint);
    uint8_t metersToDistanceHand(double meters);

    float getForwardDirection(const types::SensorData& data);

    uint8_t normalize(float value, float range);

    void setHands(const types::Hands& handPositions);
    uint32_t calculatePixelColor(uint8_t pixel, const types::Hands& handPositions);
    uint8_t calculatePixelColor(uint8_t pixel, uint8_t handPosition);

    time_t refreshDuration;
    time_t handMotionDuration;
    uint8_t numPositions;

    time_t prevUpdateTime;
    time_t prevMoveTime;

    Motor bigMotor;
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Display::Display() : refreshDuration(1),
  handMotionDuration(1),
  numPositions(4),
  prevUpdateTime(0),
  prevMoveTime(0)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Display::start(time_t startTime, time_t refreshDur, time_t handMotionDur, uint8_t nPositions, const types::MotorConfig& bigMotorConfig)
{
  prevUpdateTime = startTime;
  refreshDuration = refreshDur;
  handMotionDuration = handMotionDur;
  numPositions = nPositions;

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  bigMotor.start(bigMotorConfig);

  Serial.begin(9600);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Display::process(time_t currentTime, const types::SensorData& data)
{
  // First check to see whether the final hand positions need to be updated
  if ((currentTime - prevUpdateTime) >= refreshDuration)
  {
    prevUpdateTime += refreshDuration;
  
    types::Hands handPositions;
  
    // (Hour, minute, second)
    // (North, speed, altitude)
    // (North, waypoint direction, waypoint distance)
    // (North, home direction, home distance)
    switch (data.selection)
    {
      case 0:
        {
          calculateTimeOfDay(handPositions, data);
          break;
        }
      case 1:
        {
          calculateTravelingData(handPositions, data);
          break;
        }
      case 2:
        {
          calculateWaypointReturnData(handPositions, data);
          break;
        }
      case 3:
        {
          calculateHomeReturnData(handPositions, data);
          break;
        }
      default:
        {
          Serial.println("U");
          break;
        }
    }
  
    setHands(handPositions);
  }

  // Now let the hands know it's time to move
  if ((currentTime - prevMoveTime) >= handMotionDuration)
  {
    prevMoveTime = currentTime;
    bigMotor.stepUp();
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Display::calculateTimeOfDay(types::Hands& handPositions, const types::SensorData& data)
{
  Serial.println("Time of day");
  float second = data.second;
  float minute = data.minute * 60.0f + second;
  // Hour needs to be offset by the time difference from Colorado to London. (-7 hours)
  float hour = (data.hour - 7) * 3600.0f + minute;

  handPositions.bigHand = normalize(hour, 43200.0f); // Seconds in 12 hours
  handPositions.mediumHand = normalize(minute, 3600.0f); // Seconds in 60 minutes
  handPositions.smallHand = normalize(second, 60.0f); // Seconds in 60 seconds
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Display::calculateTravelingData(types::Hands& handPositions, const types::SensorData& data)
{
  Serial.println("Traveling");
  handPositions.bigHand = calculateNorth(data);
  handPositions.mediumHand = calculateSpeed(data);
  handPositions.smallHand = calculateAltitude(data);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Display::calculateWaypointReturnData(types::Hands& handPositions, const types::SensorData& data)
{
  Serial.println("Waypoint return");
  handPositions.bigHand = calculateNorth(data);
  calculateDistanceAndDirection(handPositions.smallHand,
                                handPositions.mediumHand,
                                data,
                                Fast);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Display::calculateHomeReturnData(types::Hands& handPositions, const types::SensorData& data)
{
  Serial.println("Home return");
  handPositions.bigHand = calculateNorth(data);
  calculateDistanceAndDirection(handPositions.smallHand,
                                handPositions.mediumHand,
                                data,
                                Slow);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
uint8_t Display::calculateNorth(const types::SensorData& data)
{
  // Heading/track angle are the angle from north to forward; 
  // we want the angle from forward to north
  float angleToNorth = -1.0f * getForwardDirection(data);

  return normalize(angleToNorth, 2 * PI);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
uint8_t Display::calculateSpeed(const types::SensorData& data)
{
  float groundSpeed = data.groundSpeedKnots * 1.8519984f; // Convert to km/hr
  if (groundSpeed > 1000.0f) // Close to the speed of sound, this gets linear
  {
    groundSpeed = (groundSpeed / 14400.0f + (25.0f / 72.0f));
  }
  else if (groundSpeed >= 0.0f) // This should always be true, but just in case
  {
    // Positive part of the parametric quadratic Bezier curve defined by the 
    // points (0, 0), (100, 4.25/12), and (1000, 5/12). This curve is
    // satisfactorily linear near the edges, pleasantly curved near the middle,
    // and coincidentally passes quite close to both (100, 1/4) (important) 
    // and (50, 3/20) (bonus)
    float t = (float)((-5.0 + sqrt(25.0f + 2.0f * groundSpeed)) / 40.0);
    groundSpeed = (17.0f * t - 7.0f * t * t) / 24.0f;
  }
  else
  {
    groundSpeed = 0.0f;
  }
  
  // Switch from (0 at noon, clockwise positive) to (0 at 9:00, clockwise positive)
  groundSpeed -= 0.25f;

  return normalize(groundSpeed, 1.0f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
uint8_t Display::calculateAltitude(const types::SensorData& data)
{
  // Positive altitudes expected to be from 0 at 4:00 to 5km at 12:00
  // so scale it so that 5 km takes up 1/3 of a circle
  float altPercent = data.altitudeMeters / 15000.0f;

  // Negative altitudes should go from 0 at 4:00 to -10km at 6:00
  // Twice the altitude in half the distance is 1/4 the scale
  if (altPercent < 0.0f)
  {
    altPercent /= 4.0f;
  }

  // Switch from (0 at noon, clockwise positive) to (0 at 4:00, counterclockwise positive)
  altPercent = 1.0f/3.0f - altPercent;

  return normalize(altPercent, 1.0f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Display::calculateDistanceAndDirection(uint8_t& dist, uint8_t& dir, const types::SensorData& data, char waypoint)
{
  double toLat;
  double toLong;
  switch (waypoint)
  {
    case Fast:
      toLat = data.fastWaypointLatitude;
      toLong = data.fastWaypointLongitude;
      break;
    case Slow:
      toLat = data.slowWaypointLatitude;
      toLong = data.slowWaypointLongitude;
      break;
    default:
      toLat = 0.0;
      toLong = 0.0;
      break;
  }
  // Calculate distance using the haversine formula for calculating distances
  double fromLat = data.latitude;
  double fromLong = data.longitude;

  double a = sin((toLat - fromLat) / 2.0) * sin ((toLat - fromLat) / 2.0) +
             cos(toLat) * cos(fromLat) * sin((toLong - fromLong) / 2.0) * sin((toLong - fromLong) / 2.0);
  double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
  double d = EARTH__RADIUS * c;
  dist = metersToDistanceHand(d);

  // Calculate bearing 
  double b = atan2(sin(toLong - fromLong) * cos(toLat),
                   cos(fromLat) * sin(toLat) - sin(fromLat) * cos(toLat) * cos(toLong - fromLong));

  // Bearing is dependent on which direction we're facing
  b -= getForwardDirection(data);
                   
  dir = normalize(b, 2 * PI);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
uint8_t Display::metersToDistanceHand(double meters)
{
  // 10 m = 5% of circle
  // Each successive multiple of 10 = another 10% of circle
  // Maximum value is 70% of circle (about equal to half Earth's circumference)
  // Result can't be less than 0, since 3.17 > sqrt(10) and the calculation only
  // gets negative when meters < sqrt(10)
  double distFraction = log10(max(meters, 3.17)) * 0.1 - 0.05;

  if (distFraction > 0.7)
  {
    distFraction = 0.7;
  }

  // Switch from (0 at noon, clockwise positive) to (0 at noon, counterclockwise positive)
  distFraction *= -1;
  return normalize(distFraction, 1.0f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
float Display::getForwardDirection(const types::SensorData& data)
{
  // If we're going fast enough, assume we're facing forward. GPS tracker is
  // likely more accurate than the compass at those speeds (especially because
  // the compass is likely to be surrounded by ferrous metal).
  // 10 knots is sort of close to a sensible marathon running speed.

  if (data.groundSpeedKnots < 10.0f)
  {
    return data.heading;
  }
  else
  {
    return data.trackAngle;
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
uint8_t Display::normalize(float value, float range)
{
  // Constrain to [0, range)
  while (value >= range)
  {
    value -= range;
  }
  while (value < 0.0f)
  {
    value += range;
  }

  // Scale from [0, range) to [0, POCKETWATCH__DISPLAY__NUMPOSITIONS)
  return (uint8_t)(value * POCKETWATCH__DISPLAY__NUMPOSITIONS / range);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Display::setHands(const types::Hands& handPositions)
{
  Serial.print("Big hand: ");
  Serial.print(handPositions.bigHand);
  Serial.print("; Medium hand: ");
  Serial.print(handPositions.mediumHand);
  Serial.print("; Small hand: ");
  Serial.print(handPositions.smallHand);
  Serial.println(" ");

  for (uint8_t p = 0; p < 16; ++p)
  {
    strip.setPixelColor((15 - p), calculatePixelColor(p, handPositions));
//    strip.setPixelColor(p, strip.Color(32, 32, 32));
  }

  strip.setPixelColor(16, strip.Color(16, 16, 16));
  strip.show();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
uint8_t distance(uint8_t pointA, uint8_t pointB)
{
  uint8_t big = max(pointA, pointB);
  uint8_t small = min(pointA, pointB);

  if (big - small <= POCKETWATCH__DISPLAY__NUMPOSITIONS/2)
  {
    return (big - small);
  }
  else
  {
    small += POCKETWATCH__DISPLAY__NUMPOSITIONS/2;
    big -= (POCKETWATCH__DISPLAY__NUMPOSITIONS - POCKETWATCH__DISPLAY__NUMPOSITIONS/2);

    return (small - big);
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
uint32_t Display::calculatePixelColor(uint8_t pixel, const types::Hands& handPositions)
{
  return strip.Color(calculatePixelColor(pixel, handPositions.bigHand),
                     calculatePixelColor(pixel, handPositions.mediumHand),
                     calculatePixelColor(pixel, handPositions.smallHand));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
uint8_t Display::calculatePixelColor(uint8_t pixel, uint8_t handPosition)
{
  uint8_t dist = distance(pixel * POCKETWATCH__DISPLAY__NUMPOSITIONS / 16.0f, handPosition);
  switch(dist)
  {
    case 0:
      return 255;
      break;
    case 1:
      return 213;
      break;
    case 2:
      return 175;
      break;
    case 3:
      return 142;
      break;
    case 4:
      return 114;
      break;
    case 5:
      return 89;
      break;
    case 6:
      return 68;
      break;
    case 7:
      return 50;
      break;
    case 8:
      return 36;
      break;
    case 9:
      return 25;
      break;
    case 10:
      return 16;
      break;
    case 11:
      return 10;
      break;
    case 12:
      return 5;
      break;
    case 13:
      return 2;
      break;
    case 14:
      return 1;
      break;
    case 15:
      return 1;
      break;
    default:
      return 0;
      break;
  }
}

} // end namespace pocketwatch

#endif
