#ifndef POCKETWATCH_WAYPOINTKEEPER_H
#define POCKETWATCH_WAYPOINTKEEPER_H

#include <EEPROM.h>

#define DBL_SIZE 8
#define POCKETWATCH__WAYPOINTKEEPER__EEPROMSTART 0
#define POCKETWATCH__WAYPOINTKEEPER__FASTLAT (POCKETWATCH__WAYPOINTKEEPER__EEPROMSTART + DBL_SIZE * 0)
#define POCKETWATCH__WAYPOINTKEEPER__FASTLON (POCKETWATCH__WAYPOINTKEEPER__EEPROMSTART + DBL_SIZE * 1)
#define POCKETWATCH__WAYPOINTKEEPER__SLOWLAT (POCKETWATCH__WAYPOINTKEEPER__EEPROMSTART + DBL_SIZE * 2)
#define POCKETWATCH__WAYPOINTKEEPER__SLOWLON (POCKETWATCH__WAYPOINTKEEPER__EEPROMSTART + DBL_SIZE * 3)


namespace pocketwatch
{

typedef unsigned long time_t;

class WaypointKeeper
{
public:
  WaypointKeeper();

  void start(int buttonPin, 
             time_t startTime, 
             time_t minReleaseDur, 
             time_t maxReleaseDur, 
             time_t fastHoldDur,
             time_t slowHoldDur,
             uint8_t numFastHolds,
             uint8_t numSlowHolds);
  void process(time_t currentTime, double lat, double lon);

  double getFastLat() const { return getVal(POCKETWATCH__WAYPOINTKEEPER__FASTLAT); }
  double getFastLon() const { return getVal(POCKETWATCH__WAYPOINTKEEPER__FASTLON); }
  double getSlowLat() const { return getVal(POCKETWATCH__WAYPOINTKEEPER__SLOWLAT); }
  double getSlowLon() const { return getVal(POCKETWATCH__WAYPOINTKEEPER__SLOWLON); }

private:

  double getVal(uint16_t startAddress) const;
  void setVal(uint16_t startAddress, double value);

  uint8_t pin;

  double storedLatitude;
  double storedLongitude;

  time_t minReleaseDuration;
  time_t maxReleaseDuration;
  time_t fastHoldDuration;
  uint8_t numFastHoldsNeeded;
  time_t slowHoldDuration;
  uint8_t numSlowHoldsNeeded;

  uint8_t numFastHoldsMade;
  uint8_t numSlowHoldsMade;

  time_t lastUpdateTime;

  bool buttonWasPressed;

};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WaypointKeeper::WaypointKeeper() : pin(0),
                                   storedLatitude(0.0),
                                   storedLongitude(0.0),
                                   minReleaseDuration(1),
                                   maxReleaseDuration(1),
                                   fastHoldDuration(1),
                                   numFastHoldsNeeded(1),
                                   slowHoldDuration(1),
                                   numSlowHoldsNeeded(1),
                                   numFastHoldsMade(0),
                                   numSlowHoldsMade(0),
                                   lastUpdateTime(0),
                                   buttonWasPressed(false)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WaypointKeeper::start(int buttonPin, 
                           time_t startTime,
                           time_t minReleaseDur, 
                           time_t maxReleaseDur, 
                           time_t fastHoldDur,
                           time_t slowHoldDur,
                           uint8_t numFastHolds,
                           uint8_t numSlowHolds)
{
  pin = buttonPin;
  minReleaseDuration = minReleaseDur;
  maxReleaseDuration = maxReleaseDur;
  fastHoldDuration = fastHoldDur;
  slowHoldDuration = slowHoldDur;
  numFastHoldsNeeded = numFastHolds;
  numSlowHoldsNeeded = numSlowHolds;
  
  pinMode(pin, INPUT);

  buttonWasPressed = (digitalRead(pin) == HIGH);
  if (buttonWasPressed)
  {
    lastUpdateTime = startTime;
  }

//  // Denver Botanic Gardens
//  setVal(POCKETWATCH__WAYPOINTKEEPER__FASTLAT, 0.69345851663);
//  setVal(POCKETWATCH__WAYPOINTKEEPER__FASTLON, -1.83196589508);
//
//  // North Umpqua River
//  setVal(POCKETWATCH__WAYPOINTKEEPER__SLOWLAT, 0.75580837485);
//  setVal(POCKETWATCH__WAYPOINTKEEPER__SLOWLON, -2.15131453045);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WaypointKeeper::process(time_t currentTime, double lat, double lon)
{
  bool buttonIsPressedNow = (digitalRead(pin) == HIGH);

  if (buttonIsPressedNow && buttonWasPressed)
  {
    time_t pressedDuration = currentTime - lastUpdateTime;
    if ((pressedDuration >= slowHoldDuration) &&
        (numSlowHoldsMade + 1 >= numSlowHoldsNeeded) &&
        (numFastHoldsMade == 0))
    {
      setVal(POCKETWATCH__WAYPOINTKEEPER__SLOWLAT, storedLatitude);
      setVal(POCKETWATCH__WAYPOINTKEEPER__SLOWLON, storedLongitude);

      numSlowHoldsMade = 0;
    }
    else if ((pressedDuration >= fastHoldDuration) &&
             (numFastHoldsMade + 1 >= numFastHoldsNeeded) &&
             (numSlowHoldsMade == 0))
    {
      setVal(POCKETWATCH__WAYPOINTKEEPER__FASTLAT, storedLatitude);
      setVal(POCKETWATCH__WAYPOINTKEEPER__FASTLON, storedLongitude);

      numFastHoldsMade = 0;
    }
  }
  else if (buttonIsPressedNow && ( ! buttonWasPressed))
  {
    // The button has just been pressed.
    time_t unpressedDuration = currentTime - lastUpdateTime;
    if (numFastHoldsMade == 0 && numSlowHoldsMade == 0)
    {
      // We're starting a new sequence - this is where we want the waypoint.
      storedLatitude = lat;
      storedLongitude = lon;
    }
    else if ((unpressedDuration < minReleaseDuration) || 
             (unpressedDuration > maxReleaseDuration))
    {
      // We had been in a sequence already, but we messed up. Reset the counts.
      numFastHoldsMade = 0;
      numSlowHoldsMade = 0;
    }
    else // We're in a sequence and have not messed up yet.
    {
      // Nothing interesting happens in here. We just don't reset anything.
    }
    lastUpdateTime = currentTime;
  }
  else if (( ! buttonIsPressedNow) && buttonWasPressed)
  {
    // The button has just been released.
    time_t pressedDuration = currentTime - lastUpdateTime;
    // If this was a slow hold and we're not already on a fast hold sequence
    if ((pressedDuration >= slowHoldDuration) && 
        (numFastHoldsMade == 0))
    {
      ++numSlowHoldsMade;
      numFastHoldsMade = 0;
    }
    // If this was a fast hold and we're not already on a slow hold sequence
    else if ((pressedDuration >= fastHoldDuration) &&
             (numSlowHoldsMade == 0))
    {
      ++numFastHoldsMade;
      numSlowHoldsMade = 0;
    }
    else
    {
      // Either the button was not held for long enough, or we messed up the
      // sequence. Reset the counts.
      numFastHoldsMade = 0;
      numSlowHoldsMade = 0;
    }
    lastUpdateTime = currentTime;
  }

  // Set this for the next time through
  buttonWasPressed = buttonIsPressedNow;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double WaypointKeeper::getVal(uint16_t startAddress) const
{
  byte eepromBytes[DBL_SIZE];

  for (uint8_t i = 0; i < DBL_SIZE; ++i)
  {
    eepromBytes[i] = EEPROM.read(startAddress + i);
  }

  return *((double*)(&eepromBytes));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WaypointKeeper::setVal(uint16_t startAddress, double value)
{
  byte* valueBytes = (byte*)(&value);

  for (uint8_t i = 0; i < DBL_SIZE; ++i)
  {
    EEPROM.write(startAddress + i, valueBytes[i]);
  }

}

} // end namespace pocketwatch

#endif
