#ifndef POCKETWATCH_BLINKER_H
#define POCKETWATCH_BLINKER_H

namespace pocketwatch
{

typedef unsigned long time_t;

class Blinker
{
public:
  Blinker();

  void start(uint8_t ledPin, time_t startTime, time_t onDur, time_t offDur);
  void process(time_t currentTime);

private:

  const char Off = 0;
  const char On = 1;

  void turnOff();
  void turnOn();

  uint8_t pin;
  time_t onDuration;
  time_t offDuration;
  
  time_t prevStateChangeTime;
  char state;

};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Blinker::Blinker() : pin(0),
                     onDuration(1000),
                     offDuration(1000),
                     prevStateChangeTime(0),
                     state(Blinker::Off)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Blinker::start(uint8_t ledPin, time_t startTime, time_t onDur, time_t offDur)
{
  pin = ledPin;
  prevStateChangeTime = startTime;
  onDuration = onDur;
  offDuration = offDur;

  pinMode(pin, OUTPUT);
  
  turnOff(); 
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Blinker::process(time_t currentTime)
{
  if ((state == Blinker::Off) && ((currentTime - prevStateChangeTime) >= offDuration))
  {
    prevStateChangeTime += offDuration;
    turnOn();
  }
  else if ((state == Blinker::On) && ((currentTime - prevStateChangeTime) >= onDuration))
  {
    prevStateChangeTime += onDuration;
    turnOff();
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Blinker::turnOff()
{
  digitalWrite(pin, 0);
  state = Blinker::Off;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Blinker::turnOn()
{
  digitalWrite(pin, 1);
  state = Blinker::On;
}
} // end namespace pocketwatch

#endif
