#ifndef POCKETWATCH_SELECTOR_H
#define POCKETWATCH_SELECTOR_H

#define NUM_ANALOG_BINS 1024

namespace pocketwatch
{

typedef unsigned long time_t;

class Selector
{
public:
  Selector();

  void start(uint8_t inputPin, time_t startTime, time_t refreshTime, time_t delayTime, uint8_t nChoices, float ratio);
  void process(time_t currentTime);

  uint8_t getChoice() const { return choice; }

private:

  uint8_t readChoice() const;

  uint8_t pin;

  uint8_t binsPerChoice;
  time_t stateChangeDelayTime;
  time_t inputRefreshTime;
  
  time_t prevStateChangeTime;
  uint8_t choice;
  time_t prevReadTime;
  uint8_t inputChoice;

};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Selector::Selector() : pin(0),
                       binsPerChoice(1),
                       stateChangeDelayTime(0),
                       inputRefreshTime(0),
                       prevStateChangeTime(0),
                       choice(0),
                       prevReadTime(0),
                       inputChoice(0)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Selector::start(uint8_t inputPin,
                     time_t startTime,
                     time_t refreshTime,
                     time_t delayTime,
                     uint8_t nChoices,
                     float ratio)
{
  pin = inputPin;
  prevStateChangeTime = startTime;
  prevReadTime = startTime;
  binsPerChoice = (uint8_t)((1.0 - ratio) * NUM_ANALOG_BINS / nChoices);
  if (binsPerChoice == 0)
  {
    binsPerChoice = 1;
  }
  stateChangeDelayTime = delayTime;
  inputRefreshTime = refreshTime;

  pinMode(pin, INPUT);

  inputChoice = readChoice();

  choice = inputChoice;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Selector::process(time_t currentTime)
{
  if ((currentTime - prevReadTime) >= inputRefreshTime)
  {
    prevReadTime = currentTime;
    uint8_t c = readChoice();
    if (c != inputChoice)
    {
      inputChoice = c;
      prevStateChangeTime = currentTime;
    }
    else if ((currentTime - prevStateChangeTime) >= stateChangeDelayTime)
    {
      choice = inputChoice;
    }
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
uint8_t Selector::readChoice() const
{
  // analogRead is expected to return either a very high number (if the
  // potentiometer is closer to Vcc) or a very low number (if it's closer to
  // ground).
  int value = analogRead(pin);

  // This will keep low numbers low, and change high numbers into small negative
  // numbers and then into small positive numbers.
  if (value >= NUM_ANALOG_BINS/2)
  {
    value -= NUM_ANALOG_BINS;
    value = abs(value);
  }
  value /= binsPerChoice;

  return (uint8_t)value;
}


} // end namespace pocketwatch

#endif
