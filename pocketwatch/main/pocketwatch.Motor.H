#ifndef POCKETWATCH_MOTOR_H
#define POCKETWATCH_MOTOR_H


#include "pocketwatch.Types.H"

namespace pocketwatch
{

typedef unsigned long time_t;

class Motor
{
public:
  Motor();

  void start(const types::MotorConfig& c);

  void sleep();
  void wake();
  void stepUp();
  void stepDown();

private:

  struct Coil
  {
    Coil();

    void turnUp();
    void turnDown();
    void turnOff();
    
    uint8_t pin1;
    uint8_t pin2;
  };

  void setState(uint8_t s);

  Coil coilA;
  Coil coilB;

  uint8_t state;

};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Motor::Motor() : coilA(),
                 coilB(),
                 state(0)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Motor::start(const types::MotorConfig& c)
{
  coilA.pin1 = c.pinA1;
  coilA.pin2 = c.pinA2;
  coilB.pin1 = c.pinB1;
  coilB.pin2 = c.pinB2;

  pinMode(coilA.pin1, OUTPUT);
  pinMode(coilA.pin2, OUTPUT);
  pinMode(coilB.pin1, OUTPUT);
  pinMode(coilB.pin2, OUTPUT);

  setState(0);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Motor::sleep()
{
  coilA.turnOff();
  coilB.turnOff();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Motor::wake()
{
  // Set state to what it already should be
  setState(state);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Motor::stepUp()
{
  setState((state + 1) & 0x7);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Motor::stepDown()
{
  setState((state - 1) & 0x7);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Motor::setState(uint8_t s)
{
  //    A  DOWN OFF  UP
  // B     
  // UP    7    0    1
  // OFF   6    x    2
  // DOWN  5    4    3
  // 
  state = s;
  switch(state)
  {
    case 0:
      coilA.turnOff();
      coilB.turnUp();
      break;
    case 1:
      coilA.turnUp();
      coilB.turnUp();
      break;
    case 2:
      coilA.turnUp();
      coilB.turnOff();
      break;
    case 3:
      coilA.turnUp();
      coilB.turnDown();
      break;
    case 4:
      coilA.turnOff();
      coilB.turnDown();
      break;
    case 5:
      coilA.turnDown();
      coilB.turnDown();
      break;
    case 6:
      coilA.turnDown();
      coilB.turnOff();
      break;
    case 7:
      coilA.turnDown();
      coilB.turnUp();
      break;
    default:
      coilA.turnOff();
      coilB.turnOff();
      break;
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Motor::Coil::Coil() : pin1(0),
                      pin2(0)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Motor::Coil::turnUp()
{
  digitalWrite(pin2, LOW);
  digitalWrite(pin1, HIGH);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Motor::Coil::turnDown()
{
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Motor::Coil::turnOff()
{
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, HIGH);
}

} // end namespace pocketwatch

#endif
