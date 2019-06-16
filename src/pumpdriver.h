#ifndef __PUMPDRIVER_H__
#define __PUMPDRIVER_H__

#include "AccelStepper.h"
#include <TMCStepper.h>

class myAccelStepper: public AccelStepper{
public:
  myAccelStepper(uint8_t interface = AccelStepper::FULL4WIRE, uint8_t pin1 = 2, uint8_t pin2 = 3, uint8_t pin3 = 4,
     uint8_t pin4 = 5, bool enable = true)
     :AccelStepper(interface, pin1, pin2, pin3, pin4, enable){};

  boolean runSpeed();
  void reset();
  void setCurrent(uint16_t mA);

  volatile uint16_t stepper_count;
  volatile bool running;
  volatile bool moveToPosition;
  volatile bool trip;
};

extern myAccelStepper motor;
extern TMC2130Stepper driver;

void initStepperRunner();

#endif
