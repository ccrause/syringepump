#ifndef __PUMPDRIVER_H__
#define __PUMPDRIVER_H__

#include "AccelStepper.h"
#include <TMCStepper.h>

#define stPmm 800 //800; //steps per mm
#define _maxSpeed 10 // mm/s
#define accelRamp (_maxSpeed*stPmm/3)
//#define maxAcceleration maxSpeed / 2  // mm/s/s


//extern int32_t speed;

class myAccelStepper: public AccelStepper{
public:
  myAccelStepper(uint8_t interface = AccelStepper::FULL4WIRE, uint8_t pin1 = 2, uint8_t pin2 = 3, uint8_t pin3 = 4,
     uint8_t pin4 = 5, bool enable = true)
     :AccelStepper(interface, pin1, pin2, pin3, pin4, enable){};

  boolean runSpeed();
  void reset();
  float getAcceleration();
  void lowSpeedSettings();
  void highSpeedSettings();

  int32_t speed_mm_s;
  volatile uint16_t stepper_count;
  volatile bool running;
  volatile bool moveToPosition;
  volatile bool trip;
  int lSG;  // low speed stallGuard setting
  int hSG;  // high speed stallGuard setting
};

extern myAccelStepper motor;
extern TMC2130Stepper driver;

void initStepperRunner();

#endif
