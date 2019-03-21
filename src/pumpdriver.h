#ifndef __PUMPDRIVER_H__
#define __PUMPDRIVER_H__

#include "AccelStepper.h"

//extern const long stPmm; //800; //steps per mm
//extern long strokePosLimit; // max stroke position in steps (default 100 mm)
//extern long dosingStroke;
//extern bool updateVol;
//extern bool displayDispenseProgress;

// motorIsRunning should block other tasks from accessing motor related parameters
// except for reading motor.currentPosition & motor.distanceToGo
extern volatile bool motorIsRunning;
extern volatile bool moveToPosition;
extern volatile uint16_t stepper_count;
extern volatile bool trip;


// -------------------------------------------STEPPER--------------------------------------------------
// AccelStepper Setup Steppername (type, StepPin, DirectionPin)
extern AccelStepper motor;

void initStepperRunner();

#endif
