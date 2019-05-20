#ifndef __PINCONFIG_H__
#define __PINCONFIG_H__

//--------------------------------------------Pins--------------------------------------------------
#define enablePin 12 //enable= low
#define ms1 14 //micro stepping
#define ms2 27 //micro stepping
#define ms3 26 //micro stepping all on for 16th step
#define resetPin 25 // reset stepper controller by pulling low
#define sleepPin 33
#define stepPin 32
#define dirPin 21//35
#define faultPin 34 // low indicate fault on stepper controller (over temperature, over current/short)

#define dispenseButton 39 //cycle start button
#define servo 13 // servo signal
#define encoderPin 36 // screw encoder

#endif // __PINCONFIG_H__
