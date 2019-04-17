#ifndef __PINCONFIG_H__
#define __PINCONFIG_H__

//--------------------------------------------Pins--------------------------------------------------
// #define enable 12
#define ms1 14 //micro stepping
#define ms2 27 //micro stepping
#define ms3 26 //micro stepping all on for 16th step
#define stepPin 25
#define dirPin 33
#define resetPin 34 // reset stepper controller by pulling low
#define faultPin 39 // low indicate fault on stepper controller (over temperature, over current/short)

#define dispenseButton 35 //cycle start button
#define servo 13 // servo signal
#define encoderPin 32 // screw encoder

#endif // __PINCONFIG_H__
