#ifndef _CODING_H_
#define _CODING_H_

#include "syringelist.h"

extern syringeType syringe;

void doZero();
void resetAll();
void prime();
void switchValve(uint8_t pos);
void emptySyringe();
void moveUp();
void moveDown();
void stopMove();
void settingMode();
void titrateMode();
void dispenseMode();
void manualMode();
void homeMode();
void settingsDone(float tmpDispenseVol, uint32_t tmpPrimeVol, uint32_t tmpPrimeCycles,
                  uint32_t tmpSpeed, uint32_t tmpLowSpeed);


#endif // _CODING_H_
