#ifndef _CODING_H_
#define _CODING_H_

void resetAll();
void prime();
void switchValve(uint8_t pos);
void emptySyringe();
void moveUp();
void moveDown();
void stopMove();
void settingMode();
void manualMode();
void settingsDone(float tmpDispenseVol, uint32_t tmpPrimeVol, uint32_t tmpPrimeCycles, uint32_t tmpSpeed);


#endif // _CODING_H_
