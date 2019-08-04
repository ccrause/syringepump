#ifndef _NEXTIONINTERFACE_
#define _NEXTIONINTERFACE_

#include "Nextion.h"

// Functional interface to Nextion GUI elements
void processNexMessages();
void updateStatusTxt(const char status[]);
void updateErrorTxt(const char err[]);
void updateVolumeTxt(const char txt[]);
void updateVolumeTxt2NoAck(float vol);

void updateValveDisplay(uint8_t pos);
void updateProgressbarHome(uint32_t pos);
void updateProgressbarTitrate(uint32_t pos);
void updateProgressbarTitrateNoAck(uint32_t pos);
void updateSettings(float tmpDispenseVol, uint32_t tmpPrimeVol, uint32_t tmpPrimeCycles, uint32_t tmpSpeed);
void setNexMaxVolLimit(uint32_t limit);

void initNextionInterface();
void nexTripAlert();
void nexReset();

void nexDisableScreen();
void nexEnableScreen();

#endif // _NEXTIONINTERFACE_
