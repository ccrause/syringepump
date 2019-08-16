#ifndef _NEXTIONINTERFACE_
#define _NEXTIONINTERFACE_

#include "Nextion.h"

extern float currentPlungerPosition, currentSyringeVolume;

// Functional interface to Nextion GUI elements
void processNexMessages();
void updateStatusTxt(const char status[]);
void updateStatusTxt0(const char status[]);
void updateStatusTxt1(const char status[]);
void updateStatusTxt2(const char status[]);
void updateStatusTxt3(const char status[]);
void updateStatusTxt4(const char status[]);

void updateErrorTxt(const char err[]);
void updateErrorTxt0(const char err[]);
void updateErrorTxt1(const char err[]);
void updateErrorTxt2(const char err[]);
void updateErrorTxt4(const char err[]);

void updateVolumeTxt(const char txt[]);
void updateVolumeTxt0(const char txt[]);
void updateVolumeTxt1(const char txt[]);
void updateVolumeTxt2(const char txt[]);
void updateVolumeTxt4(const char txt[]);
void updateVolumeTxt2NoAck(float vol);
void updateVolumeTxt4NoAck(float vol);

void updateValveDisplay(uint8_t pos);
void updateProgressbarHome(uint32_t pos);
void updateProgressbarDispense(uint32_t pos);
void updateProgressbarTitrate(uint32_t pos);
void updateProgressbarManual(uint32_t pos);

void updateSettingsDisplay(float tmpDispenseVol, uint32_t tmpPrimeVol, uint32_t tmpPrimeCycles,
                    uint32_t tmpHighSpeed, uint32_t tmpLowSpeed);
void setNexMaxVolLimit(uint32_t limit);

void initNextionInterface();
void nexTripAlert();
void nexReset();

void nexDisableScreen();
void nexEnableScreen();

#endif // _NEXTIONINTERFACE_
