#ifndef _NEXTIONINTERFACE_
#define _NEXTIONINTERFACE_

#include "Nextion.h"

/*extern NexPage mainPage;
extern NexPage settingsPage;
extern NexPage manualPage;
extern NexPage tripPage;

// Page 0 (Home)
extern NexButton primeButton;
extern NexButton emptyButton;
extern NexButton settingsButtonP0;
extern NexButton manualButtonP0;
extern NexDSButton valvePosition0;
extern NexProgressBar progressBar0;
//extern NexText statusText;
extern NexText volumeText;
extern NexText errMsg0;

//Page 1 (Settings)
extern NexText errMsg1;
extern NexText nexDispenseVol;
extern NexNumber nexPrimeVol;
extern NexNumber nexPrimeCycles;
extern NexNumber nexSpeedPct;
extern NexButton manualButtonP1;
extern NexButton homeButtonP1;
extern NexNumber nexMaxVolLimit;

//Page 2 (manual control)
extern NexDSButton valvePosition1;
extern NexDSButton switchValveButton;
extern NexButton upButton;
extern NexButton downButton;
extern NexButton homeButtonP2;
extern NexButton settingsButtonP2;
extern NexProgressBar progressBar1;
extern NexText errMsg2;
extern NexText volumeTextP2;

//Page 3 (TRIP Screen)
extern NexButton resetSystemButton;
*/

// Functional interface to Nextion GUI elements
void processNexMessages();
void updateStatusTxt(const char status[]);
void updateErrorTxt(const char err[]);
void updateVolumeTxt(const char txt[]);
void updateVolumeTxt2NoAck(float vol);

void updateValveDisplay(uint8_t pos);
void updateProgressbar0(uint32_t pos);
void updateProgressbar2(uint32_t pos);
void updateProgressbar2NoAck(uint32_t pos);
void updateSettings(float tmpDispenseVol, uint32_t tmpPrimeVol, uint32_t tmpPrimeCycles, uint32_t tmpSpeed);
void setNexMaxVolLimit(uint32_t limit);

void initNextionInterface();
void nexTripAlert();
void nexReset();

#endif // _NEXTIONINTERFACE_
