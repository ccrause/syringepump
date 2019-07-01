#include "Nextion.h"
#include "nextioninterface.h"
#include "txtmessages.h"
#include"coding.h"

#include <errno.h>

NexPage mainPage(0, 0, "page0"); // Home Screen
NexPage settingsPage(1, 0, "page1"); // Settings Screen
NexPage manualPage(2, 0, "page2"); // Manual Control Screen
NexPage tripPage(3, 0, "page3"); // Stall / Tripp and reset Screen

// Page 0 (Home)
NexButton primeButton(0, 1, "b0"); //Prime Syringe
NexButton emptyButton(0, 2, "b1"); // Empty Syringe
NexButton settingsButtonP0(0, 3, "b2"); //Page1 not used in mcu
NexButton manualButtonP0(0, 9, "b3"); //Page1 not used in mcu
NexDSButton valvePosition0(0, 4, "bt0"); // Actual Valve Position 0=IN 1=Out
NexProgressBar progressBar0(0, 5, "j0"); //Syringe Slider 0=100% Full
NexText statusText(0, 6, "t0"); //Status Text Ready, Running, Filling. Error
NexText volumeText(0, 7, "t1"); //Current Syringe Volume
NexText errMsg0(0, 8, "t2"); //Error Display 28 Carracters max

//Page 1 (Settings)
NexText errMsg1(1, 3, "t4"); //Error Display 28 Carracters max
NexText nexDispenseVol(1, 2, "t0"); //Set Volume
NexNumber nexPrimeVol(1, 5, "n1"); //global vairable to limit stkokeNumber must be updated on startup
NexNumber nexPrimeCycles(1, 7, "n2"); //Number of cycles to prime
NexNumber nexSpeedPct(1, 4, "n0"); // % of max speed
NexButton manualButtonP1(1, 8, "b1"); // Navigate to manual page
NexButton homeButtonP1(1, 1, "b0"); // Home Page button update the values for the syringe by reading the vairious values
NexNumber nexMaxVolLimit(1, 9, "maxStroke"); //global variable to limit stkokeNumber must be updated on startup

//Page 2 (manual control)
NexDSButton valvePosition1(2, 1, "bt0"); // Actual Valve Position 0=In 1=Out
NexDSButton switchValveButton(2, 2, "bt1"); // Switch Valve Position 0=In 1=Out
NexButton upButton(2, 3, "b0"); // Move syringe UP
NexButton downButton(2, 4, "b1"); // Move Syringe Down
NexButton homeButtonP2(2, 5, "b2"); // Home Page button
NexButton settingsButtonP2(2, 6, "b3"); //Page1 not used in mcu
NexProgressBar progressBar2(2, 7, "j0"); //Syringe Slider 0=100% Full
NexText errMsg2(2, 8, "t0"); // Errror Diplay 15 caracters max
NexText volumeTextP2(2, 9, "t1"); //Current Syringe Volume

//Page 3 (TRIP Screen)
NexButton resetSystemButton(3, 1, "b0"); //Page1 not used in mcu

// Buttons that have a executable action assigned to them
NexTouch *nex_listen_list[] = {
  &primeButton,
  &homeButtonP1,
  &homeButtonP2,
  &switchValveButton,
  &emptyButton,
  &upButton,
  &downButton,
  &settingsButtonP0,
  &settingsButtonP2,
  &manualButtonP0,
  &manualButtonP1,
  &resetSystemButton,
  NULL
};

//buffer to read values from Nextion
char buffer[30] = {0};

bool getDispenseVolume(float *val);
uint32_t getPrimeVolume(void);
uint32_t getSpeed(void);
uint32_t getPrimeCycles(void);

void processNexMessages(){
  nexLoop(nex_listen_list);
}

void primeButtonPopCallBack(void *ptr){
  Serial.println("primeButtonPopCallBack");
  prime();
}

// Check settings, if anything is invalid, return to setings page
// and display some error messge
// If OK, proceed to next page
bool checkSettings(void){
  delay(100); // hack to try and read text, perhaps nextion is slow in processing requests?

  float tmpDispenseVol;
  if(getDispenseVolume(&tmpDispenseVol) == false){
    sendCommand("page 1");
    recvRetCommandFinished(100);
    errMsg1.setText("Error converting");
    return false;
  }
  else{
    if(tmpDispenseVol > 3*syringeInfo[syringe].vol) {
      Serial.println("Set volume exceeded");
      sendCommand("page 1");
      recvRetCommandFinished(100);
      errMsg1.setText("Set volume exceeded");
      char correctedVol[6];
      itoa(3*syringeInfo[syringe].vol, &correctedVol[0], 10);
      nexDispenseVol.setText(correctedVol);
      return false;
    }
  }
  uint32_t tmpPrimeVol = getPrimeVolume();
  uint32_t tmpPrimeCycles = getPrimeCycles();
  uint32_t tmpSpeed = getSpeed();

  settingsDone(tmpDispenseVol, tmpPrimeVol, tmpPrimeCycles, tmpSpeed);
  return true;
}

void settingsButtonPopCallBack(void *ptr) {
  clearManualMode();
  settingMode();
}

void manualButton0PopCallBack(void *ptr) {
  manualMode();
}

// Page 1 Home button
void homeButton1PopCallBack(void *prt_){
  if (checkSettings()){
    clearSettingMode();
  }
  else{
    settingMode();
  }
}

void manualButton1PopCallBack(void *ptr) {
  if (checkSettings()){
    clearSettingMode();
    manualMode();
  }
  else{
    settingMode();
  }
}

// Page 2 home button
void homeButton2PopCallBack(void *prt_){
  clearManualMode();
}


void resetSystemButtonPopCallback(void *ptr_) {
  resetAll();
}

//actuate the three way valve invert the current position
void switchValveButtonPopCallBack(void *prt){
  uint32_t dual_state;
  switchValveButton.getValue(&dual_state);
  if (!dual_state) {
    switchValve(0);
  }
  else {
    switchValve(1);
  }
}

// empty the syringe by moving plunger up to zero volume
void emptyButtonPopCallBack(void *prt){
  emptySyringe();
}

// move the plunger up 5mm if possible
void upButtonPushCallback(void *prt){
  moveUp();
}

void downButtonPushCallback(void *prt){
  moveDown();
}

void up_downButtonPopCallback(void *prt){
  stopMove();
}

// Interface to display elements
void updateStatusTxt(const char status[]){
  statusText.setText(status);
}

void updateErrorTxt(const char err[]){
  errMsg0.setText(err);
  errMsg1.setText(err);
  errMsg2.setText(err);
}

void updateVolumeTxt(const char txt[]){
  volumeText.setText(txt);
  volumeTextP2.setText(txt);
}

void updateVolumeTxt2NoAck(float vol){
  const char sendCmdTxt[] = "page2.t1.txt=\"%.3f\"\xff\xff\xff";
  sprintf(buffer, sendCmdTxt, vol);
  nexSerial.write(buffer);
}

void updateValveDisplay(uint8_t pos){
  valvePosition0.setValue(pos);  //update valve p0
  valvePosition1.setValue(pos); //update valve p1
  switchValveButton.setValue(pos);  //update valve switch p1
}

void updateProgressbar0(uint32_t pos){
  progressBar0.setValue(pos);
}

// Send command but do not read response
// This leaves Nextion response in serial buffer
// This response will then be removed by nexLoop without interfering with the nexLoop logic.
void updateProgressbar2NoAck(uint32_t pos){
  const char sendCmdVal[] = "page2.j0.val=%d\xff\xff\xff";
  sprintf(buffer, sendCmdVal, pos);
  nexSerial.write(buffer);
}

void updateProgressbar2(uint32_t pos){
  progressBar2.setValue(pos);
}

void updateSettings(float tmpDispenseVol, uint32_t tmpPrimeVol, uint32_t tmpPrimeCycles, uint32_t tmpSpeed){
  sprintf(buffer, "%.3f", tmpDispenseVol);
  nexDispenseVol.setText(buffer);
  nexPrimeVol.setValue(tmpPrimeVol);
  nexPrimeCycles.setValue(tmpPrimeCycles);
  nexSpeedPct.setValue(tmpSpeed);
}

void setNexMaxVolLimit(uint32_t limit){
  nexMaxVolLimit.setValue(limit);
}

void initNextionInterface(){
  //register the pop events
  primeButton.attachPop(primeButtonPopCallBack);
  switchValveButton.attachPop(switchValveButtonPopCallBack);
  emptyButton.attachPop(emptyButtonPopCallBack);
  upButton.attachPush(upButtonPushCallback);
  upButton.attachPop(up_downButtonPopCallback);
  downButton.attachPush(downButtonPushCallback);
  downButton.attachPop(up_downButtonPopCallback);
  homeButtonP1.attachPop(homeButton1PopCallBack);
  homeButtonP2.attachPop(homeButton2PopCallBack);
  settingsButtonP0.attachPop(settingsButtonPopCallBack);
  settingsButtonP2.attachPop(settingsButtonPopCallBack);
  manualButtonP0.attachPop(manualButton0PopCallBack);
  manualButtonP1.attachPop(manualButton1PopCallBack);
  resetSystemButton.attachPop(resetSystemButtonPopCallback);
}

void nexTripAlert(){
  Serial.println(tripMessage);
  sendCommand("page 3"); // page3.show() doesn't work??
}

void nexReset(){
  sendCommand("rest");
}

// Errors result in return to settings page
bool getDispenseVolume(float *val) {
  if(debugPrint) Serial.println("Get dispense volume from Nextion");
  memset(buffer, 0, sizeof(buffer));
  if(!nexDispenseVol.getText(buffer, sizeof(buffer))){
    Serial.println("Error reading volume.");
    return false;
  }
  else{
    char *endpointer;
    errno = 0;
    *val = strtof(buffer, &endpointer);
    if(*endpointer != '\0' || errno !=0){
      if(debugPrint) Serial.println("Error converting dispense volume from text.");
      return false;
    }
    else{
      if(debugPrint) Serial.printf("Volume = %f\n", *val);
      return true;
    }
  }
}

uint32_t getPrimeVolume(void) {
  if(debugPrint) Serial.println("Get priming volume from Nextion");

  uint32_t temp = 0;
  if(!nexPrimeVol.getValue(&temp)) {
    Serial.println("Error reading priming volume.");
    return 0;
  }
  else{
    if(debugPrint) Serial.printf("Priming volume = %d\n", temp);
    return temp;
  }
}

uint32_t getSpeed(void) {
  if(debugPrint) Serial.println("Get speed% from Nextion");

  uint32_t temp = 0;
  if(!nexSpeedPct.getValue(&temp)) {
    Serial.println("Error reading speed.");
    return 0;
  }
  else{
    if(debugPrint) Serial.printf("Speed = %d%%\n", temp);
    return temp;
  }
}

uint32_t getPrimeCycles(void) {
  if(debugPrint) Serial.println("Get prime cycles from Nextion");

  uint32_t temp = 0;
  if(!nexPrimeCycles.getValue(&temp)) {
    Serial.println("Error reading prime cycles.");
    return 0;
  }
  else{
    if(debugPrint) Serial.printf("Prime cycles = %d\n", temp);
    return temp;
  }
}

void nexDisableScreen(){
  sendCommand("tsw 255,0");
  recvRetCommandFinished();
}

void nexEnableScreen(){
  sendCommand("tsw 255,1");
  recvRetCommandFinished();
}
