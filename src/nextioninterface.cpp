#include "Nextion.h"
#include "nextioninterface.h"
#include "txtmessages.h"
#include"coding.h"

#include <errno.h>

NexPage page0(0, 0, "page0");       // Startup / home Screen
NexPage page1(1, 0, "page1");       // Dispense screen
NexPage page2(2, 0, "page2");       // Titrate creen
NexPage page3(3, 0, "page3");       // Settings screen
NexPage page4(4, 0, "page4");       // Titrate creen
NexPage tripPage(5, 0, "page5");    // Stall / Tripp and reset Screen

// Page 0 (Home)
NexText statusTextP0(0, 1, "t0");     // Status Text Ready, Running, Filling. Error
NexText errMsgP0(0, 2, "t1");        // Error Display 50 Characters max, 5 lines
NexText volumeTextP0(0, 3, "t2");     // Current Syringe Volume
NexProgressBar progressBarP0(0, 4, "j0"); //Syringe Slider 0=100% Full
NexDSButton valvePositionP0(0, 5, "bt0"); // Actual Valve Position 0=IN 1=Out
NexButton zeroButton(0, 6, "b0");   // Start zeroing of plunger
NexButton settingsButtonP0(0, 7, "b1"); //Page1 not used in mcu
NexButton primeButton(0, 8, "b2");  //Prime Syringe
NexButton titrateButton(0, 9, "b3");  // Empty Syringe
NexButton dispenseButton(0, 10, "b4");  // Empty Syringe

// Page 1 Dispense
NexText statusTextP1(1, 1, "t0");     // Status Text Ready, Running, Filling. Error
NexText errMsgP1(1, 2, "t1");         // Error Display 28 Carracters max
NexText volumeTextP1(1, 3, "t2");     // Current Syringe Volume
NexProgressBar progressBarP1(1, 4, "j0"); //Syringe Slider 0=100% Full
NexDSButton valvePositionP1(1, 5, "bt0"); // Actual Valve Position 0=IN 1=Out
NexButton homeButtonP1(1, 6, "b0");
NexButton emptyButtonP1(1, 7, "b1");

// Page 2 Titrate
NexText statusTextP2(2, 1, "t0");     // Status Text Ready, Running, Filling. Error
NexText errMsgP2(2, 2, "t1");         // Error Display 28 Carracters max
NexText volumeTextP2(2, 3, "t2");     // Current Syringe Volume
NexProgressBar progressBarP2(2, 4, "j0"); //Syringe Slider 0=100% Full
NexDSButton valvePositionP2(2, 5, "bt0"); // Actual Valve Position 0=IN 1=Out
NexButton homeButtonP2(2, 6, "b0");
NexButton zeroTotalButton(2, 7, "b2");    // Zero total volume titrated
NexDSButton rateSwitch(2, 8, "bt1");      // High/low speed for titration

// Page 3 Settings
NexText statusTextP3(3, 1, "t0");    // Status Text Ready, Running, Filling. Error
NexText errMsgP3(3, 2, "t1");        // Error Display 28 Carracters max
NexButton homeButtonP3(3, 3, "b0");  // Home Page button update the values for the syringe by reading the vairious values
NexButton manualButtonP3(3, 4, "b1");
NexText dispenseVolumeText(3, 5, "t2"); //Set Volume
NexNumber primeVolumeNumber(3, 6, "n0"); //global vairable to limit stkokeNumber must be updated on startup
NexNumber primeCyclesNumber(3, 7, "n1"); //Number of cycles to prime
NexNumber highSpeedNumber(3, 8, "n2");   // % of max speed
NexNumber lowSpeedNumber(3, 9, "n3"); // % of max speed
NexNumber nexMaxVolLimit(3, 10, "va0"); //global variable to limit stkokeNumber must be updated on startup

// Page 4 Manual
NexText statusTextP4(4, 1, "t0");     // Status Text Ready, Running, Filling. Error
NexText errMsgP4(4, 2, "t1");        // Error Display 28 Carracters max
NexText volumeTextP4(4, 3, "t2");     // Current Syringe Volume
NexProgressBar progressBarP4(4, 4, "j0"); //Syringe Slider 0=100% Full
NexDSButton valvePositionP4(4, 5, "bt0"); // Actual Valve Position 0=IN 1=Out
NexButton homeButtonP4(4, 6, "b0");
NexButton settingsButtonP4(4, 7, "b0");
NexButton downButton(4, 8, "b2"); // Move Syringe Down
NexButton upButton(4, 9, "b3"); // Move syringe UP
NexButton refillButton(4, 10, "b4");
NexDSButton switchValveButton(4, 11, "bt1"); // Switch Valve Position 0=In 1=Out

//Page 5 TRIP
NexButton resetSystemButton(5, 1, "b0"); //Page1 not used in mcu

// Buttons that have a executable action assigned to them
NexTouch *nex_listen_list[] = {
  // Page 0
  &zeroButton,
  &settingsButtonP0,
  &primeButton,
  &titrateButton,
  &dispenseButton,

  // Page 1
  &homeButtonP1,
  &emptyButtonP1,

  // Page 2
  &homeButtonP2,
  &rateSwitch,
  &zeroTotalButton,

  // Page 3
  &homeButtonP3,
  &manualButtonP3,

  // Page 4
  &homeButtonP4,
  &settingsButtonP4,
  &downButton,
  &upButton,
  &refillButton,
  &switchValveButton,

  // Page 5
  &resetSystemButton,
  NULL
};

//buffer to read values from Nextion
char buffer[30] = {0};

bool getDispenseVolume(float *val);
uint32_t getPrimeVolume(void);
uint32_t getHighSpeed(void);
uint32_t getLowSpeed(void);
uint32_t getPrimeCycles(void);

void processNexMessages(){
  nexLoop(nex_listen_list);
}

void zeroButtonReleased(void *ptr){
  if (debugPrint) Serial.println("zeroButtonPressed");
  errMsgP0.setText("Calibrating zero offset of syringe");
  statusTextP0.setText("Zeroing");
  if(doZero()){
    errMsgP0.setText("Prime syringe before selecting\n titrate or dispense");
    statusTextP0.setText("Zero OK");
  }
  // If doZero fails it will set states and display trip alert
}

void settingsButtonReleased(void *ptr) {
  if (debugPrint) Serial.println("settingsButtonReleased");
  settingMode();
}

void primeButtonReleased(void *ptr){
  if (debugPrint) Serial.println("primeButtonReleased");
  prime();
}

void titrateButtonReleased(void *ptr){
  if (debugPrint) Serial.println("titrateButtonReleased");
  if(!titrateMode()){
    sendCommand("page 0");
    recvRetCommandFinished(100);
    errMsgP0.setText("Prime first");
  }
  else{
    progressBarP2.setValue((uint32_t)currentPlungerPosition);
    dtostrf(currentSyringeVolume, 5, 3, buffer);
    volumeTextP2.setText(buffer);
  }
}

void dispenseButtonReleased(void *ptr){
  if (debugPrint) Serial.println("dispenseButtonReleased");
  if(!dispenseMode()){
    sendCommand("page 0");
    recvRetCommandFinished(100);
    errMsgP0.setText("Prime first");
  }
  else{
    progressBarP1.setValue((uint32_t)currentPlungerPosition);
    dtostrf(currentSyringeVolume, 5, 3, buffer);
    volumeTextP1.setText(buffer);
  }
}

void homeButtonReleased(void *ptr){
  if (debugPrint) Serial.println("homeButtonReleased");
  homeMode();
  progressBarP0.setValue((uint32_t)currentPlungerPosition);
  dtostrf(currentSyringeVolume, 5, 3, buffer);
  volumeTextP0.setText(buffer);
}

// empty the syringe by moving plunger up to zero volume
void emptyButtonReleased(void *prt){
  if (debugPrint) Serial.println("emptyButtonReleased");
  emptySyringe();
}

void rateSwitchButtonReleased(void *prt){
  if (debugPrint) Serial.println("rateSwitchButtonReleased");
  //
}

void zeroTotalButtonReleased(void *prt){
  if (debugPrint) Serial.println("zeroTotalButtonReleased");
  doZeroTitrateTotal();
}

void upButtonPushed(void *prt){
  if (debugPrint) Serial.println("upButtonPushed");
  moveUp();
}

void downButtonPushed(void *prt){
  if (debugPrint) Serial.println("downButtonPushed");
  moveDown();
}

void up_downButtonReleased(void *prt){
  /*if (debugPrint)*/ Serial.println("up_downButtonReleased");
  stopMove();
}

void refillButtonReleased(void *prt){
  if (debugPrint) Serial.println("refillButtonReleased");
  fillSyringe();
}

void resetSystemButtonReleased(void *ptr_) {
  resetAll();
}

//actuate the three way valve invert the current position
void switchValveButtonReleased(void *prt){
  uint32_t dual_state;
  switchValveButton.getValue(&dual_state);
  if (!dual_state) {
    switchValve(0);
  }
  else {
    switchValve(1);
  }
}

// Check settings, if anything is invalid, return to setings page
// and display some error messge
// If OK, proceed to next page
bool checkSettings(void){
  delay(100); // hack to try and read text, perhaps nextion is slow in processing requests?

  float tmpDispenseVol;
  if(getDispenseVolume(&tmpDispenseVol) == false){
    sendCommand("page 3");
    recvRetCommandFinished(100);
    errMsgP3.setText("Error converting");
    return false;
  }
  else{
    if(tmpDispenseVol > 3*syringeInfo[syringe].vol) {
      Serial.println("Set volume exceeded");
      sendCommand("page 3");
      recvRetCommandFinished(100);
      errMsgP3.setText("Set volume exceeded");
      char correctedVol[6];
      itoa(3*syringeInfo[syringe].vol, &correctedVol[0], 10);
      dispenseVolumeText.setText(correctedVol);
      return false;
    }
  }
  uint32_t tmpPrimeVol = getPrimeVolume();
  uint32_t tmpPrimeCycles = getPrimeCycles();
  uint32_t tmpHighSpeed = getHighSpeed();
  uint32_t tmpLowSpeed = getLowSpeed();

  settingsDone(tmpDispenseVol, tmpPrimeVol, tmpPrimeCycles, tmpHighSpeed, tmpLowSpeed);
  return true;
}

void manualButtonReleased(void *ptr) {
  if (debugPrint) Serial.println("manualButtonReleased");
  manualMode();
  progressBarP4.setValue((uint32_t)currentPlungerPosition);
  dtostrf(currentSyringeVolume, 5, 3, buffer);
  volumeTextP4.setText(buffer);
}

// First read settings
// if no error, proceed to Home
void homeButtonP3Released(void *prt_){
  if (debugPrint) Serial.println("homeButtonP3Released");
  if(checkSettings()){
    homeMode();
    progressBarP0.setValue((uint32_t)currentPlungerPosition);
    dtostrf(currentSyringeVolume, 5, 3, buffer);
    volumeTextP0.setText(buffer);
  }
}

// Interface to display elements
void updateStatusTxt(const char status[]){
  statusTextP0.setText(status);
}

void updateErrorTxt(const char err[]){
  errMsgP0.setText(err);
  errMsgP1.setText(err);
  errMsgP2.setText(err);
  errMsgP3.setText(err);
  errMsgP4.setText(err);
}

void updateErrorTxt0(const char txt[]){
  errMsgP0.setText(txt);
}

void updateErrorTxt1(const char txt[]){
  errMsgP1.setText(txt);
}

void updateErrorTxt2(const char txt[]){
  errMsgP2.setText(txt);
}

void updateErrorTxt4(const char txt[]){
  errMsgP4.setText(txt);
}

void updateVolumeTxt(const char txt[]){
  volumeTextP0.setText(txt);
  volumeTextP1.setText(txt);
  volumeTextP2.setText(txt);
  volumeTextP4.setText(txt);
}

void updateVolumeTxt0(const char txt[]){
  volumeTextP0.setText(txt);
}

void updateVolumeTxt1(const char txt[]){
  volumeTextP1.setText(txt);
}

void updateVolumeTxt2(const char txt[]){
  volumeTextP2.setText(txt);
}

void updateVolumeTxt4(const char txt[]){
  volumeTextP4.setText(txt);
}

void updateVolumeTxt2NoAck(float vol){
  const char sendCmdTxt[] = "page2.t1.txt=\"%.3f\"\xff\xff\xff";
  sprintf(buffer, sendCmdTxt, vol);
  nexSerial.write(buffer);
}

void updateVolumeTxt4NoAck(float vol){
  const char sendCmdTxt[] = "page4.t1.txt=\"%.3f\"\xff\xff\xff";
  sprintf(buffer, sendCmdTxt, vol);
  nexSerial.write(buffer);
}

void updateValveDisplay(uint8_t pos){
  valvePositionP0.setValue(pos);
  valvePositionP1.setValue(pos);
  valvePositionP2.setValue(pos);
  valvePositionP4.setValue(pos);
  switchValveButton.setValue(pos);  //update valve switch p1
}

void updateProgressbarHome(uint32_t pos){
  progressBarP0.setValue(pos);
}

void updateProgressbarDispense(uint32_t pos){
  progressBarP1.setValue(pos);
}

void updateProgressbarTitrate(uint32_t pos){
  progressBarP2.setValue(pos);
}
// Send command but do not read response
// This leaves Nextion response in serial buffer
// This response will then be removed by nexLoop without interfering with the nexLoop logic.
void updateProgressbarTitrateNoAck(uint32_t pos){
  const char sendCmdVal[] = "page2.j0.val=%d\xff\xff\xff";
  sprintf(buffer, sendCmdVal, pos);
  nexSerial.write(buffer);
}

void updateProgressbarManual(uint32_t pos){
  progressBarP4.setValue(pos);
}

void updateProgressbarManualNoAck(uint32_t pos){
  const char sendCmdVal[] = "page4.j0.val=%d\xff\xff\xff";
  sprintf(buffer, sendCmdVal, pos);
  nexSerial.write(buffer);
}

void updateSettingsDisplay(float tmpDispenseVol, uint32_t tmpPrimeVol, uint32_t tmpPrimeCycles,
                    uint32_t tmpHighSpeed, uint32_t tmpLowSpeed){
  sprintf(buffer, "%.3f", tmpDispenseVol);
  dispenseVolumeText.setText(buffer);
  primeVolumeNumber.setValue(tmpPrimeVol);
  primeCyclesNumber.setValue(tmpPrimeCycles);
  highSpeedNumber.setValue(tmpHighSpeed);
  lowSpeedNumber.setValue(tmpLowSpeed);
}

void setNexMaxVolLimit(uint32_t limit){
  nexMaxVolLimit.setValue(limit);
}

void initNextionInterface(){
  //register the pop events
  zeroButton.attachPop(zeroButtonReleased);
  settingsButtonP0.attachPop(settingsButtonReleased);
  primeButton.attachPop(primeButtonReleased);
  titrateButton.attachPop(titrateButtonReleased);
  dispenseButton.attachPop(dispenseButtonReleased);

  // Page 1
  homeButtonP1.attachPop(homeButtonReleased);
  emptyButtonP1.attachPop(emptyButtonReleased);

  // Page 2
  homeButtonP2.attachPop(homeButtonReleased);
  rateSwitch.attachPop(rateSwitchButtonReleased);
  zeroTotalButton.attachPop(zeroTotalButtonReleased);

  // Page 3
  homeButtonP3.attachPop(homeButtonP3Released);
  manualButtonP3.attachPop(manualButtonReleased);

  // Page 4
  homeButtonP4.attachPop(homeButtonReleased);
  settingsButtonP4.attachPop(settingsButtonReleased);
  downButton.attachPush(downButtonPushed);
  downButton.attachPop(up_downButtonReleased);
  upButton.attachPush(upButtonPushed);
  upButton.attachPop(up_downButtonReleased);
  refillButton.attachPop(refillButtonReleased),
  switchValveButton.attachPop(switchValveButtonReleased);

  // Page 5
  resetSystemButton.attachPop(resetSystemButtonReleased);
}

void nexTripAlert(){
  Serial.println(tripMessage);
  sendCommand("page 5"); // page3.show() doesn't work??
}

void nexReset(){
  sendCommand("rest");
}

// Errors result in return to settings page
bool getDispenseVolume(float *val) {
  if(debugPrint) Serial.println("Get dispense volume from Nextion");
  memset(buffer, 0, sizeof(buffer));
  if(!dispenseVolumeText.getText(buffer, sizeof(buffer))){
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
  if(!primeVolumeNumber.getValue(&temp)) {
    Serial.println("Error reading priming volume.");
    return 0;
  }
  else{
    if(debugPrint) Serial.printf("Priming volume = %d\n", temp);
    return temp;
  }
}

uint32_t getHighSpeed(void) {
  if(debugPrint) Serial.println("Get high speed% from Nextion");

  uint32_t temp = 0;
  if(!highSpeedNumber.getValue(&temp)) {
    Serial.println("Error reading high speed.");
    return 0;
  }
  else{
    if(debugPrint) Serial.printf("High speed = %d%%\n", temp);
    return temp;
  }
}

uint32_t getLowSpeed(void) {
  if(debugPrint) Serial.println("Get low speed% from Nextion");

  uint32_t temp = 0;
  if(!lowSpeedNumber.getValue(&temp)) {
    Serial.println("Error reading low speed.");
    return 0;
  }
  else{
    if(debugPrint) Serial.printf("Low speed = %d%%\n", temp);
    return temp;
  }
}

uint32_t getPrimeCycles(void) {
  if(debugPrint) Serial.println("Get prime cycles from Nextion");

  uint32_t temp = 0;
  if(!primeCyclesNumber.getValue(&temp)) {
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
