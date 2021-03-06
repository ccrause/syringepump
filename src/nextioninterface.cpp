#include "Nextion.h"
#include "nextioninterface.h"
#include "txtmessages.h"
#include"coding.h"

#include <errno.h>

NexPage homePage(0, 0, "page0");
NexPage dispensePage(1, 0, "page1");
NexPage titratePage(2, 0, "page2");
NexPage settingsPage(3, 0, "page3");
NexPage manualPage(4, 0, "page4");
NexPage tripPage(5, 0, "page5");

// Page 0 (Home)
NexText statusTextP0(0, 1, "t0");     // Status Text Ready, Running, Filling. Error
NexText errMsgP0(0, 2, "t1");        // Error Display 50 Characters max, 5 lines
NexText volumeTextP0(0, 3, "t2");     // Current Syringe Volume
NexProgressBar progressBarP0(0, 4, "j0"); //Syringe Slider 0=100% Full
NexDSButton valvePositionP0(0, 5, "bt0"); // Actual Valve Position 0=IN 1=Out
NexDSButton zeroButton(0, 6, "bt1");   // Start zeroing of plunger
NexDSButton settingsButtonP0(0, 7, "bt2"); //Page1 not used in mcu
NexDSButton primeButton(0, 8, "bt3");  //Prime Syringe
NexDSButton titrateButton(0, 9, "bt4");  // Empty Syringe
NexDSButton dispenseScreenButton(0, 10, "bt5");  // Empty Syringe

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
NexText dispenseVolumeText(3, 4, "t2"); //Set Volume
NexNumber primeVolumeNumber(3, 5, "n0"); //global vairable to limit stkokeNumber must be updated on startup
NexNumber primeCyclesNumber(3, 6, "n1"); //Number of cycles to prime
NexNumber highSpeedNumber(3, 7, "n2");   // % of max speed
NexNumber lowSpeedNumber(3, 8, "n3"); // % of max speed
NexNumber nexMaxVolLimit(3, 9, "va0"); //global variable to limit stkokeNumber must be updated on startup
NexDSButton manualButtonP3(3, 11, "bt1"); //Go to manual page ds button

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
  &dispenseScreenButton,

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
  zeroButton.setValue(0);
  // If doZero fails it will set states and display trip alert
}

void settingsButtonReleased(void *ptr) {
  if (debugPrint) Serial.println("settingsButtonReleased");
  settingMode();
  settingsButtonP0.setValue(0);
  settingsPage.show();
}

void primeButtonReleased(void *ptr){
  if (debugPrint) Serial.println("primeButtonReleased");
  prime();
  primeButton.setValue(0);
}

void titrateButtonReleased(void *ptr){
  if (debugPrint) Serial.println("titrateButtonReleased");

  if(!titrateMode()){
    homePage.show();
    recvRetCommandFinished(100);
    errMsgP0.setText("Prime first");
  }
  else{
    titratePage.show();
    rateSwitch.setValue(0);
    setTitrateRate(false);
    progressBarP2.setValue((uint32_t)currentPlungerPosition);
    dtostrf(currentSyringeVolume, 5, 3, buffer);
    volumeTextP2.setText(buffer);
  }
  titrateButton.setValue(0);
}

void dispenseButtonReleased(void *ptr){
  if (debugPrint) Serial.println("dispenseButtonReleased");
  if(!dispenseMode()){
    homePage.show();
    recvRetCommandFinished(100);
    errMsgP0.setText("Prime first");
  }
  else{
    dispensePage.show();
    progressBarP1.setValue((uint32_t)currentPlungerPosition);
    dtostrf(currentSyringeVolume, 5, 3, buffer);
    volumeTextP1.setText(buffer);
  }
  dispenseScreenButton.setValue(0);
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
  uint32_t val;
  rateSwitch.getValue(&val);
  setTitrateRate((bool)val);
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
  if (debugPrint) Serial.println("up_downButtonReleased");
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
    settingsPage.show();
    recvRetCommandFinished(100);
    errMsgP3.setText("Error converting");
    return false;
  }
  else{
    if(tmpDispenseVol > 3*syringeInfo[syringe].vol) {
      Serial.println("Set volume exceeded");
      settingsPage.show();
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
  manualPage.show();
  manualButtonP3.setValue(0);
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
  statusTextP1.setText(status);
  statusTextP2.setText(status);
  statusTextP3.setText(status);
  statusTextP4.setText(status);
}

void updateStatusTxt0(const char status[]){
  statusTextP0.setText(status);
}

void updateStatusTxt1(const char status[]){
  statusTextP1.setText(status);
}

void updateStatusTxt2(const char status[]){
  statusTextP2.setText(status);
}

void updateStatusTxt3(const char status[]){
  statusTextP3.setText(status);
}

void updateStatusTxt4(const char status[]){
  statusTextP4.setText(status);
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

void updateProgressbarManual(uint32_t pos){
  progressBarP4.setValue(pos);
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
  dispenseScreenButton.attachPop(dispenseButtonReleased);

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
  tripPage.show();
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
//  recvRetCommandFinished();
}

void nexEnableScreen(){
  sendCommand("tsw 255,1");
//  recvRetCommandFinished();
}

void nexDisablePrime(){
  primeButton.disable();
  primeButton.setValue(1);
}

void nexDisableTitrate(){
  titrateButton.disable(); // show as pressed in
  titrateButton.setValue(1);
}

void nexDisableDispense(){
  dispenseScreenButton.disable();
  dispenseScreenButton.setValue(1);
}

void nexEnablePrime(){
  primeButton.enable();
  primeButton.setValue(0);
}

void nexEnableTitrate(){
  titrateButton.enable();
  titrateButton.setValue(0);
}

void nexEnableDispense(){
  dispenseScreenButton.enable();
  dispenseScreenButton.setValue(0);
}
