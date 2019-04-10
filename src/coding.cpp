#ifdef __AVR__
  #include <avr/io.h>
#endif
#include "Arduino.h"
#include "Nextion.h"
#include "SPI.h"
#include "SD.h"
#include "AccelStepper.h"
#include "pumpdriver.h"
#include "Servo.h"  // Uses ServoESP32 library.  In platformIO: http://platformio.org/lib/show/1739/ServoESP32
#include "pinconfig.h"
#include "AccelStepper.h"
#include <Preferences.h>

AccelStepper motor(1, stepPin, dirPin);

#define maxSpeed 40 // mm/s
#define maxAcceleration maxSpeed / 2  // mm/s/s
#define defaultStroke 90
#define defaultSpeedPct 50
#define defaultVolume 5.0
#define defaultDiameter 20
#define defaultPrimeCycles 2

#define stPmm 800 //800; //steps per mm
#define pi 3.14159265
#define configNamespace "pumpSettings"
#define configNameStroke "stroke"
#define configNameSpeed "speed"
#define configNameVolume "volume"
#define configNameDiameter "diameter"
#define configNamePrimeCycles "prime"

// Common status strings
const char msgFilling[] = "Filling";
const char msgPriming[] = "Priming";
const char msgError[] = "Error";
const char msgZeroing[] = "Zeroing";
const char msgReady[] = "Ready";
const char msgNotReady[] = "Not Ready";
const char msgDispensing[] = "Dispensing";
const char msgEmptying[] = "Emptying";

enum OpState {osUnInitialized=0,  // startup state
              osZeroed=1,         // zeroing completed
              osEmpty=2,          // empty action completed
              osPrimed=4,         // prime cycle completed
              osSettings=8,       // in settings screen
              osBusy=16,          // busy with action - not required at the moment since all actions are blocking
              osManual=32,        // in manual screen
              osTripped=128};     // motor is tripped
OpState currentState = osUnInitialized;
#define includeState(state) currentState = (OpState)(currentState | state)
#define excludeState(state) currentState = (OpState)(currentState & ~(state))
#define containState(state) ((int)(currentState & state) == (int)state)

//--------------------------------------------Variables-----------------------------------------------
float stroke;   // max travel distance mm
uint32_t speedPct; // % maximum speed
float dispenseVol, dispenseCycleVol, totalDispensedVol; // requested volume mL
byte dispenseCycles;  // number of dispense cycles to dispense total volume
long dispenseStroke = 0;  // stroke per dispense cycle
float diameter;    // syringe diameter mm
int32_t speed;    // Actual speed in mm/sec
uint32_t primeCycles; //number of times the syringe cycles
float syringeVol = 0;

long strokePosLimit = defaultStroke * stPmm; // max stroke position in steps (default 100 mm)
bool displayDispenseVolume = false;

bool debugPrint = true;

// -------------------------------------------Servo----------------------------------------------------
Servo valve;
const int in = 0; // servo angle for inlet aliagnment
const int out = 140; // servo angle for outlet aliagnment
enum valvePosition {vpInlet=0, vpOutlet};

//-------------------------------------------------NEXTION---------------------------------------------
NexPage page0 = NexPage(0, 0, "page0"); // Home Screen
NexPage page1 = NexPage(1, 0, "page1"); // Settings Screen
NexPage page2 = NexPage(2, 0, "page2"); // Manual Control Screen
NexPage page3 = NexPage(3, 0, "page3"); // Stall / Tripp and reset Screen

// Page 0 (Home)
NexButton primeButton = NexButton(0, 1, "b0"); //Prime Syringe
NexButton emptyButton = NexButton(0, 2, "b1"); // Empty Syringe
NexButton settingsButtonP0 = NexButton(0, 3, "b2"); //Page1 not used in mcu
NexButton manualButtonP0 = NexButton(0, 9, "b3"); //Page1 not used in mcu
NexDSButton valvePosition0 = NexDSButton(0, 4, "bt0"); // Actual Valve Position 0=IN 1=Out
NexProgressBar progressBar0 = NexProgressBar(0, 5, "j0"); //Syringe Slider 0=100% Full
NexText statusText = NexText(0, 6, "t0"); //Status Text Ready, Running, Filling. Error
NexText volumeText = NexText(0, 7, "t1"); //Current Syringe Volume
NexText errMsg0 = NexText(0, 8, "t2"); //Error Display 28 Carracters max

//Page 1 (Settings)
NexButton homeButtonP1 = NexButton(1, 1, "b0"); // Home Page button update the values for the syringe by reading the vairious values
NexText volumeSettingText = NexText(1, 2, "t0"); //Set Volume
NexText diameterText = NexText(1, 3, "t1"); //Syringe Diameter mm Float
NexText errMsg1 = NexText(1, 4, "t4"); //Error Display 28 Carracters max
NexNumber speedNumber = NexNumber(1, 5, "n0"); // % of max speed
NexNumber strokeNumber = NexNumber(1, 6, "n1"); //Stroke Length mm int
NexNumber primeCyclesNumber = NexNumber(1, 8, "n2"); //Number of cycles to prime
NexButton manualButtonP1 = NexButton(1, 9, "b1");

//Page 2 (manual control)
NexDSButton valvePosition1 = NexDSButton(2, 1, "bt0"); // Actual Valve Position 0=In 1=Out
NexDSButton switchValveButton = NexDSButton(2, 2, "bt1"); // Switch Valve Position 0=In 1=Out
NexButton upButton = NexButton(2, 3, "b0"); // Move syringe UP
NexButton downButton = NexButton(2, 4, "b1"); // Move Syringe Down
NexButton homeButtonP2 = NexButton(2, 5, "b2"); // Home Page button
NexButton settingsButtonP2 = NexButton(2, 6, "b3"); //Page1 not used in mcu
NexProgressBar progressBar1 = NexProgressBar(2, 7, "j0"); //Syringe Slider 0=100% Full
NexText errMsg2 = NexText(2, 8, "t0"); // Errror Diplay 15 caracters max
NexText volumeTextP2 = NexText(2, 9, "t1"); //Current Syringe Volume

//Page 3 (TRIP Screen
NexButton resetSystemButton = NexButton(3, 1, "b0"); //Page1 not used in mcu

//buffer to read values from Nextion
char buffer[30] = {0};

// Buttons that have a executable action assigned to them
NexTouch *nex_listen_list[] = {
  &primeButton, // Prime
  &homeButtonP1,// Update Diameter, Stroke and Speed
  &homeButtonP2,
  &switchValveButton, // Change Valve position
  &emptyButton, //Empty Syringe
  &upButton, //Move up 5mm
  &downButton, //Move down 5mm
  &settingsButtonP0,
  &settingsButtonP2,
  &manualButtonP0,
  &manualButtonP1,
  &resetSystemButton,
  NULL
};

// Read/write config parameters
Preferences configStorage;

void updateDosingParams();

void nexTripAlert(){
  const char * msg2 = "TRIP: please service & restart";
  Serial.println(msg2);
  sendCommand("page 3"); // page3.show() doesn't work??
}

void resetAll() {
  Serial.println("Software reset...");
  sendCommand("rest");
  delay(10);  // make sure command is transmitter over serial
  ESP.restart();
}

void safeMoveTo(long newpos){
  // limit max travel to maximum allowed syringe stroke length
  if(newpos > strokePosLimit){
    newpos = strokePosLimit;
  }

  // Reset trip counter to prevent spurious trip when reversing direction
  stepper_count = 0;

  if(debugPrint){
    Serial.println("Before moveTo:");
    Serial.printf("curPos: %ld\ntargetPos: %ld\nspeed: %0.2f\n",
                  motor.currentPosition(), motor.targetPosition(), motor.speed());
  }
  motor.moveTo(newpos);
  if(debugPrint){
    Serial.println("After moveTo:");
    Serial.printf("curPos: %ld\ntargetPos: %ld\nspeed: %0.2f\n",
                  motor.currentPosition(), motor.targetPosition(), motor.speed());
  }
  moveToPosition = true;
  motorIsRunning = true;
  float deltaVol = 0;

  // Wait until move is finished
  while(motorIsRunning && (trip == false)){
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if(debugPrint) Serial.printf("curPos: %ld\ntargetPos: %ld\nspeed: %0.2f\n",
                  motor.currentPosition(), motor.targetPosition(), motor.speed());
    // Update progress bar with plunger movement
    if(containState(osZeroed) && (nexSerial.availableForWrite() > 20)) {
      float progress = (100.0f * (float)(strokePosLimit - motor.currentPosition())) / strokePosLimit;
      if(progress > 100){
        progress = 100;
      }
      else if(progress < 0){
        progress = 0;
      }
      progressBar0.setValue((uint32_t)progress);
      float Vol = syringeVol * (100.0f - progress) / 100.0f;
      dtostrf(Vol, 5, 3, buffer);
      volumeText.setText(buffer);

      // update Volume display
      if(displayDispenseVolume){
        progress = (100.0f * (float)(strokePosLimit - motor.currentPosition())) / dispenseStroke;
        deltaVol = dispenseCycleVol * progress / 100.0f;
        dtostrf(totalDispensedVol + deltaVol, 5, 3, buffer);
        errMsg0.setText(buffer);
      }
    }
  }
  totalDispensedVol += deltaVol;

  // Error diagnostics
  if(trip == true){
    // if not yet zeroed, set zero point 1 mm back
    if(debugPrint) Serial.println("Trip detected");
    if(containState(osZeroed)){
      includeState(osTripped);
      excludeState(osBusy);
      nexTripAlert();
    }
  }
  else if(motor.distanceToGo() != 0){
    if(debugPrint){
      Serial.println("safeMove didn't complete distance.");
      Serial.printf("Current position: %ld\n", motor.currentPosition());
    }
  }
}

void safeRun(long newSpeed){
  if(debugPrint) Serial.printf("New speed: %ld\n", newSpeed);

  // Reset trip counter to prevent spurious trip when reversing direction
  stepper_count = 0;
  motor.setSpeed(newSpeed);
  moveToPosition = false;
  motorIsRunning = true;

  // Wait until move is stopped from pop event
  while(motorIsRunning && (trip == false)){
    nexLoop(nex_listen_list);  // need to listed for pop event to stop - BREAK THE BLOCKING FLOW - BEWARE!!!!
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // Update progress bar with plunger movement
    if(containState(osZeroed) && (nexSerial.availableForWrite() > 20)) {
      double progress = (100.0 * (float)(strokePosLimit - motor.currentPosition())) / strokePosLimit;
      if(progress > 100){
        progress = 100;
      }
      else if(progress < 0){
        progress = 0;
      }

      // Send command but do not read response
      // This leaves Nextion response in serial buffer
      // This response will then be removed by nexLoop without interfering with the nexLoop logic.
      char buf[32] = {0};
      const char sendCmdVal[] = "page2.j0.val=%d\xff\xff\xff";
      sprintf(buf, sendCmdVal, (int)progress);
      nexSerial.write(buf);

      float Vol = syringeVol * (100.0f - progress) / 100.0f;
      const char sendCmdTxt[] = "page2.t1.txt=\"%.3f\"\xff\xff\xff";
      sprintf(buf, sendCmdTxt, Vol);
      nexSerial.write(buf);
    }
  }
  if (debugPrint) Serial.println("Stopped running");

  // Need to reset AccelStepper state so that using .run() doesn't give problems
  motor.setCurrentPosition(motor.currentPosition());

  // Error diagnostics
  if(trip == true){
    includeState(osTripped);
    excludeState(osBusy);
    nexTripAlert();
  }
}

// prohibitedStates can combine different states e.g. osTripped | osBusy and will fail on any one
// requiredStates can combine different states e.g. osZeroed | osPrimed and will fail if any one is not set
// boolean passPreconditions(OpState prohibitedStates, OpState requiredStates, char* context){
boolean passPreconditions(uint8_t prohibitedStates, uint8_t requiredStates, const char* context){
  boolean result = (currentState & requiredStates) == requiredStates;
  result &= (boolean)((currentState & prohibitedStates) == 0);

  if(!result){
    Serial.printf("Precondition check failed in %s\n", context);
    if(debugPrint) Serial.printf("currentState=%#.2x prohibitedstates=%#.2x requiredstates=%#.2x\n", currentState, prohibitedStates, requiredStates);
  }
  return result;
}

void switchValve(valvePosition pos) {
  if(pos == vpInlet){
    valve.write(in);  //actuate Servo to inlet side
    valvePosition0.setValue(0);  //update valve p0
    valvePosition1.setValue(0); //update valve p1
    switchValveButton.setValue(0);  //update valve switch p1
  }
  else if (pos == vpOutlet) {
    valve.write(out);  //actuate Servo to inlet side
    valvePosition0.setValue(1);  //update valve p0
    valvePosition1.setValue(1); //update valve p1
    switchValveButton.setValue(1);  //update valve switch p1
  }
  else if(debugPrint) Serial.printf("Invalid parameter passed to switchValve: %d", pos);
  delay(250);
}

//Filling the syringe to remove air
void primeButtonPopCallBack(void *ptr){
  if(!passPreconditions(osTripped | osSettings | osManual, osZeroed, "prime")){
    return;
  }
  includeState(osBusy);
  Serial.println(msgPriming);
  errMsg0.setText("Please Wait");
  statusText.setText(msgPriming); // nextion status
  totalDispensedVol = 0;
  for (byte i = 0; i < primeCycles; i++){
    switchValve(vpOutlet);
    safeMoveTo(0);
    if(containState(osTripped)) return;
    if(debugPrint) Serial.println("Syringe empty");

    switchValve(vpInlet);
    safeMoveTo(strokePosLimit);
    if(containState(osTripped)) return;
    if(debugPrint) Serial.println("Syringe Filled");
  }
  excludeState(osEmpty | osBusy);
  includeState(osPrimed);
  errMsg0.setText(" ");
  statusText.setText(msgReady);
  Serial.println(msgReady);
  // Empty Serial input buffer
  while(Serial.available()) Serial.read();
}

//actuate the three way valve invert the current position
void switchValveButtonPopCallBack(void *prt){
  if(!passPreconditions(osTripped | osBusy | osManual, 0, "swicth valve")) return;
  uint32_t dual_state;
  includeState(osBusy);
  switchValveButton.getValue(&dual_state);
  if (!dual_state) {
    switchValve(vpInlet);
    Serial.println("valve to inlet");
  }
  else {
    switchValve(vpOutlet);
    Serial.println("valve to outlet");
  }
  excludeState(osBusy);
}

// empty the syringe by moving plunger up to zero volume
void emptyButtonPopCallBack(void *prt){
  if(!passPreconditions(osTripped | osSettings | osManual | osBusy, 0, "empty")) {
    return;
  }
  includeState(osBusy);
  Serial.println(msgEmptying);
  statusText.setText(msgEmptying);
  switchValve(vpOutlet);
  if(debugPrint) Serial.println("valve to outlet");
  safeMoveTo(0);
  Serial.println(msgReady);
  statusText.setText(msgReady);
  includeState(osEmpty);
  excludeState(osBusy);
}

// move the plunger up 5mm if possible
void upButtonPushCallback(void *prt){
  if (debugPrint) Serial.println("Up button push callback");

  if(!passPreconditions(osTripped | osBusy, 0, "up")) {
    return;
  }
  includeState(osBusy);
  safeRun(-speed*stPmm / 8);
}

void downButtonPushCallback(void *prt){
  if (debugPrint) Serial.println("Down button push callback");
  if(!passPreconditions(osTripped | osBusy, 0, "up")) {
    return;
  }
  includeState(osBusy);
  safeRun(speed*stPmm / 8);
}

void up_downButtonPopCallback(void *prt){
  if (debugPrint) Serial.println("Up_down button pop callback");
  motorIsRunning = false;
  excludeState(osBusy);
}

void settingsButtonPopCallBack(void *ptr) {
  includeState(osSettings);
  excludeState(osManual);
}

void manualButtonPopCallBack(void *ptr) {
  includeState(osManual);
  excludeState(osSettings);
}

float getVolume(void) {
  if(debugPrint) Serial.println("Get volume from Nextion");
  memset(buffer, 0, sizeof(buffer));
  if(!volumeSettingText.getText(buffer, sizeof(buffer))){
    Serial.println("Error reading volume.");
    return 0;
  }
  else{
    float temp = atof(buffer);
    if(debugPrint) Serial.printf("Volume = %f\n", temp);
    return temp;
  }
}

float getDiameter(void) {
  if(debugPrint) Serial.println("Get diameter from Nextion");
  memset(buffer, 0, sizeof(buffer));
  if(!diameterText.getText(buffer, sizeof(buffer))){
    Serial.println("Error reading diameter.");
    return 0;
  }
  else{
    float temp = atof(buffer);
    if(debugPrint) Serial.printf("Diameter = %f\n", temp);
    return temp;
  }
}

uint32_t getStroke(void) {
  if(debugPrint) Serial.println("Get stroke from Nextion");

  uint32_t temp = 0;
  if(!strokeNumber.getValue(&temp)) {
    Serial.println("Error reading stroke.");
    return 0;
  }
  else{
    if(debugPrint) Serial.printf("Stroke = %d\n", temp);
    return temp;
  }
}

uint32_t getSpeed(void) {
  if(debugPrint) Serial.println("Get speed% from Nextion");

  uint32_t temp = 0;
  if(!speedNumber.getValue(&temp)) {
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
  if(!primeCyclesNumber.getValue(&temp)) {
    Serial.println("Error reading prime cycles.");
    return 0;
  }
  else{
    if(debugPrint) Serial.printf("Prime cycles = %d\n", temp);
    return temp;
  }
}

void updateDosingParams(){
  if(debugPrint) Serial.printf("stroke: %dmm\n", (int)stroke);
  strokePosLimit = stroke * stPmm;
  if(debugPrint) Serial.printf("strokePosLimit: %ld\n", strokePosLimit);

  float tmpProgress = (100.0f * (float)(strokePosLimit - motor.currentPosition())) / strokePosLimit;
  if(tmpProgress > 100) {tmpProgress = 100;}
  else if(tmpProgress < 0) {tmpProgress = 0;}
  progressBar0.setValue((uint32_t)tmpProgress);

  speed = (maxSpeed * speedPct) / 100.0f;
  if(debugPrint) Serial.printf("Speed = %d mm/s\n", speed);
  motor.setMaxSpeed(speed * stPmm);
  motor.setAcceleration(speed * stPmm / 2);

  float Ac = (diameter/10.0f);
  Ac = Ac*Ac/4.0f*pi;  // in cm
  if(debugPrint) Serial.printf("Ac: %.3f cm^2\n", Ac);
  syringeVol = (float)stroke/10.0f * Ac;
  if(debugPrint) Serial.printf("Syringe volume: %.3f\n", syringeVol);
  dispenseCycles = round((dispenseVol/syringeVol) + 0.5f);
  dispenseCycleVol = dispenseVol / dispenseCycles;
  if(debugPrint) Serial.printf("Dispense cycles: %d\n", dispenseCycles);

  float tmpVol = syringeVol * (100.0f - tmpProgress) / 100.0f;
  dtostrf(tmpVol, 5, 3, buffer);
  volumeText.setText(buffer);

  dispenseStroke = dispenseCycleVol / Ac * 10.0f * stPmm;  // cm3/cm2 * 10 => mm
  if (dispenseStroke > strokePosLimit){
    Serial.println("Dosing volume exeeds stroke limit");
    dispenseStroke = strokePosLimit;
  }
  if(debugPrint) Serial.printf("dispenseStroke = %ld\n", dispenseStroke);
}

void homeButtonPopCallBack(void *prt_){
  // update syringe diameter, stroke lenght and speed by reading the value's from nextion hmi
  delay(100); // hack to try and read stroke text, perhaps nextion is slow to copy & convert text from page1 to page0?
  excludeState(osSettings);
  excludeState(osManual);
  if(debugPrint) Serial.println("Reading settings from Nextion.");
  uint32_t tmpStroke = getStroke();
  uint32_t tmpSpeed = getSpeed();
  float tmpDiameter = getDiameter();
  float tmpVol = getVolume();
  uint32_t tmpPrimeCycles = getPrimeCycles();

  if(configStorage.begin(configNamespace, false)){  // RW mode
    if(debugPrint) Serial.println("Saving config");

    if(tmpStroke != stroke) {
      stroke = tmpStroke;
      if(debugPrint) Serial.println("Saving stroke");
      if(configStorage.putUShort(configNameStroke, stroke) == 0){
        Serial.println("Error saving stroke");
      }
    }

    if(tmpSpeed != speedPct) {
      speedPct = tmpSpeed;
      if(debugPrint) Serial.println("Saving speed");
      if(configStorage.putUShort(configNameSpeed, speedPct) == 0){
        Serial.println("Error saving speed");
      }
    }

    if(tmpVol != dispenseVol) {
      dispenseVol = tmpVol;
      if(debugPrint) Serial.println("Saving volume");
      if(configStorage.putFloat(configNameVolume, dispenseVol) == 0){
        Serial.println("Error saving volume");
      }
    }

    if(tmpDiameter != diameter) {
      diameter = tmpDiameter;
      if(debugPrint) Serial.println("Saving diameter");
      if(configStorage.putFloat(configNameDiameter, diameter) == 0){
        Serial.println("Error saving diameter");
      }
    }

    if(tmpPrimeCycles != primeCycles) {
      primeCycles = tmpPrimeCycles;
      if(debugPrint) Serial.println("Saving prime cycles");
      if(configStorage.putUShort(configNamePrimeCycles, primeCycles) == 0){
        Serial.println("Error saving prime cycles");
      }
    }
    configStorage.end();
  }
  else{
    Serial.println("Error opening config for saving");
  }
  updateDosingParams();
}

void resetSystemButtonPopCallback(void *ptr_) {
  resetAll();
}

void dispense(){
  if(!passPreconditions(osTripped | osSettings | osManual | osBusy, osPrimed, "dispense")){
    const char btnMsg[] = "Button active in Home screen only";
    if(containState(osSettings)){
      errMsg1.setText(btnMsg);
    }
    else if(containState(osManual)){
      errMsg2.setText(btnMsg);
    }
    return;
  }
  includeState(osBusy);
  sendCommand("tsw 255, 0");  // disable touch events on screen
  statusText.setText(msgDispensing);
  Serial.println(msgDispensing);
  // start dispense cycle from bottom position
  if (motor.currentPosition() != strokePosLimit) {
    switchValve(vpInlet);
    statusText.setText(msgFilling);
    safeMoveTo(strokePosLimit);
  }
  if(containState(osTripped)) return;  // do nothing if tripped

  totalDispensedVol = 0;
  for(byte i=0; i<dispenseCycles; i++){
    switchValve(vpOutlet);
    statusText.setText(msgDispensing);
    displayDispenseVolume = true;
    safeMoveTo(motor.currentPosition() - dispenseStroke);
    if(containState(osTripped)) return;  // do nothing if tripped

    displayDispenseVolume = false;
    switchValve(vpInlet);
    statusText.setText(msgFilling);
    safeMoveTo(strokePosLimit);
    if(containState(osTripped)) return;  // do nothing if tripped
  }
  excludeState(osBusy);
  statusText.setText(msgReady);
  Serial.println(msgReady);
  sendCommand("tsw 255, 1");  // enable touch events on screen
  // Empty Serial input buffer
  while(Serial.available()) Serial.read();
}

// Reads config data from non-volatile storage
void loadConfig(){
  if(!configStorage.begin(configNamespace, true)){  // Read-only mode
    Serial.println("Error opening config for loading");
  }
  stroke = configStorage.getUShort(configNameStroke, defaultStroke);
  speedPct = configStorage.getUShort(configNameSpeed, defaultSpeedPct);
  dispenseVol = configStorage.getFloat(configNameVolume, defaultVolume);
  diameter = configStorage.getFloat(configNameDiameter, defaultDiameter);
  primeCycles = configStorage.getUShort(configNamePrimeCycles, defaultPrimeCycles);
  configStorage.end();
  updateDosingParams();

  // Update display
  strokeNumber.setValue(stroke);
  speedNumber.setValue(speedPct);
  sprintf(buffer, "%.3f", dispenseVol);
  volumeSettingText.setText(buffer);
  sprintf(buffer, "%.3f", diameter);
  diameterText.setText(buffer);
  primeCyclesNumber.setValue(primeCycles);
}

bool checkZero(){
  excludeState(osZeroed);  // disable screen updates, enable zeroing  on trip

  // 1. Move up until trip, set pos = -1 mm
  if(debugPrint) Serial.println("1.Move up...");
  safeMoveTo(-100 * stPmm);
  if(trip){
    motorIsRunning = false;
    trip = false;
    motor.setCurrentPosition(-stPmm);
  }
  else{
    if(debugPrint) Serial.println("Unexpectedly no trip");
    return false;
  }

  // 2. Move 10 mm down without tripping
  safeMoveTo(10 * stPmm);
  if(debugPrint) Serial.println("2. Moved down 10 mm.");
  if(trip) return false;  // do nothing if tripped

  // 3. Move up until trip, set pos = -1 mm
  if(debugPrint) Serial.println("3. Move back up...");
  safeMoveTo(-100 * stPmm);
  if(trip){
    motorIsRunning = false;
    trip = false;
    motor.setSpeed(0);  // Expect to trip at full speed, need to reset to 0 else code will want to slow down first
    // should trip close to previous trip set at -1 mm
    if(abs(motor.currentPosition() + stPmm) > (stPmm/2)){
      Serial.println("Error checking zero point");
      return false;
    }
  }
  else{
    if(debugPrint) Serial.println("Unexpectedly no trip");
    return false;
  }

  // 4. Move to zero
  if(debugPrint) Serial.println("4. Move to 0");
  safeMoveTo(0);
  if(trip) return false;
  includeState(osZeroed);
  return true;
}

void setup(){
  nexInit(115200);
  Serial.begin(115200);
  if(debugPrint) delay(2000);  // delay startup to allow for serial monitor connection
  sendCommand("tsw 255,0"); // disable touch events of all components on screen
  includeState(osBusy);
  if(debugPrint) Serial.printf("Setup executing on core # %d\n", xPortGetCoreID());
//  sendCommand("baud=4800");
//  delay(200);
//  nexSerial.begin(4800);

  initStepperRunner();
  if(debugPrint) Serial.println("Loading config");
  loadConfig();

  // pinMode(Bot, INPUT_PULLUP); //Limit Switch Connected to 0V
  // pinMode(Top, INPUT_PULLUP); //Limit Switch Connected to 0V
  pinMode(doseButton, INPUT_PULLUP); //Switch Connected to 0V

  motor.setPinsInverted (false, false, false);
  pinMode(ms1, OUTPUT); //micro step
  pinMode(ms2, OUTPUT); //micro step
  pinMode(ms3, OUTPUT); //micro step
  pinMode(enable, OUTPUT); //stepper driver
  pinMode(dirPin, OUTPUT); //stepper driver
  pinMode(stepPin, OUTPUT ); //stepper driver
  digitalWrite (ms1, HIGH);
  digitalWrite (ms2, HIGH);
  digitalWrite (ms3, HIGH);
  digitalWrite (enable, LOW); //enable stepper
  delay(1);

  valve.attach(servo); //enable servo
  switchValve(vpOutlet);

  //register the pop events
  primeButton.attachPop(primeButtonPopCallBack);
  switchValveButton.attachPop(switchValveButtonPopCallBack);
  emptyButton.attachPop(emptyButtonPopCallBack);
  upButton.attachPush(upButtonPushCallback);
  upButton.attachPop(up_downButtonPopCallback);
  downButton.attachPush(downButtonPushCallback);
  downButton.attachPop(up_downButtonPopCallback);
  homeButtonP1.attachPop(homeButtonPopCallBack);
  homeButtonP2.attachPop(homeButtonPopCallBack);
  settingsButtonP0.attachPop(settingsButtonPopCallBack);
  settingsButtonP2.attachPop(settingsButtonPopCallBack);
  manualButtonP0.attachPop(manualButtonPopCallBack);
  manualButtonP1.attachPop(manualButtonPopCallBack);
  resetSystemButton.attachPop(resetSystemButtonPopCallback);

  errMsg0.setText("Please Wait    Setting Up"); //Top Line note spacing
  errMsg1.setText("Please Wait"); //P1 top line
  statusText.setText(msgZeroing); //Status
  volumeText.setText("-----"); // Volume

  if(debugPrint) Serial.println(msgZeroing);

  motor.setMinPulseWidth(4);
  motor.setMaxSpeed(maxSpeed*stPmm/4);      // Set Max Speed of Stepper (Slower to get better accuracy)
  motor.setAcceleration(maxSpeed*stPmm/8);  // Set Acceleration of Stepper

  // Plunger zeroing
  if(!checkZero()){
    Serial.println("Error finding zero");
    nexTripAlert();
    return;
  };
  progressBar0.setValue(100); //show slider value as zero
  progressBar1.setValue(100); //Progress bar empty P1
  Serial.println("Homing Completed");

  //reset motor settings after position update
  motor.setMaxSpeed(speed * stPmm);
  motor.setAcceleration(speed * stPmm / 2);

  Serial.println("Setup done");
  errMsg0.setText("Please prime   syringe"); //Top Line note spacing
  errMsg1.setText("Please prime"); //P1 top line
  statusText.setText(msgNotReady); //Status

  excludeState(osBusy);
  sendCommand("tsw 255,1"); // enable all touch events
  if(debugPrint) {
    Serial.println("Debug messages on");
  }
  else {
    Serial.println("Debug messages off");
  }
}

byte prevDosingButtonState = 1; // starting state = on
byte dosingButtonState;

void loop() {
  nexLoop(nex_listen_list);

  // First read button state
  dosingButtonState = digitalRead(doseButton);

  // Then check serial for commands
  if(Serial.available()){
    char c = Serial.read();
    if(containState(osBusy)) {
      Serial.printf("Busy, ignoring command %c\n", c);
    }
    else if(c == '?' || c == 'h'){
      Serial.println("0 : valve to inlet");
      Serial.println("1 : valve to outlet");
      Serial.println("p : prime");
      // Serial.println("+ : move 5 mm up");
      // Serial.println("- : move 5 mm down");
      Serial.println("e : empty");
      Serial.println("d : dispense");
      Serial.println("b : disable debug printing");
      Serial.println("B : enable debug printing");
      Serial.println("R : reset (use this to recover from a trip)");
    }
    else if(c == '0'){
      switchValve(vpInlet);
      Serial.println("Valve to inlet");
    }
    else if(c == '1'){
      switchValve(vpOutlet);
      Serial.println("Valve to outlet");
    }
    else if(c == 'p'){
      primeButtonPopCallBack(0);
    }
    // else if(c == '+'){
    //   upButtonPopCallBack(0);
    // }
    // else if(c == '-'){
    //   downButtonPopCallBack(0);
    // }
    else if(c == 'd'){
      // simulate a button press
      dosingButtonState = 0;
    }
    else if(c == 'e'){
      emptyButtonPopCallBack(0);
    }
    else if(c == 'b'){
      debugPrint = false;
      Serial.println("Debug messages off");
    }
    else if(c == 'B'){
      debugPrint = true;
      Serial.println("Debug messages on");
    }
    else if(c == 'R'){
      resetAll();
    }
  }

  // Check dosing button only if it was previously not pressed
  if((dosingButtonState == 0) && (prevDosingButtonState != 0)){
    prevDosingButtonState = 0;
    dispense();
  }
  else if((dosingButtonState == 1) && (prevDosingButtonState == 0)) {
    prevDosingButtonState = 1;
  }
}
