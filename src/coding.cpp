#ifdef __AVR__
  #include <avr/io.h>
#endif
#include "Arduino.h"
#include "Nextion.h"
#include "SPI.h"
#include "SD.h"
#include "pinconfig.h"
#include "pumpdriver.h"
#include "Servo.h"  // Uses ServoESP32 library.  In platformIO: http://platformio.org/lib/show/1739/ServoESP32
#include <Preferences.h>

#include "espinfo.h"
#include "syringelist.h"
#include "nextioninterface.h"
#include "txtmessages.h"
#include "pumpdriver.h"
#include "esp_ota.h"

#define defaultSpeedPct 50
#define defaultDispenseVolume 5.0
#define defaultPrimeVolume 20
#define defaultPrimeCycles 1

#define configNamespace "pumpSettings"
#define configNameSpeed "speed"
#define configNameDispenseVolume "dispenseVolume"
#define configNamePrimeVolume "primeVolume"
#define configNamePrimeCycles "prime"

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
// Syringe specific data retrieved based on below setting
syringeType syringe = st20;

const float syringeVol = syringeInfo[syringe].vol;
const long strokeLimitSteps = syringeInfo[syringe].stroke * stPmm;

//float stroke;   // max travel distance mm
uint32_t speedPct; // % maximum speed
float dispenseVol, dispenseCycleVol, totalDispensedVol; // requested volume mL
uint32_t primeVol, primeSteps;
byte dispenseCycles;  // number of dispense cycles to dispense total volume
long dispenseSteps = 0;  // stroke per dispense cycle
uint32_t primeCycles; //number of times the syringe cycles
bool displayDispenseVolume = false;

bool debugPrint = false;
bool debugTMC = true;

// -------------------------------------------Servo----------------------------------------------------
Servo valve;
const int in = 0; // servo angle for inlet aliagnment
const int out = 140; // servo angle for outlet aliagnment
enum valvePosition {vpInlet=0, vpOutlet};

//buffer to read values from Nextion
char buf[30] = {0};

// Read/write config parameters
Preferences configStorage;

void updateDosingParams();
void processSerial();

void resetAll() {
  Serial.println("Software reset...");
  nexReset();
  delay(10);  // make sure command is transmitter over serial
  ESP.restart();
}

void safeMoveToUpdateDisplay(float* deltaVolume){
  float progress = (100.0f * (float)(strokeLimitSteps - motor.currentPosition())) / strokeLimitSteps;
  if(progress > 100){
    progress = 100;
  }
  else if(progress < 0){
    progress = 0;
  }
  updateProgressbar0((uint32_t)progress);
  float Vol = syringeVol * (100.0f - progress) / 100.0f;
  dtostrf(Vol, 5, 3, buf);
  updateVolumeTxt(buf);

  // update Volume display
  if(displayDispenseVolume){
    progress = (100.0f * (float)(primeSteps - motor.currentPosition())) / dispenseSteps;
    *deltaVolume = dispenseCycleVol * progress / 100.0f;
    dtostrf(totalDispensedVol + *deltaVolume, 5, 3, buf);
    updateErrorTxt(buf);
  }
}

#define R_SENSE 0.11
#define CS_ACTUAL_bm (1 << 20 | 1 << 19 | 1 << 18 | 1 << 17 | 1 << 16)
#define CS_ACTUAL_bp 16
#define SG_RESULT_bm (1 << 9 | 1 << 8 | 1 << 7 | 1 << 6 | 1 << 5 | 1 << 4 | 1 << 3 | 1 << 2 | 1 << 1 | 1 << 0)
#define SG_RESULT_bp 0
uint16_t rms_current(uint8_t CS) {
  return (float)(CS+1)/32.0 * (0.180)/(R_SENSE+0.02) / 1.41421 * 1000;
}


void safeMoveTo(long newpos){
  // limit max travel to maximum allowed syringe stroke length
  if(newpos > strokeLimitSteps){
    newpos = strokeLimitSteps;
  }

  // Reset trip counter to prevent spurious trip when reversing direction
  motor.stepper_count = 0;

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
  motor.moveToPosition = true;
  motor.running = true;
  float deltaVol = 0;

  if(debugTMC){
    Serial.printf("SGT = %d, Accel = %d\n", driver.sgt(), motor.getAcceleration());
    Serial.println("SG    | Current|  Speed |  TSTEP");
  }

  // Wait until move is finished
  while(motor.running && (motor.trip == false)){
    vTaskDelay(50 / portTICK_PERIOD_MS);

    if(debugPrint) Serial.printf("curPos: %ld\ntargetPos: %ld\nspeed: %0.2f\n",
                  motor.currentPosition(), motor.targetPosition(), motor.speed());
    // Update progress bar with plunger movement
    if(containState(osZeroed) && (nexSerial.availableForWrite() > 50)) {
      safeMoveToUpdateDisplay(&deltaVol);
    }

    if(debugTMC && (Serial.availableForWrite() > 40)){
      uint32_t drv_status = driver.DRV_STATUS();
      // Split out stallGuard value
      Serial.printf("%.4d  |  ", (drv_status & SG_RESULT_bm)>>SG_RESULT_bp);
      Serial.printf("%.4d  |  ", rms_current((drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp));
      Serial.printf("%.4d  |  ", (int) motor.speed());
      Serial.printf("%.4d\n", driver.TSTEP());
    }
  }
  safeMoveToUpdateDisplay(&deltaVol);
  totalDispensedVol += deltaVol;

  // Error diagnostics
  if(motor.trip == true){
    // if not yet zeroed, set zero point 1 mm back
    if(debugPrint) Serial.println("Trip detected");
    if(containState(osZeroed)){
      includeState(osTripped);
      excludeState(osBusy);
      motor.reset();
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

void safeRun(){
  // Reset trip counter to prevent spurious trip when reversing direction
  motor.stepper_count = 0;
  motor.moveToPosition = false;
  motor.running = true;

  if(debugTMC){
    Serial.printf("SGT = %d\n", driver.sgt());
    Serial.println("SG    | Current|  Speed | TSTEP");
  }

  // Wait until move is stopped from pop event
  while(motor.running && (motor.trip == false)){
    processNexMessages();
    processSerial();  // in case a serial stop command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS);
    // Update progress bar with plunger movement
    if(debugTMC && (Serial.availableForWrite() > 40)){
      uint32_t drv_status = driver.DRV_STATUS();
      // Split out stallGuard value
      Serial.printf("%.4d  |  ", (drv_status & SG_RESULT_bm)>>SG_RESULT_bp);
      Serial.printf("%.4d  |  ", rms_current((drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp));
      Serial.printf("%.4d  |  ", (int) motor.speed());
      Serial.printf("%.4d\n", driver.TSTEP());
    }

    if(containState(osZeroed) && (nexSerial.availableForWrite() > 20)) {
      double progress = (100.0 * (float)(strokeLimitSteps - motor.currentPosition())) / strokeLimitSteps;
      if(progress > 100){
        progress = 100;
      }
      else if(progress < 0){
        progress = 0;
      }

      updateProgressbar2NoAck(progress);
      float Vol = syringeVol * (100.0f - progress) / 100.0f;
      updateVolumeTxt2NoAck(Vol);
    }
  }
  if (debugPrint) Serial.println("Stopped running");

  // Need to reset AccelStepper state so that using .run() doesn't give problems
  motor.setCurrentPosition(motor.currentPosition());

  // Error diagnostics
  if(motor.trip == true){
    includeState(osTripped);
    excludeState(osBusy);
    motor.reset();
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
    Serial.printf("currentState=%#.2x prohibitedstates=%#.2x requiredstates=%#.2x\n", currentState, prohibitedStates, requiredStates);
  }
  return result;
}

void switchValve(uint8_t pos) {
  if(!passPreconditions(osTripped | osBusy, 0, "switch valve")) return;
  if(debugPrint)  Serial.printf("switchValve pos = %d\n", pos);

  if((valvePosition)pos == vpInlet){
    valve.write(in);  //actuate Servo to inlet side
    updateValveDisplay(pos);
  }
  else if ((valvePosition)pos == vpOutlet) {
    valve.write(out);  //actuate Servo to inlet side
    updateValveDisplay(pos);
  }
  else if(debugPrint) Serial.printf("Invalid parameter passed to switchValve: %d", pos);
  delay(1000);
}

void prime(){
  if(!passPreconditions(osTripped | osSettings | osManual, osZeroed, "prime")){
    return;
  }
  includeState(osBusy);
  nexDisableScreen();
  Serial.println(msgPriming);

  updateErrorTxt("Please Wait");
  updateStatusTxt(msgPriming);

  totalDispensedVol = 0;
  for (byte i = 0; i < primeCycles; i++){
    excludeState(osBusy);
    switchValve(vpOutlet);
    includeState(osBusy);
    safeMoveTo(0);
    if(containState(osTripped)) return;
    if(debugPrint) Serial.println("Syringe empty");

    excludeState(osBusy);
    switchValve(vpInlet);
    includeState(osBusy);
    safeMoveTo(primeSteps);  //strokeLimitSteps);
    if(containState(osTripped)) return;
    if(debugPrint) Serial.println("Syringe Filled");
  }
  excludeState(osEmpty | osBusy);
  includeState(osPrimed);
  updateErrorTxt(" ");
  updateStatusTxt(msgReady);
  Serial.println(msgReady);
  nexEnableScreen();
  // Empty Serial input buffer
  while(Serial.available()) Serial.read();
}

void updateDosingParams(){
  primeSteps = primeVol / syringeVol * syringeInfo[syringe].stroke * stPmm;
  if(debugPrint) Serial.printf("Prime steps: %ul\n", primeSteps);

  float tmpProgress = (100.0f * (float)(strokeLimitSteps - motor.currentPosition())) / strokeLimitSteps;
  if(tmpProgress > 100) {tmpProgress = 100;}
  else if(tmpProgress < 0) {tmpProgress = 0;}
  updateProgressbar0((uint32_t)tmpProgress);

  motor.speed_mm_s = (_maxSpeed * speedPct) / 100.0f;
  if(debugPrint) Serial.printf("Speed = %d mm/s\n", motor.speed_mm_s);
  motor.highSpeedSettings();

  if(debugPrint) Serial.printf("Syringe volume: %.3f\n", syringeVol);
  dispenseCycles = round((dispenseVol/syringeVol) + 0.5f);
  dispenseCycleVol = dispenseVol / dispenseCycles;
  if(debugPrint) Serial.printf("Dispense cycles: %d\n", dispenseCycles);

  float tmpVol = syringeVol * (100.0f - tmpProgress) / 100.0f;
  dtostrf(tmpVol, 5, 3, buf);
  updateVolumeTxt(buf);

  dispenseSteps = dispenseCycleVol / syringeVol * syringeInfo[syringe].stroke * stPmm;
  if (dispenseSteps > strokeLimitSteps){
    Serial.println("Dosing volume exeeds stroke limit");
    dispenseSteps = strokeLimitSteps;
  }
  if(debugPrint) Serial.printf("dispenseStroke = %ld\n", dispenseSteps);
}

void settingsDone(float tmpDispenseVol, uint32_t tmpPrimeVol, uint32_t tmpPrimeCycles, uint32_t tmpSpeed){
  excludeState(osSettings);
  excludeState(osManual);
  if(debugPrint) Serial.println("Updating settings");

  if(configStorage.begin(configNamespace, false)){  // RW mode
    if(debugPrint) Serial.println("Saving config");

    if(tmpDispenseVol != dispenseVol) {
      dispenseVol = tmpDispenseVol;
      if(debugPrint) Serial.println("Saving dispense volume");
      if(configStorage.putFloat(configNameDispenseVolume, dispenseVol) == 0){
        Serial.println("Error saving dispense volume");
      }
    }

    if(tmpPrimeVol != primeVol) {
      primeVol = tmpPrimeVol;
      if(debugPrint) Serial.println("Saving priming volume");
      if(configStorage.putULong(configNamePrimeVolume, primeVol) == 0){
        Serial.println("Error saving volume");
      }
    }

    if(tmpPrimeCycles != primeCycles) {
      primeCycles = tmpPrimeCycles;
      if(debugPrint) Serial.println("Saving prime cycles");
      if(configStorage.putUShort(configNamePrimeCycles, primeCycles) == 0){
        Serial.println("Error saving prime cycles");
      }
    }

    if(tmpSpeed != speedPct) {
      speedPct = tmpSpeed;
      if(debugPrint) Serial.println("Saving speed");
      if(configStorage.putUShort(configNameSpeed, speedPct) == 0){
        Serial.println("Error saving speed");
      }
    }

    configStorage.end();
  }
  else{
    Serial.println("Error opening config for saving");
  }
  updateDosingParams();
}

// empty the syringe by moving plunger up to zero volume
void emptySyringe(){
  if(!passPreconditions(osTripped | osSettings | osManual | osBusy, 0, "empty")) {
    return;
  }
  Serial.println(msgEmptying);
  updateStatusTxt(msgEmptying);
  switchValve(vpOutlet);
  includeState(osBusy);
  if(debugPrint) Serial.println("valve to outlet");
  safeMoveTo(0);
  Serial.println(msgReady);
  updateStatusTxt(msgReady);
  includeState(osEmpty);
  excludeState(osBusy);
}

void moveUp(){
  if (debugPrint) Serial.println("Moving plunger up.");

  if(!passPreconditions(osTripped | osBusy, 0, "up")) {
    return;
  }
  includeState(osBusy);
  motor.lowSpeedSettings();
  motor.setSpeed(-motor.maxSpeed());
  safeRun();//-speed*stPmm / 8);
}

void moveDown(){
  if (debugPrint) Serial.println("Moving down.");
  if(!passPreconditions(osTripped | osBusy, 0, "up")) {
    return;
  }
  includeState(osBusy);
  motor.lowSpeedSettings();
  motor.setSpeed(motor.maxSpeed());
  safeRun();//speed*stPmm / 8);
}

void stopMove(){
  if (debugPrint) Serial.println("Up_down button pop callback");
  motor.running = false;
  excludeState(osBusy);
}

void settingMode(){
  includeState(osSettings);
  excludeState(osManual);
}

void manualMode(){
  includeState(osManual);
  excludeState(osSettings);
}

void dispense(){
  if(!passPreconditions(osTripped | osSettings | osManual | osBusy, osPrimed, "dispense")){
    const char btnMsg[] = "Dispense active in Home screen only";
    if(containState(osSettings)){
      updateErrorTxt(btnMsg);
    }
    else if(containState(osManual)){
      updateErrorTxt(btnMsg);
    }
    return;
  }
  nexDisableScreen();
  updateStatusTxt(msgDispensing);
  Serial.println(msgDispensing);
  // start dispense cycle from primed position
  if (motor.currentPosition() != primeSteps) {
    switchValve(vpInlet);
    updateStatusTxt(msgFilling);
    includeState(osBusy);
    safeMoveTo(strokeLimitSteps);
  }
  if(containState(osTripped)) return;  // do nothing if tripped

  totalDispensedVol = 0;
  for(byte i=0; i<dispenseCycles; i++){
    excludeState(osBusy);
    switchValve(vpOutlet);
    includeState(osBusy);
    updateStatusTxt(msgDispensing);
    displayDispenseVolume = true;
    safeMoveTo(motor.currentPosition() - dispenseSteps);
    if(containState(osTripped)) return;  // do nothing if tripped

    displayDispenseVolume = false;
    excludeState(osBusy);
    switchValve(vpInlet);
    updateStatusTxt(msgFilling);
    includeState(osBusy);
    safeMoveTo(primeSteps);
    if(containState(osTripped)) return;  // do nothing if tripped
  }
  excludeState(osBusy);
  updateStatusTxt(msgReady);
  Serial.println(msgReady);
  nexEnableScreen();
  // Empty Serial input buffer
  while(Serial.available()) Serial.read();
}

// Reads config data from non-volatile storage
void loadConfig(){
  if(!configStorage.begin(configNamespace, true)){  // Read-only mode
    Serial.println("Error opening config for loading");
  }

  dispenseVol = configStorage.getFloat(configNameDispenseVolume, defaultDispenseVolume);
  primeVol = configStorage.getULong(configNamePrimeVolume, defaultPrimeVolume);
  primeCycles = configStorage.getUShort(configNamePrimeCycles, defaultPrimeCycles);
  speedPct = configStorage.getUShort(configNameSpeed, defaultSpeedPct);
  configStorage.end();
  updateDosingParams();

  // Update display
  updateSettings(dispenseVol, primeVol, primeCycles, speedPct);
}

bool checkZero(){
  excludeState(osZeroed);  // disable screen updates, enable zeroing  on trip
  excludeState(osBusy);
  switchValve(vpOutlet);
  displayDispenseVolume = false;
  // 1. Move up until trip, set pos = -1 mm
  if(debugPrint) Serial.println("1.Move up...");
  safeMoveTo(-100 * stPmm);
  if(motor.trip){
    motor.running = false;
    motor.trip = false;
    motor.setCurrentPosition(-stPmm);
  }
  else{
    if(debugPrint) Serial.println("Unexpectedly no trip");
    return false;
  }

  // 2. Move 10 mm down without tripping
  switchValve(vpInlet);
  safeMoveTo(5 * stPmm);
  if(debugPrint) Serial.println("2. Moved down 5 mm.");
  if(motor.trip) {
    motor.reset();
    return false;  // do nothing if tripped
  }

  // 3. Move up until trip, set pos = -1 mm
  switchValve(vpOutlet);
  if(debugPrint) Serial.println("3. Move back up...");
  safeMoveTo(-100 * stPmm);
  if(motor.trip){
    motor.running = false;
    motor.trip = false;
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
  switchValve(vpInlet);
  if(debugPrint) Serial.println("4. Move to 0");
  safeMoveTo(0);
  if(motor.trip) {
    motor.reset();
    return false;
  }

  includeState(osZeroed);
  excludeState(osBusy);
  return true;
}

void setup(){
  pinMode(dispenseButton, INPUT_PULLUP); //Switch Connected to 0V

  nexInit(115200);
  Serial.begin(115200);
  if(debugPrint) delay(2000);  // delay startup to allow for serial monitor connection
  nexDisableScreen();

  // Set Nextion variable used to limit user input
  setNexMaxVolLimit(syringeInfo[syringe].vol);
  initNextionInterface();

  if(digitalRead(dispenseButton) == LOW){
    updateStatusTxt("OTA");
    runOTA();  // perhaps stop checkZero and only wait for OTA updates...
  }

  includeState(osBusy);
  if(debugPrint) Serial.printf("Setup executing on core # %d\n", xPortGetCoreID());
//  sendCommand("baud=4800");
//  delay(200);
//  nexSerial.begin(4800);

  initStepperRunner();

  if(debugPrint) Serial.println("Loading config");
  loadConfig();

  valve.attach(servo); //enable servo

  updateErrorTxt("Please wait    Setting Up"); //Top Line note spacing
  updateStatusTxt(msgZeroing); //Status
  updateVolumeTxt("-----"); // Volume

  if(debugPrint) Serial.println(msgZeroing);

  motor.lowSpeedSettings();
  // Plunger zeroing
  if(!checkZero()){
    Serial.println("Error finding zero");
    nexTripAlert();
    return;
  };
  updateProgressbar0(100); //show slider value as zero
  updateProgressbar2(100); //Progress bar empty P1
  Serial.println("Homing Completed");

  //reset motor settings after position update
  motor.highSpeedSettings();
  Serial.println("Setup done");
  updateErrorTxt("Please prime   syringe"); //Top Line note spacing
  updateStatusTxt(msgNotReady); //Status

  excludeState(osBusy);
  nexEnableScreen();
  if(debugPrint) {
    Serial.println("Debug messages on");
  }
  else {
    Serial.println("Debug messages off");
  }
  Serial.println(info());
  Serial.println("\nEnter '?' or 'h' for list of serial commands.\n");
}

byte prevDosingButtonState = 1; // starting state = on
byte dosingButtonState;

void processSerial(){
  if(Serial.available()){
    char c = Serial.read();
    if(containState(osBusy) && !(('R' == c) || ('?' == c) || ('h' == c) ||
                                 ('/' == c) || ('\n' == c) || ('\r' == c))) {
      Serial.printf("Busy, ignoring command %c\n", c);
    }
    else if(c == '?' || c == 'h'){
      Serial.println("? or h : show this list");
      Serial.println("+ : move up, enter / to stop");
      Serial.println("- : move down, enter / to stop");
      Serial.println("0 : valve to inlet");
      Serial.println("1 : valve to outlet");
      Serial.println("b : disable debug printing");
      Serial.println("B : enable debug printing");
      Serial.println("d : dispense");
      Serial.println("e : empty");
      Serial.println("h xxx : Set high speed SGT");
      Serial.println("i : show platform information");
      Serial.println("l xxx : Set low speed SGT");
      Serial.println("p : prime");
      Serial.println("R : reset (use this to recover from a trip)");
      Serial.println("r : clear trip (no reboot, testing only)");
      Serial.println("z : Zero");
    }
    else if(c == '/'){
      if(motor.running && !motor.moveToPosition){
        stopMove();
      }
      else{
        Serial.println("Motor not in free running mode");
      }
    }
    else if(c == '+'){
      moveUp();
    }
    else if(c == '-'){
      moveDown();
    }
    else if(c == '0'){
      switchValve(vpInlet);
      Serial.println("Valve to inlet");
    }
    else if(c == '1'){
      switchValve(vpOutlet);
      Serial.println("Valve to outlet");
    }
    else if(c == 'b'){
      debugPrint = false;
      Serial.println("Debug messages off");
    }
    else if(c == 'B'){
      debugPrint = true;
      Serial.println("Debug messages on");
    }
    else if(c == 'd'){
      dispense();
    }
    else if(c == 'e'){
      emptySyringe();
    }
    else if(c == 'i'){
      Serial.print(info());
    }
    else if(c == 'p'){
      prime();
    }
    else if(c == 'R'){
      resetAll();
    }
    else if(c == 'r'){
      digitalWrite(motorEnablePin, LOW);
      digitalWrite(TMC_VIO, HIGH);
      motor.running = false;
      currentState = osUnInitialized;
      sendCommand("page 0");
      includeState(osZeroed);
    }
    else if(c == 'h'){
      char buf[30];
      uint8_t i = 0;
      int val;
      c = Serial.read();
      while((c != '\n') && (c != '\r') && (i < 29)){
        if(c != ' ') {
          buf[i] = c;
          i++;
        }
        c = Serial.read();
      }
      buf[i] = 0;
      Serial.printf("Read hSG: %s\n", buf);
      val = atoi(buf);
      motor.hSG = val;
    }
    else if(c == 'l'){
      char buf[30];
      uint8_t i = 0;
      int val;
      c = Serial.read();
      while((c != '\n') && (c != '\r') && (i < 29)){
        if(c != ' ') {
          buf[i] = c;
          i++;
        }
        c = Serial.read();
      }
      buf[i] = 0;
      Serial.printf("Read lSG: %s\n", buf);
      val = atoi(buf);
      motor.lSG = val;
    }
    else if(c == 'z'){
      motor.lowSpeedSettings();
      if(!checkZero()){
        Serial.println("Error finding zero");
        nexTripAlert();
      };
      updateProgressbar0(100); //show slider value as zero
      updateProgressbar2(100); //Progress bar empty P1
      Serial.println("Homing Completed");

      motor.highSpeedSettings();
    }
  }
}

void loop() {
  processNexMessages();
  processSerial();

  dosingButtonState = digitalRead(dispenseButton);
  // Check dosing button only if it was previously not pressed
  if((dosingButtonState == 0) && (prevDosingButtonState != 0)){
    prevDosingButtonState = 0;
    dispense();
  }
  else if((dosingButtonState == 1) && (prevDosingButtonState == 0)) {
    prevDosingButtonState = 1;
  }
}
