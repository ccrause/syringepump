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
#include "checkreset.h"

#define defaultHighSpeedPct 80
#define defaultLowSpeedPct 40
#define defaultDispenseVolume 5.0
#define defaultPrimeVolume 10
#define defaultPrimeCycles 1

#define configNamespace "pumpSettings"
#define configNameHighSpeed "highSpeed"
#define configNameLowSpeed "LowSpeed"
#define configNameDispenseVolume "dispenseVolume"
#define configNamePrimeVolume "primeVolume"
#define configNamePrimeCycles "prime"

enum OpState {osUnInitialized=0,  // startup state
              osZeroed=1,         // zeroing completed
              osPrimed=2,         // prime cycle completed
              osFilling=4,        // busy filling - used for titrate refill display
              osTripped=128};     // motor is tripped
OpState currentState = osUnInitialized;
#define includeState(state) currentState = (OpState)(currentState | state)
#define excludeState(state) currentState = (OpState)(currentState & ~(state))
#define containState(state) ((int)(currentState & state) == (int)state)

// Actually the current screen
enum modeState {msHome,
                msDispense,
                msTitrate,
                msSettings,
                msManual};
modeState currentMode = msHome;

//--------------------------------------------Variables-----------------------------------------------
// Syringe specific data retrieved based on below setting
syringeType syringe = st20;

const float syringeVol = syringeInfo[syringe].vol;
const long strokeLimitSteps = syringeInfo[syringe].stroke * stPmm;

uint32_t highSpeedPct, lowSpeedPct;
float dispenseVol, dispenseCycleVol, totalDispensedVol; // requested volume mL
uint32_t primeVol, primeSteps;
byte dispenseCycles;  // number of dispense cycles to dispense total volume
long dispenseSteps = 0;  // stroke per dispense cycle
uint32_t primeCycles; //number of times the syringe cycles
bool displayDispenseVolume = false;

bool debugPrint = false;
bool debugTMC = false;
bool noNexPassFailMsg = true;

// -------------------------------------------Servo----------------------------------------------------
Servo valve;
const int in = 0; // servo angle for inlet alignment
const int out = 140; // servo angle for outlet alignment
enum valvePosition {vpInlet=0, vpOutlet};

//buffer to read values from Nextion
char buf[30] = {0};

// Read/write config parameters
Preferences configStorage;

float currentPlungerPosition, currentSyringeVolume, deltaVol;
bool titrateSlowSpeed = false;

void updateDosingParams();
void processSerial();

void resetAll() {
  Serial.println("Software reset...");
  nexReset();
  delay(10);  // make sure command is transmitter over serial
  ESP.restart();
}

void safeMoveToUpdateDisplay(float* deltaVolume){
  currentPlungerPosition = (100.0f * (float)(strokeLimitSteps - motor.currentPosition())) / strokeLimitSteps;
  if(currentPlungerPosition > 100){
    currentPlungerPosition = 100;
  }
  else if(currentPlungerPosition < 0){
    currentPlungerPosition = 0;
  }

  currentSyringeVolume = syringeVol * (100.0f - currentPlungerPosition) / 100.0f;
  dtostrf(currentSyringeVolume, 5, 3, buf);

  if(currentMode == msHome){
    updateProgressbarHome((uint32_t)currentPlungerPosition);
    updateVolumeTxt0(buf);
  }
  else if(currentMode == msDispense){
    updateProgressbarDispense((uint32_t)currentPlungerPosition);
    updateVolumeTxt1(buf);
  }
  else if (currentMode == msTitrate){
    updateProgressbarTitrate((uint32_t)currentPlungerPosition);
    updateVolumeTxt2(buf);
  }
  else if (currentMode == msManual){
    updateProgressbarManual((uint32_t)currentPlungerPosition);
    updateVolumeTxt4(buf);
  }

  // update Volume display
  if(displayDispenseVolume){
    currentPlungerPosition = (100.0f * (float)(primeSteps - motor.currentPosition())) / dispenseSteps;
    *deltaVolume = dispenseCycleVol * currentPlungerPosition / 100.0f;
    dtostrf(totalDispensedVol + *deltaVolume, 5, 3, buf);
    switch(currentMode){
      case msDispense:{
        updateErrorTxt1(buf);
        break;
      }
      case msTitrate:{
        updateErrorTxt2(buf);
        break;
      }
    }
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
  motor.running = true;

  // only zero if in dispense mode
  // titrate mode does a manual zero from button press
  if(currentMode == msDispense) deltaVol = 0;

  if(debugTMC){
    Serial.printf("SGT = %d, Accel = %d\n", driver.sgt(), motor.getAcceleration());
    Serial.println("SG    | Current|  Speed |  TSTEP");
  }

  // Wait until move is finished
  while(motor.running && (motor.trip == false)){
    vTaskDelay(50 / portTICK_PERIOD_MS);

    if(currentMode == msTitrate  && !containState(osFilling)){
      if(digitalRead(dispenseButton) != 0){
        motor.running = false;
        if(debugPrint) Serial.println("Titrate button released");
      }
    }
    else if (currentMode == msManual){
      processNexMessages();  // wait for button release event
    }

//    if(debugPrint) Serial.printf("curPos: %ld\ntargetPos: %ld\nspeed: %0.2f\n",
//                  motor.currentPosition(), motor.targetPosition(), motor.speed());
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

  if(containState(osZeroed)) safeMoveToUpdateDisplay(&deltaVol);
  if(currentMode == msDispense) totalDispensedVol += deltaVol;

  // Error diagnostics
  if(motor.trip == true){
    // if not yet zeroed, set zero point 1 mm back
    if(debugPrint) Serial.println("Trip detected");
    if(containState(osZeroed)){
      includeState(osTripped);
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

// empty the syringe by moving plunger up to zero volume
void emptySyringe(){
  if(containState(osTripped)) {
    Serial.println("Cannot empty syringe because state=TRIP");
    return;
  }
  nexDisableScreen();
  Serial.println(msgEmptying);
  updateStatusTxt(msgEmptying);
  switchValve(vpOutlet);
  if(debugPrint) Serial.println("valve to outlet");
  safeMoveTo(0);
  if(containState(osTripped)){
    nexTripAlert();
    motor.reset();
    return;
  }

  Serial.println(msgReady);
  updateStatusTxt(msgReady);
  nexEnableScreen();
}

void fillSyringe(){
  if(containState(osTripped)) {
    Serial.println("Cannot fill syringe because state=TRIP");
    return;
  }
  includeState(osFilling);
  nexDisableScreen();
  Serial.println(msgFilling);
  updateStatusTxt(msgFilling);
  switchValve(vpInlet);
  if(debugPrint) Serial.println("valve to inlet");
  // Reset previous speed
  motor.setCurrentPosition(motor.currentPosition());
  safeMoveTo(primeSteps);
  if(containState(osTripped)){
    nexTripAlert();
    motor.reset();
    return;
  }
  Serial.println(msgReady);
  updateStatusTxt(msgReady);
  nexEnableScreen();
  excludeState(osFilling);
  if(debugPrint) Serial.println("Syringe Filled");
}

void prime(){
  if(!containState(osZeroed)){
    Serial.println("Must ZERO before PRIME");
    return;
  }
  nexDisableScreen();
  Serial.println(msgPriming);
  motor.highSpeedSettings();
  displayDispenseVolume = false;

  updateErrorTxt("Please Wait");
  updateStatusTxt0(msgPriming);

  totalDispensedVol = 0;
  for (byte i = 0; i < primeCycles; i++){
    switchValve(vpOutlet);
    // Reset previous speed
    motor.setCurrentPosition(motor.currentPosition());
    safeMoveTo(0);
    if(containState(osTripped)) return;
    if(debugPrint) Serial.println("Syringe empty");

    fillSyringe();
  }
  includeState(osPrimed);
  updateErrorTxt(" ");
  updateStatusTxt0(msgReady);
  Serial.println(msgReady);
  nexEnableDispense();
  nexEnableTitrate();

  nexEnableScreen();
  // Empty Serial input buffer
  while(Serial.available()) Serial.read();
}

void updateDosingParams(){
  primeSteps = primeVol / syringeVol * syringeInfo[syringe].stroke * stPmm;
  if(debugPrint) Serial.printf("Prime steps: %u\n", primeSteps);

  float tmpProgress = (100.0f * (float)(strokeLimitSteps - motor.currentPosition())) / strokeLimitSteps;
  if(tmpProgress > 100) {tmpProgress = 100;}
  else if(tmpProgress < 0) {tmpProgress = 0;}
  updateProgressbarHome((uint32_t)tmpProgress);

  motor.highSpeed_mm_s = (_maxSpeed * highSpeedPct + 0.5) / 100.0f;
  if (motor.highSpeed_mm_s == 0) motor.highSpeed_mm_s = 1;
  if(debugPrint) Serial.printf("High speed = %d mm/s\n", motor.highSpeed_mm_s);
  motor.highSpeedSettings();

  motor.lowSpeed_mm_s = (_maxSpeed * lowSpeedPct + 0.5) / 100.0f;
  if (motor.lowSpeed_mm_s == 0) motor.lowSpeed_mm_s = 1;
  if(debugPrint) Serial.printf("Low speed = %d mm/s\n", motor.lowSpeed_mm_s);

  if(debugPrint) Serial.printf("Syringe volume: %.3f\n", syringeVol);
  dispenseCycles = round((dispenseVol/primeVol) + 0.499f);
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

void settingsDone(float tmpDispenseVol, uint32_t tmpPrimeVol, uint32_t tmpPrimeCycles,
                  uint32_t tmpHighSpeed, uint32_t tmpLowSpeed){
  if(debugPrint) Serial.println("Updating settings");

  if(configStorage.begin(configNamespace, false)){  // RW mode
    if(debugPrint) Serial.println("Saving config");

    if(tmpDispenseVol != dispenseVol) {
      if(tmpDispenseVol > 0){
        dispenseVol = tmpDispenseVol;
      }
      else{
        dispenseVol = defaultDispenseVolume;
      }
      if(debugPrint) Serial.println("Saving dispense volume");
      if(configStorage.putFloat(configNameDispenseVolume, dispenseVol) == 0){
        Serial.println("Error saving dispense volume");
      }
    }

    if(tmpPrimeVol != primeVol) {
    // check for illogical input values
      if(tmpPrimeVol > 0){
        primeVol = tmpPrimeVol;
      }
      else{
        primeVol = defaultPrimeVolume;
      }
      if(debugPrint) Serial.println("Saving priming volume");
      if(configStorage.putULong(configNamePrimeVolume, primeVol) == 0){
        Serial.println("Error saving volume");
      }
    }

    if(tmpPrimeCycles != primeCycles) {
      if(tmpPrimeCycles > 0){
        primeCycles = tmpPrimeCycles;
      }
      else{
        primeCycles = defaultPrimeCycles;
      }
      if(debugPrint) Serial.println("Saving prime cycles");
      if(configStorage.putUShort(configNamePrimeCycles, primeCycles) == 0){
        Serial.println("Error saving prime cycles");
      }
    }

    if(tmpHighSpeed != highSpeedPct) {
      if(tmpHighSpeed > 0){
        highSpeedPct = tmpHighSpeed;
      }
      else{
        highSpeedPct = defaultHighSpeedPct;
      }
      if(debugPrint) Serial.println("Saving high speed");
      if(configStorage.putUShort(configNameHighSpeed, highSpeedPct) == 0){
        Serial.println("Error saving high speed");
      }
    }

    if(tmpLowSpeed != lowSpeedPct) {
      if(tmpLowSpeed > 0){
        lowSpeedPct = tmpLowSpeed;
      }
      else{
        lowSpeedPct = tmpLowSpeed;
      }
      if(debugPrint) Serial.println("Saving low speed");
      if(configStorage.putUShort(configNameLowSpeed, lowSpeedPct) == 0){
        Serial.println("Error saving low speed");
      }
    }

    configStorage.end();
  }
  else{
    Serial.println("Error opening config for saving");
  }
  updateDosingParams();
}


void moveUp(){
  if (debugPrint) Serial.println("Moving plunger up.");

  if(containState(osTripped)) {
    Serial.println("Cannot move because state=TRIP");
    return;
  }
  // Hack to clear internal state of AccelSteppr
  motor.setCurrentPosition(motor.currentPosition());
  motor.lowSpeedSettings();
  if(debugPrint)  Serial.printf("MaxSpeed = %.3f\n", motor.maxSpeed());
  displayDispenseVolume = false;
  safeMoveTo(0);
}

void moveDown(){
  if (debugPrint) Serial.println("Moving down.");
  if(containState(osTripped)) {
    Serial.println("Cannot move because state=TRIP");
    return;
  }
  // Hack to clear internal state of AccelSteppr
  motor.setCurrentPosition(motor.currentPosition());
  motor.lowSpeedSettings();
  displayDispenseVolume = false;
  safeMoveTo(strokeLimitSteps);
}

void stopMove(){
  motor.running = false;
}

void settingMode(){
  currentMode = msSettings;
}

bool titrateMode(){
  if(containState(osPrimed)){
    currentMode = msTitrate;
    totalDispensedVol = 0;
    return true;
  }
  else{
    return false;
  }
}

void setTitrateRate(bool slowRate){
  titrateSlowSpeed = slowRate;
}

void doZeroTitrateTotal(){
  // reset speed to 0
  motor.setCurrentPosition(motor.currentPosition());
  displayDispenseVolume = false;
  fillSyringe();
  switchValve(vpOutlet);
  totalDispensedVol = 0;
  updateErrorTxt2("");
}

bool dispenseMode(){
  if(containState(osPrimed)){
    currentMode = msDispense;
    return true;
  }
  else{
    return false;
  }
}

void manualMode(){
  currentMode = msManual;
}

void homeMode(){
  currentMode = msHome;
}

void dispense(){
  if(!passPreconditions(osTripped, osPrimed, "dispense") && !(currentMode == msDispense)) {
    const char btnMsg[] = "Dispense active in Dispense screen only";
    updateErrorTxt(btnMsg);
    return;
  }
  nexDisableScreen();
  updateStatusTxt1(msgDispensing);
  Serial.println(msgDispensing);
  motor.highSpeedSettings();
  motor.setCurrentPosition(motor.currentPosition());
  // start dispense cycle from primed position
  if (motor.currentPosition() != primeSteps) {
    switchValve(vpInlet);
    updateStatusTxt1(msgFilling);
    safeMoveTo(primeSteps);
  }
  if(containState(osTripped)) return;  // do nothing if tripped

  totalDispensedVol = 0;
  for(byte i=0; i<dispenseCycles; i++){
    switchValve(vpOutlet);
    updateStatusTxt1(msgDispensing);
    displayDispenseVolume = true;
    safeMoveTo(motor.currentPosition() - dispenseSteps);
    if(containState(osTripped)) return;  // do nothing if tripped

    displayDispenseVolume = false;
    switchValve(vpInlet);
    updateStatusTxt1(msgFilling);
    safeMoveTo(primeSteps);
    if(containState(osTripped)) return;  // do nothing if tripped
  }
  updateStatusTxt1(msgReady);
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
  highSpeedPct = configStorage.getUShort(configNameHighSpeed, defaultHighSpeedPct);
  lowSpeedPct = configStorage.getUShort(configNameLowSpeed, defaultLowSpeedPct);
  configStorage.end();
  updateDosingParams();

  // Update display
  updateSettingsDisplay(dispenseVol, primeVol, primeCycles, highSpeedPct, lowSpeedPct);
}

bool doZero(){
bool isOK;
  excludeState(osZeroed);  // disable screen updates, enable zeroing  on trip
  switchValve(vpOutlet);
  motor.lowSpeedSettings();
  displayDispenseVolume = false;
  // 1. Move up until trip, set pos = -1 mm
  if(debugPrint) Serial.println("1.Move up...");
  safeMoveTo(-100 * stPmm);
  if(motor.trip){
    motor.running = false;
    motor.trip = false;
    motor.setCurrentPosition(-stPmm);
    isOK = true;
  }
  else{
    if(debugPrint) Serial.println("Unexpectedly no trip");
    motor.trip = true;
    isOK = false;
  }

  // 2. Move 10 mm down without tripping
  if(isOK){
    switchValve(vpInlet);
    safeMoveTo(2 * stPmm);
    if(debugPrint) Serial.println("2. Moved down 2 mm.");
    if(motor.trip) {
      motor.reset();
      isOK = false;
    }
  }

  // 3. Move up until trip, set pos = -1 mm
  if(isOK){
    switchValve(vpOutlet);
    if(debugPrint) Serial.println("3. Move back up...");
    safeMoveTo(-100 * stPmm);
    if(motor.trip){
      motor.running = false;
      motor.setSpeed(0);  // Expect to trip at full speed, need to reset to 0 else code will want to slow down first
    // should trip close to previous trip set at -1 mm
      if(abs(motor.currentPosition() + stPmm) > (stPmm/2)){
        Serial.println("Error verifying zero");
        isOK = false;
      }
      motor.trip = false;
    }
    else{
      if(debugPrint) Serial.println("Unexpectedly no trip");
      motor.trip = true;
      isOK = false;
    }
  }
  // 4. Move to zero
  if(isOK){
    switchValve(vpInlet);
    if(debugPrint) Serial.println("4. Move to 0");
    safeMoveTo(0);
    if(motor.trip) {
      isOK = false;
    }
  }

  if(isOK){
    includeState(osZeroed);
    nexEnablePrime();
  }
  else{
    motor.reset();
    nexTripAlert();
  }

  return isOK;
}

void setup(){
  pinMode(dispenseButton, INPUT_PULLUP); //Switch Connected to 0V

  nexInit(115200);
  Serial.begin(115200);
  sendCommand("bkcmd=0");

  // Print CPU0 reset reason on Nextion
  updateErrorTxt(reset_reason());

  // Reset default messages on all screens
  updateStatusTxt("Press zero to start");
  updateVolumeTxt("-----");

  //if(debugPrint)
//  delay(2000);  // delay startup to allow for serial monitor connection

  // Set Nextion variable used to limit user input
  setNexMaxVolLimit(syringeInfo[syringe].vol);
  initNextionInterface();

  if(digitalRead(dispenseButton) == LOW){
    updateStatusTxt0("OTA");
    runOTA();  // perhaps stop checkZero and only wait for OTA updates...
  }

  nexDisableDispense();
  nexDisablePrime();
  nexDisableTitrate();

  if(debugPrint) Serial.printf("Setup executing on core # %d\n", xPortGetCoreID());

  initStepperRunner();

  if(debugPrint) Serial.println("Loading config");
  loadConfig();

  valve.attach(servo); //enable servo

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
    if(c == '?'){
      Serial.println("? : show this list");
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
      Serial.println("m : Disable TMC controller printing");
      Serial.println("M : Enable TMC controller printing");
      Serial.println("p : prime");
      Serial.println("r : clear trip (no reboot, testing only)");
      Serial.println("R : reset (use this to recover from a trip)");
      Serial.println("z : Zero");
    }
    else if(c == '/'){
      if(motor.running){
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
    else if(c == 'm'){
      debugTMC = false;
      Serial.println("TMC messages off");
    }
    else if(c == 'M'){
      debugTMC = true;
      Serial.println("TMC messages on");
    }
    else if(c == 'p'){
      prime();
    }
    else if(c == 'r'){
      digitalWrite(motorEnablePin, LOW);
      digitalWrite(TMC_VIO, HIGH);
      motor.running = false;
      currentState = osUnInitialized;
      sendCommand("page 0");
      includeState(osZeroed);
    }
    else if(c == 'R'){
      resetAll();
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
      doZero();
      if(motor.trip){
        Serial.println("Error finding zero");
        nexTripAlert();
      };
      updateProgressbarHome(100); //show slider value as zero
      updateProgressbarTitrate(100); //Progress bar empty P1
      Serial.println("Homing Completed");

      motor.highSpeedSettings();
    }
  }
}

void loop() {
  processNexMessages();
  processSerial();

  dosingButtonState = digitalRead(dispenseButton);
  if(currentMode == msDispense){
    // Check dosing button only if it was previously not pressed
    if((dosingButtonState == 0) && (prevDosingButtonState != 0)){
      prevDosingButtonState = 0;

      dispense();
    }
    else if((dosingButtonState != 0) && (prevDosingButtonState == 0)) {
      prevDosingButtonState = 1;
    }
  }
  // This needs to loop and read button and Nextion message
  else if (currentMode == msTitrate){
    if (dosingButtonState == 0){
      displayDispenseVolume = true;
      if(titrateSlowSpeed) motor.lowSpeedSettings();
      else motor.highSpeedSettings();

      // Hack to clear internal state of AccelStepper
      motor.setCurrentPosition(motor.currentPosition());
      safeMoveTo(0);
      if (motor.currentPosition() == 0){
        totalDispensedVol += deltaVol;
        displayDispenseVolume = false;
        fillSyringe();
        switchValve(vpOutlet);
      }
    }
  }
}
