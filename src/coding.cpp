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

#define stPmm 800 //800; //steps per mm
#define pi 3.14159265
#define primeCycles 2 //number of times the syringe cycles
#define configNamespace "pumpSettings"
#define configNameStroke "stroke"
#define configNameSpeed "speed"
#define configNameVolume "volume"
#define configNameDiameter "diameter"

enum OpState {osUnInitialized=0,  // default state
              osZeroed=1,         // set after zeroing completed
              osEmpty=2,          // set when empty action completed
              osPrimed=4,         // set after prime cycle completed
              osSettings=8,       // set when in settings screen
              osTripped=128};     // set when motor is tripped
OpState currentState = osUnInitialized;
#define includeState(state,b) state = (OpState)(state | b)
#define excludeState(state,b) state = (OpState)(state & ~b)
#define containState(state,b) ((int)(state & b) == (int)b)

//--------------------------------------------Variables-----------------------------------------------
uint32_t stroke;   // max travel distance mm
uint32_t speedPct; // % maximum speed
float dispenseVol, dispenseCycleVol, totalDispensedVol; // requested volume mL
byte dispenseCycles;  // number of dispense cycles to dispense total volume
long dispenseStroke = 0;  // stroke per dispense cycle
float diameter;    // syringe diameter mm
uint32_t speed;    // Actual speed in mm/sec

long strokePosLimit = defaultStroke * stPmm; // max stroke position in steps (default 100 mm)
bool displayDispenseVolume = false;

bool debugPrint = false;

// -------------------------------------------Servo----------------------------------------------------
Servo valve;
const int in = 0; // servo angle for inlet aliagnment
const int out = 140; // servo angle for outlet aliagnment
enum valvePosition {vpInlet=0, vpOutlet};

//-------------------------------------------------NEXTION---------------------------------------------
NexPage page0 = NexPage(0, 0, "page0"); // Home Screen
NexPage page1 = NexPage(1, 0, "page1"); // Settings Screen

// Page 0 (Home)

NexButton primeButton = NexButton(0, 1, "b0"); //Prime Syringe
NexButton emptyButton = NexButton(0, 2, "b1"); // Empty Syringe
NexButton settingsButton = NexButton(0, 3, "b2"); //Page1 not used in mcu
NexProgressBar progressBar0 = NexProgressBar(0, 5, "j0"); //Syringe Slider 0=100% Full
NexDSButton valvePosition0 = NexDSButton(0, 4, "bt0"); // Actual Valve Position 0=IN 1=Out
NexText errMsg0 = NexText(0, 8, "t2"); //Error Display 28 Carracters max
NexText statusText = NexText(0, 6, "t0"); //Status Text Ready, Running, Filling. Error
NexText volumeText = NexText(0, 7, "t1"); //Current Syringe Volume

//Page 1 (Settings)
NexText errMsg1 = NexText(1, 9, "t4"); // Errror Diplay 15 caracters max
NexDSButton switchValveButton = NexDSButton(1, 6, "bt1"); // Switch Valve Position 0=In 1=Out
NexButton upButton = NexButton(1, 2, "b2"); // Move syringe UP
NexButton downButton = NexButton(1, 3, "b3"); // Move Syringe Down
NexButton homeButton = NexButton(1, 1, "b0"); // Home Page button update the values for the syringe by reading the vairious values
NexDSButton valvePosition1 = NexDSButton(1, 4, "bt0"); // Actual Valve Position 0=In 1=Out
NexProgressBar progressBar1 = NexProgressBar(1, 5, "j0"); //Syringe Slider 0=100% Full
NexText volumeSettingText = NexText(1, 7, "t0"); //Set Volume
NexText diameterText = NexText(1, 8, "t1"); //Syringe Diameter mm Float Not visable
NexNumber strokeNumber = NexNumber(1, 11, "n1");
NexNumber speedNumber = NexNumber(1, 10, "n0");

//buffer to read values from Nextion
char buffer[30] = {0};

// Buttons that have a executable action assigned to them
NexTouch *nex_listen_list[] = {
  &primeButton, // Prime
  &homeButton,// Update Diameter, Stroke and Speed
  &switchValveButton, // Change Valve position
  &emptyButton, //Empty Syringe
  &upButton, //Move up 5mm
  &downButton, //Move down 5mm
  &settingsButton,
  NULL
};

// Read/write config parameters
Preferences configStorage;

void updateDosingParams();

void nexTripAlert(){
  #define screen_width 272
  #define screen_height 483
  #define w 186
  #define h 100
  #define x ((screen_width - w) / 2)
  #define y ((screen_height - h) / 2)
  #define fontID 2
  #define bufLen 60
  char commandBuffer[bufLen];

  snprintf(commandBuffer, bufLen, "xstr %d,%d,%d,%d,%d,%s,%s,%d,%d,%d,\"%s\"",
           x, y, w, h, fontID, "BLACK", "RED", 1, 1, 1, "TRIP");
  sendCommand(commandBuffer);
  if(!recvRetCommandFinished(100)){
    Serial.println("Error drawing text:");
    Serial.println(commandBuffer);
  }
}

void safeMoveTo(long newpos){
  // limit max travel to maximum allowed syringe stroke length
  if(newpos > strokePosLimit){
    newpos = strokePosLimit;
  }

  if(debugPrint) Serial.printf("New position: %ld\n", newpos);

  // Reset trip counter to prevent spurious trip when reversing direction
  stepper_count = 0;
  motor.moveTo(newpos);
  motorIsRunning = true;
  float deltaVol = 0;

  // Wait until move is finished
  while(motorIsRunning && (trip == false)){
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // Update progress bar with plunger movement
    if(containState(currentState, osZeroed) && (nexSerial.availableForWrite() > 20)) {
      double progress = (100.0 * (float)(strokePosLimit - motor.currentPosition())) / strokePosLimit;
      if(progress > 100){
        progress = 100;
      }
      else if(progress < 0){
        progress = 0;
      }
      progressBar0.setValue((uint32_t)progress);
      progressBar1.setValue((uint32_t)progress);

      // update Volume display
      if(displayDispenseVolume){
        progress = (100.0 * (float)(strokePosLimit - motor.currentPosition())) / dispenseStroke;
        deltaVol = dispenseCycleVol * progress / 100.0;
        dtostrf(totalDispensedVol + deltaVol, 5, 3, buffer);
        volumeText.setText(buffer);
      }
    }
  }
  totalDispensedVol += deltaVol;

  // Error diagnostics
  if(trip == true){
    includeState(currentState, osTripped); //currentState |= osTripped;
    const char * msg1 = "==TRIP==";
    const char * msg2 = "TRIP: please service & restart";
    Serial.println(msg2);
    errMsg0.setText(msg2);
    errMsg1.setText(msg2);
    statusText.setText(msg1);
    sendCommand("tsw 255,0"); // disable touch events of all components on screen
    nexTripAlert();
  }
  else if(motor.distanceToGo() != 0){
    if(debugPrint){
      Serial.println("safeMove didn't complete distance.");
      Serial.printf("Current position: %ld\n", motor.currentPosition());
    }
    vTaskDelay(1);
    if(digitalRead(Top) == 0){
      if(debugPrint) Serial.println("Hit top trip, setting pos=0");
      motor.setCurrentPosition(0);
    }
    else if (digitalRead(Bot) == 0){
      if(debugPrint) Serial.println("Hit top trip, setting pos=100");
      motor.setCurrentPosition(100 * stPmm);
    }
  }
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
  else if(debugPrint) Serial.printf("Invalid parametr passed to switchValve: %d", pos);
}

//Filling the syringe to remove air
void primeButtonPopCallBack(void *ptr){
  if(containState(currentState, osTripped) || containState(currentState, osSettings)) return;  // do nothing if tripped
  if(!containState(currentState, osZeroed)) {
    Serial.println("Cannot prime until ZEROED");
    return;
  }
  Serial.println("Priming started");
  errMsg0.setText("Please Wait");
  statusText.setText("Priming"); // nextion status
  totalDispensedVol = 0;
  for (byte i = 0; i < (primeCycles); i++){
    switchValve(vpOutlet);
    delay(250);       // give time for servo to move
    safeMoveTo(0);
    if((currentState & osTripped) == osTripped) return;  // do nothing if tripped
    if(debugPrint) Serial.println("Syringe empty");

    switchValve(vpInlet);
    delay(250);       // give time for servo to move
    safeMoveTo(strokePosLimit);
    if((currentState & osTripped) == osTripped) return;  // do nothing if tripped
    if(debugPrint) Serial.println("Syringe Filled");
  }
  excludeState(currentState, osEmpty); // currentState = (OpState)(currentState & ~osEmpty);
  includeState(currentState, osPrimed); // currentState |= (OpState)osPrimed;
  errMsg0.setText(" ");
  statusText.setText("Ready"); // nextion status
  Serial.println("Priming finished");
}

//actuate the three way valve invert the current position
void switchValveButtonPopCallBack(void *prt){
  if(containState(currentState, osTripped)) return; //if((currentState & osTripped) == osTripped) return;  // do nothing if tripped
  uint32_t dual_state;
  switchValveButton.getValue(&dual_state);
  if (!dual_state) {
    switchValve(vpInlet);
    Serial.println("valve to inlet");
  }
  else {
    switchValve(vpOutlet);
    Serial.println("valve to outlet");
  }
}

// empty the syringe by moving plunger up to zero volume
void emptyButtonPopCallBack(void *prt){
  if(containState(currentState, osTripped) || containState(currentState, osSettings)) return;  // do nothing if tripped
  Serial.println("Empty syringe");
  switchValve(vpOutlet);
  delay(250);
  if(debugPrint) Serial.println("valve to outlet");
  safeMoveTo(0);
  if(debugPrint) Serial.println("Syringe empty");
  includeState(currentState, osEmpty); // currentState |= osEmpty;
}

// move the plunger up 5mm if possible
void upButtonPopCallBack(void *prt){
  if(containState(currentState, osTripped)) return;  // do nothing if tripped
  safeMoveTo(motor.currentPosition() - 5*stPmm);
  if (digitalRead(Top))
    Serial.println("Up 5mm");
  else
    Serial.println("Hit top stop!!!");
}

// move the plunger down 5mm if posible
void downButtonPopCallBack(void *prt){
  if(containState(currentState, osTripped)) return;  // do nothing if tripped
  safeMoveTo(motor.currentPosition() + 5*stPmm);
  if (digitalRead(Bot))
    Serial.println("Down 5mm");
  else
    Serial.println("Hit bottom stop!!!");
}

void settingsButtonPopCallBack(void *ptr) {
  includeState(currentState, osSettings);
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

void updateDosingParams(){
  strokePosLimit = stroke * stPmm;
  if(debugPrint) Serial.printf("strokePosLimit: %ld\n", strokePosLimit);

  speed = (maxSpeed * speedPct) / 100;
  if(debugPrint) Serial.printf("Speed = %d\n", speed);
  motor.setMaxSpeed(speed * stPmm);
  motor.setAcceleration(speed * stPmm / 2);

  float Ac = (diameter/10);
  Ac = Ac*Ac/4*pi;  // in cm

  float syringeVol = stroke/10 * Ac;
  if(debugPrint) Serial.printf("Syringe volume: %.3f\n", syringeVol);
  dispenseCycles = round((dispenseVol/syringeVol) + 0.5);
  dispenseCycleVol = dispenseVol / dispenseCycles;
  if(debugPrint) Serial.printf("Dispense cycles: %d\n", dispenseCycles);

  dispenseStroke = dispenseCycleVol / Ac * 10 * stPmm;  // cm3/cm2 * 10 => mm
  if (dispenseStroke > strokePosLimit){
    Serial.println("Dosing volume exeeds stroke limit");
    dispenseStroke = strokePosLimit;
  }
  if(debugPrint) Serial.printf("dispenseStroke = %ld\n", dispenseStroke);
}

void homeButtonPopCallBack(void *prt_){
  // update syringe diameter, stroke lenght and speed by reading the value's from nextion hmi
  delay(100); // hack to try and read stroke text, perhaps nextion is slow to copy & convert text from page1 to page0?
  excludeState(currentState, osSettings);
  uint32_t tmpStroke = getStroke();
  uint32_t tmpSpeed = getSpeed();
  float tmpDiameter = getDiameter();
  float tmpVol = getVolume();

  if(configStorage.begin(configNamespace, false)){  // RW mode
    if(debugPrint) Serial.println("Saving config");

    if(tmpStroke != stroke) {
      stroke = tmpStroke;
      if(configStorage.putUShort(configNameStroke, stroke) == 0){
        Serial.println("Error saving stroke");
      }
    }

    if(tmpSpeed != speedPct) {
      speedPct = tmpSpeed;
      if(configStorage.putUShort(configNameSpeed, speedPct) == 0){
        Serial.println("Error saving speed");
      }
    }

    if(tmpVol != dispenseVol) {
      dispenseVol = tmpVol;
      if(configStorage.putFloat(configNameVolume, dispenseVol) == 0){
        Serial.println("Error saving volume");
      }
    }

    if(tmpDiameter != diameter) {
      if(configStorage.putFloat(configNameDiameter, diameter) == 0){
        Serial.println("Error saving diameter");
      }
    }
    configStorage.end();
  }
  else{
    Serial.println("Error opening config for saving");
  }
  updateDosingParams();
}

void dispense(){
  if(containState(currentState, osTripped) || containState(currentState, osSettings)) return;  // do nothing if tripped
  if(!containState(currentState, osPrimed)) {
    Serial.println("Cannot dispense until PRIMED");
    return;
  }
  // start dispense cycle from bottom position
  if (motor.currentPosition() != strokePosLimit) {
    // valve should be open to inlet
    switchValve(vpInlet);
    delay(250);
    safeMoveTo(strokePosLimit);
  }
  if(containState(currentState, osTripped)) return;  // do nothing if tripped

  totalDispensedVol = 0;
  for(byte i=0; i<dispenseCycles; i++){
    switchValve(vpOutlet);
    displayDispenseVolume = true;
    delay(250);       // give time for servo to move
    safeMoveTo(motor.currentPosition() - dispenseStroke);
    if(containState(currentState, osTripped)) return;  // do nothing if tripped

    displayDispenseVolume = false;
    switchValve(vpInlet);
    delay(250);       // give time for servo to move
    safeMoveTo(strokePosLimit);
    if(containState(currentState, osTripped)) return;  // do nothing if tripped
  }
  volumeText.setText("Ready");
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
  configStorage.end();
  updateDosingParams();

  // Update display
  strokeNumber.setValue(stroke);
  speedNumber.setValue(speedPct);
  sprintf(buffer, "%.3f", dispenseVol);
  volumeSettingText.setText(buffer);
  sprintf(buffer, "%.3f", diameter);
  diameterText.setText(buffer);
}

void setup(){
  nexInit(115200); //start comunication
  Serial.begin(115200);//250000);
  delay(5);
  sendCommand("tsw 255,0"); // disable touch events of all components on screen

  if(debugPrint) Serial.printf("Setup executing on core # %d\n", xPortGetCoreID());
//  sendCommand("baud=4800");
//  delay(200);
//  nexSerial.begin(4800);

  initStepperRunner();
  if(debugPrint) Serial.println("Loading config");
  loadConfig();

  pinMode(Bot, INPUT_PULLUP); //Limit Switch Connected to 0V
  pinMode(Top, INPUT_PULLUP); //Limit Switch Connected to 0V
  pinMode(doseButton, INPUT_PULLUP); //Switch Connected to 0V

  if(debugPrint){
    Serial.printf("Bottom end: %d\n", digitalRead(Bot));
    Serial.printf("Top end: %d\n", digitalRead(Top));
  }

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
  switchValve(vpInlet);
  delay(250);

  //register the pop events
  primeButton.attachPop(primeButtonPopCallBack, &primeButton);
  homeButton.attachPop(homeButtonPopCallBack, &homeButton);
  switchValveButton.attachPop(switchValveButtonPopCallBack, &switchValveButton);
  emptyButton.attachPop(emptyButtonPopCallBack, &emptyButton);
  upButton.attachPop(upButtonPopCallBack, &upButton);
  downButton.attachPop(downButtonPopCallBack, &downButton);
  settingsButton.attachPop(settingsButtonPopCallBack, &settingsButton);

  errMsg0.setText("Please Wait   Setting Up"); //Top Line note spacing
  errMsg1.setText("Please Wait"); //P1 top line
  statusText.setText("Not Ready"); //Status
  volumeText.setText("-----"); // Volume

  if(debugPrint) Serial.println("Stepper is Homing . . . . . . . . . . . ");

  motor.setMinPulseWidth(4);
  motor.setMaxSpeed(maxSpeed*stPmm/4);      // Set Max Speed of Stepper (Slower to get better accuracy)
  motor.setAcceleration(maxSpeed*stPmm/8);  // Set Acceleration of Stepper
  safeMoveTo(-100 * stPmm);
  if(containState(currentState, osTripped)) return;  // do nothing if tripped

  motor.setCurrentPosition(0);  // Set the current position as zero
  progressBar0.setValue(100); //show slider value as zero
  progressBar1.setValue(100); //Progress bar empty P1
  Serial.println("Homing Completed");

  //reset motor settings after position update
  motor.setMaxSpeed(speed * stPmm);
  motor.setAcceleration(speed * stPmm / 2);

  Serial.println("Setup done");
  errMsg0.setText("Please prime syringe"); //Top Line note spacing
  errMsg1.setText("Please prime"); //P1 top line
  statusText.setText("Not Ready"); //Status
  volumeText.setText("-----");// enable touch events of all components on screen

  includeState(currentState, osZeroed);
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
  if(!containState(currentState, osTripped)) {;  // do nothing if tripped
    nexLoop(nex_listen_list);

    // First read button state
    dosingButtonState = digitalRead(doseButton);

    // Then check serial for commands
    if(Serial.available()){
      char c = Serial.read();
      if(c == '?' || c == 'h'){
        Serial.println("0 : valve to inlet");
        Serial.println("1 : valve to outlet");
        Serial.println("p : prime");
        Serial.println("+ : move 5 mm up");
        Serial.println("- : move 5 mm down");
        Serial.println("e : empty");
        Serial.println("d : dispense");
        Serial.println("b : disable debug printing");
        Serial.println("B : enable debug printing");
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
      else if(c == '+'){
        upButtonPopCallBack(0);
      }
      else if(c == '-'){
        downButtonPopCallBack(0);
      }
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
    }

    // Check dosing button only if it was previously not pressed
    if((dosingButtonState == 0) && (prevDosingButtonState != 0)){
      prevDosingButtonState = 0;
      Serial.println("Dispensing...");
      dispense();
    }
    else if((dosingButtonState == 1) && (prevDosingButtonState == 0)) {
      prevDosingButtonState = 1;
      if(debugPrint) Serial.println("Dispense button released");
    }
  }
}
