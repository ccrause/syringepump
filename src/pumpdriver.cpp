#include "pumpdriver.h"
#include "AccelStepper.h"
#include "pinconfig.h"
#include <esp_task_wdt.h>

#include <TMCStepper.h>
#include <SPI.h>
#include "syringelist.h"
#include "coding.h"

myAccelStepper motor(myAccelStepper::DRIVER, stepPin, dirPin);
TMC2130Stepper driver(TMC_CS);

//int32_t speed;    // Actual speed in mm/sec

// Approximate pulse width in stepper pulses of the encoder
#define maxPulseCount 220 // 800*8 / (20*2) * 1.1
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;

boolean myAccelStepper::runSpeed(){
  if (AccelStepper::runSpeed()) {
    portENTER_CRITICAL_ISR(&counterMux);
    uint16_t temp = stepper_count++;
    portEXIT_CRITICAL_ISR(&counterMux);
    if (temp > maxPulseCount){
      trip = true;
      Serial.println("#");
      // Send serial data before exiting interrupt
      delayMicroseconds(500);
    }
    return true;
  }
  return false;
}

void myAccelStepper::reset(){
  digitalWrite(motorEnablePin, HIGH);
  digitalWrite(TMC_VIO, LOW);
}

float myAccelStepper::getAcceleration(){
  return _acceleration;
}

void myAccelStepper::lowSpeedSettings(){
  AccelStepper::setMaxSpeed(_maxSpeed * stPmm/5);      // Set Max Speed of Stepper (Slower to get better accuracy)
  AccelStepper::setAcceleration(accelRamp);//maxSpeed*stPmm/10);  // Set Acceleration of Stepper
  driver.sgt(this->lSG);
  driver.rms_current(syringeInfo[syringe].lowSpeedCurrent);
}

void myAccelStepper::highSpeedSettings(){
  AccelStepper::setMaxSpeed(speed_mm_s * stPmm);      // Set Max Speed of Stepper (Slower to get better accuracy)
  AccelStepper::setAcceleration(accelRamp);//maxSpeed*stPmm/10);  // Set Acceleration of Stepper
  driver.sgt(this->hSG);
  driver.rms_current(syringeInfo[syringe].highSpeedCurrent);
}

void IRAM_ATTR encoderPulse() {
  portENTER_CRITICAL_ISR(&counterMux);
  motor.stepper_count = 0;
  portEXIT_CRITICAL_ISR(&counterMux);
}

extern bool debugPrint;

void IRAM_ATTR stallHandler() {
  motor.trip = true;
  //if (debugTMC){
    Serial.println("STALL");
    // Send serial data before exiting interrupt
    delayMicroseconds(1000);
  //}
}

// runMotor task is thin for high frequency stepping
// User output should be done in a separate task
void runMotor(void *P){
  // Create motor stall clear interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stallPin), stallHandler, RISING);

  // Unsubscribe from TWDT so that long moves doesn't time-out
  esp_task_wdt_delete(NULL);
  motor.trip = false;
  while(true){
    if(motor.running && (motor.trip == false)){
      if(motor.moveToPosition){
        while (motor.run() && (motor.trip == false)) {}
        motor.running = false;
      }
      else {
        motor.runSpeed();
      }
    }
    else
      vTaskDelay(100 / portTICK_PERIOD_MS);  // delay & yield execution for 50 ms - purpose is to not waste time in this task while motor is not running
  }
}

void initStepperRunner(){
  pinMode(motorEnablePin, OUTPUT);
  pinMode(dirPin, OUTPUT); //stepper driver
  pinMode(stepPin, OUTPUT ); //stepper driver
  pinMode(TMC_VIO, OUTPUT);
  pinMode(stallPin, INPUT_PULLDOWN);
  pinMode(encoderPin, INPUT);
  digitalWrite(motorEnablePin, LOW);
  digitalWrite(TMC_VIO, HIGH);

  motor.setPinsInverted (true, false, false);
  motor.setMinPulseWidth(4);
  motor.lSG = syringeInfo[syringe].lowSpeedSGT;
  motor.hSG = syringeInfo[syringe].highSpeedSGT;

  SPI.begin(TMC_SCK, TMC_SDI, TMC_SDO, TMC_CS);
  driver.begin();             // Initiate pins and registeries
  driver.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver.pwm_autoscale(1);
  driver.pwm_grad(8);
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // 400mA
  driver.microsteps(16);
  driver.intpol(true);
  driver.TCOOLTHRS(2000); // 20bit max, disable stallGuard at low velocity
  driver.THIGH(0);

  // Current control, 5.5.3, p. 36
  driver.sfilt(true);           // Filter SG readout
  driver.sgt(motor.lSG);        // Stall sensitivity: more positive -> less sensitive
  driver.seimin(0);             // 0=1/4, 1=1/2 of current setting
  driver.sedn(1);               // Current down step speed (0 = 32 SG values before down)
  driver.semax(1); //4             // Decrease current if SG value above (semin+semax)*32
  driver.seup(3);               // Current increment steps
  driver.semin(10);             // Increase current if SG value below semin*32

  driver.diag1_stall(true);
  driver.diag1_pushpull(true);

  xTaskCreatePinnedToCore(
    &runMotor,            // function
    "runMotor",           // name
    1000,                 // stack
    NULL,                 // void* to input parameter
    10,                   // priority
    NULL,                 // task handle
    0                     // core #0, core #1 is used to run setup & loop, i.e. the rest of the user interaction code
  );
  // Also delete idle task on CPU 0 from task watchdog timer to prevent time-outs when motor is executing a long run
  TaskHandle_t idleHandle = xTaskGetIdleTaskHandleForCPU(0);
  esp_err_t r = esp_task_wdt_delete(idleHandle);
  if(!(r == ESP_OK)){
    Serial.println("Error removing CPU0 idle task from TWDT");
  }
}
