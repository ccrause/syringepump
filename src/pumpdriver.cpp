#include "pumpdriver.h"
#include "AccelStepper.h"
#include "pinconfig.h"
#include <esp_task_wdt.h>

#include <TMCStepper.h>
#include <SPI.h>

myAccelStepper motor(myAccelStepper::DRIVER, stepPin, dirPin);
TMC2130Stepper driver(TMC_CS);

// Approximate pulse width in stepper pulses of the encoder
#define maxPulseCount 220 // 800*8 / (20*2) * 1.1
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;

void myAccelStepper::setCurrent(uint16_t mA){
  driver.rms_current(mA);
}

boolean myAccelStepper::runSpeed(){
  if (AccelStepper::runSpeed()) {
    portENTER_CRITICAL_ISR(&counterMux);
    uint16_t temp = stepper_count++;
    portEXIT_CRITICAL_ISR(&counterMux);
    if (temp > maxPulseCount){
      trip = true;
    }
    return true;
  }
  return false;
}

void myAccelStepper::reset(){
  digitalWrite(motorEnablePin, HIGH);
  digitalWrite(TMC_VIO, LOW);
}

void IRAM_ATTR encoderPulse() {
  portENTER_CRITICAL_ISR(&counterMux);
  motor.stepper_count = 0;
  portEXIT_CRITICAL_ISR(&counterMux);
}

extern bool debugPrint;

void IRAM_ATTR stallHandler() {
  motor.trip = true;
  if (debugPrint){
    Serial.println("STALL");
    // Send serial data before exiting interrupt
    delayMicroseconds(1000);
  }
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
  pinMode(stallPin, INPUT);
  pinMode(encoderPin, INPUT);
  digitalWrite(motorEnablePin, LOW);
  digitalWrite(TMC_VIO, HIGH);

  motor.setPinsInverted (false, false, false);

  SPI.begin(TMC_SCK, TMC_SDI, TMC_SDO, TMC_CS);
  driver.begin();             // Initiate pins and registeries
  driver.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver.pwm_autoscale(1);
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // 400mA
  driver.microsteps(16);
  driver.sfilt(true); // Improves TMC2660 SG readout
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(4);  // Stall sensitivity: more positive -> less sensitive
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
