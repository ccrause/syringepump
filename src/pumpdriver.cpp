#include "pumpdriver.h"
#include "AccelStepper.h"
#include "pinconfig.h"
#include <esp_task_wdt.h>

myAccelStepper motor(1, stepPin, dirPin);

// Approximate pulse width in stepper pulses of the encoder
#define maxPulseCount 96 // 800*8 / (20*2) * 1.1
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;

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

void IRAM_ATTR encoderPulse() {
  portENTER_CRITICAL_ISR(&counterMux);
  motor.stepper_count = 0;
  portEXIT_CRITICAL_ISR(&counterMux);
}

void IRAM_ATTR faultHandler() {
  motor.trip = true;
  // if (debugPrint)
  Serial.println("FAULT");
  // Send serial data before exiting interrupt
  delayMicroseconds(1000);
}

// runMotor task is thin for high frequency stepping
// User output should be done in a separate task
void runMotor(void *P){
  // Create motor stall clear interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(faultPin), faultHandler, CHANGE);

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
  motor.setPinsInverted (false, false, false);
  pinMode(ms1, OUTPUT); //micro step
  pinMode(ms2, OUTPUT); //micro step
  pinMode(ms3, OUTPUT); //micro step
  pinMode(dirPin, OUTPUT); //stepper driver
  pinMode(stepPin, OUTPUT ); //stepper driver
  // DRV8825 1/16 microstep
  digitalWrite (ms1, LOW);
  digitalWrite (ms2, LOW);
  digitalWrite (ms3, HIGH);
  // Fault detection & handling pins
  pinMode(resetPin, INPUT); // stepper driver reset, external pullup, pull down to reset
  pinMode(faultPin, INPUT); // stepper driver fault, external pullup, low indicate fault
  pinMode(encoderPin, INPUT);

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
