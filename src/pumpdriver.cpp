#include "pumpdriver.h"
#include "AccelStepper.h"
#include "pinconfig.h"
#include <esp_task_wdt.h>

// motorIsRunning should block other tasks from accessing motor related parameters
// except for reading motor.currentPosition & motor.distanceToGo
volatile bool motorIsRunning = false;
volatile bool moveToPosition = true;

// Approximate pulse width in stepper pulses of the encoder
#define maxPulseCount 220 // 800*8 / (20*2) * 1.1
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t stepper_count = 0;
volatile bool trip = false;

void IRAM_ATTR stepperPulse() {
  portENTER_CRITICAL_ISR(&counterMux);
  stepper_count++;
  portEXIT_CRITICAL_ISR(&counterMux);
  if(stepper_count > maxPulseCount){
    trip = true;
    Serial.printf("Stepper pulse count: %d\n", stepper_count);
  }
}

void IRAM_ATTR encoderPulse() {
  portENTER_CRITICAL_ISR(&counterMux);
  stepper_count = 0;
  portEXIT_CRITICAL_ISR(&counterMux);
}

// runMotor task is thin but respect end stops
// User output should be done in a separate task
void runMotor(void *P){
  // Create motor stall cross interrupts
  pinMode(stepInputPin, INPUT);
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(stepInputPin), stepperPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderPulse, CHANGE);

  // Unsubscribe from TWDT so that long moves doesn't time-out
  esp_task_wdt_delete(NULL);
  while(trip == false){
    if(motorIsRunning){
      if(moveToPosition){
        if(motor.distanceToGo() > 0){ // Move down
          while (digitalRead(Bot) && motor.run() && (trip == false)) {}
        }
        else{
          while (digitalRead(Top) && motor.run() && (trip == false)) {}
        }
        motorIsRunning = false;
      }
      else {
        if(motor.speed() > 0){ // Move down
          if(digitalRead(Bot) && (trip == false)) {
            motor.runSpeed();
          }
        }
        else{
          if (digitalRead(Top) && (trip == false)) {
            motor.runSpeed();
          }
        }
      }
    }
    else
      vTaskDelay(100 / portTICK_PERIOD_MS);  // delay & yield execution for 50 ms - purpose is to not waste time in this task while motor is not running
  }
  motorIsRunning = false;
  // Kiss task goodbye
  vTaskDelete(NULL);
}

void initStepperRunner(){
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
