#include "udpMgr.h"
#include "Motors.h"
#include "IMU_Control.h"
#include "Led_c.h"
#include "BattMgr.h"




void WiFiMgr_setup();



hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t lastIsrAt = 0;



void ARDUINO_ISR_ATTR onTimer() {
  
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  lastIsrAt = millis();
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void setup() {
  Serial.begin(115200);
  //Led_c_setup();
  //Motors_setup();
  WiFiMgr_setup();
  BattMgr_setup();
  IMU_Control_setup();
  start_timer();
  

  //Motors_test();

}

void start_timer()
{
 // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Set timer frequency to 1Mhz
  timer = timerBegin(1000000);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  //timerAlarm(timer, 1000000, true, 0);
  timerAlarm(timer, 2500, true, 0); // 2.5 ms -- 400 Hz
  //timerAlarm(timer, 4000, true, 0); // 4 ms -- 250 Hz
}

void loop() {
  //Motors_set_duty(1);
  loop_400Hz();
  //Led_c_loop();
  
}



void loop_400Hz()
{
// If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
   //Serial.println(" timer fire ...");
   //char buff[100];
   //sprintf(buff,"msg from EspDrone t=%d", lastIsrAt);
   //udpMgr_debug_send(buff);
   IMU_Control_loop();
  }
}
