#include "arduino.h"
#include "Motors.h"




//#define MOTOR01_PIN 5
//#define MOTOR02_PIN 6
//#define MOTOR03_PIN 3
//#define MOTOR04_PIN 4
#define MOTOR_RF 5
#define MOTOR_RR 6
#define MOTOR_LR 3
#define MOTOR_LF 4

// use 12 bit precision for LEDC timer
#define LEDC_TIMER_12_BIT 12

// Define the PWM properties
const int freq = 5000;  // Frequency in Hz

const int resolution = 8;  // 8-bit resolution


// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t pin, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 4095 from 2 ^ 12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(pin, duty);
}

void Motors_test()
{
  
  uint8_t val = 15;
  uint16_t dl_val = 1000;
  delay(dl_val);
  Motors_set_duty(MOTOR_RF,val);
  delay(dl_val);
  Motors_set_duty(MOTOR_RF,0);
  delay(dl_val);
  Motors_set_duty(MOTOR_RR,val);
  delay(dl_val);
  Motors_set_duty(MOTOR_RR,0);
  delay(dl_val);
  Motors_set_duty(MOTOR_LR,val);
  delay(dl_val);
  Motors_set_duty(MOTOR_LR,0);
  delay(dl_val);
  Motors_set_duty(MOTOR_LF,val);
  delay(dl_val);
  Motors_set_duty(MOTOR_LF,0);
  //return;
  delay(3000);
  val = 70;
  Motors_set_duty(MOTOR_RF,val);
  Motors_set_duty(MOTOR_LR,val);
  Motors_set_duty(MOTOR_RR,val);
  Motors_set_duty(MOTOR_LF,val);
  delay(1500);
  val = 100;
  Motors_set_duty(MOTOR_RF,val);
  Motors_set_duty(MOTOR_LR,val);
  Motors_set_duty(MOTOR_RR,val);
  Motors_set_duty(MOTOR_LF,val);
  delay(1500);
  Motors_set_duty(MOTOR_RF,0);
  Motors_set_duty(MOTOR_RR,0);
  Motors_set_duty(MOTOR_LR,0);
  Motors_set_duty(MOTOR_LF,0);
  

}

void Motors_setup() {
 // Setup timer and attach timer to a led pin
  ledcAttach(MOTOR_RF, freq, LEDC_TIMER_12_BIT);
  ledcAttach(MOTOR_RR, freq, LEDC_TIMER_12_BIT);
  ledcAttach(MOTOR_LR, freq, LEDC_TIMER_12_BIT);
  ledcAttach(MOTOR_LF, freq, LEDC_TIMER_12_BIT);
  
    

}

void Motors_loop() {
  // put your main code here, to run repeatedly:

}

void Motors_set_duty(uint8_t pin,uint32_t value)
{
  // vaue -> 0..255
  //ledcAnalogWrite(MOTOR01_PIN,  value);
  ledcAnalogWrite(pin,  value);
}


