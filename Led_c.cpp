#include <Arduino.h>
#include "Led_c.h"

void Led_c_task(void *pvParameters);

const int pin_red = 8;
const int pin_blue = 7;
const int pin_green = 9; 

int nRed_flashing_period = 0;
unsigned long last_time_flash_red = 0;
int last_val_red = 0;

int nBlew_flashing_period = 0;
unsigned long last_time_flash_blew = 0;
int last_val_blew = 0;

int nGreen_flashing_period = 0;
unsigned long last_time_flash_green = 0;
int last_val_green = 0;

void Led_c_setup() {
  pinMode(pin_red, OUTPUT);
  pinMode(pin_blue, OUTPUT);
  pinMode(pin_green, OUTPUT);

  //Led_c_set_red(HIGH);
  //Led_c_set_blue(HIGH);
  //Led_c_set_green(HIGH);
  Led_c_set_red_flashing_period(500);
  Led_c_set_blue_flashing_period(300);
  Led_c_set_green_flashing_period(1000);

  xTaskCreate(
    Led_c_task, "Led_c_task",  // A name just for humans
    2048,  // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    NULL,  // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    2,     // Priority
    NULL  // Task handle is not used here - simply pass NULL
  );

}

void Led_c_task(void *pvParameters) {  // This is a task.
  (void)pvParameters;
  while(1)
  {
    Led_c_loop();
    delay(100);
  }

}



void Led_c_set_red_flashing_period(int nPeriod)
{
  nRed_flashing_period = nPeriod;
}

void Led_c_set_red_flashing()
{
  if(nRed_flashing_period == 0)
  {
    return;
  } 

  if(millis() > (last_time_flash_red + nRed_flashing_period))
  {
    last_time_flash_red = millis();
    if(last_val_red == 0)
    {
      last_val_red = 1;
      
    }
    else
    {
      last_val_red = 0;
    }

    Led_c_set_red(last_val_red);
  }
}

void Led_c_set_blue_flashing_period(int nPeriod)
{
  nBlew_flashing_period = nPeriod;
}

void Led_c_set_blue_flashing()
{
  if(nBlew_flashing_period == 0)
  {
    return;
  } 

  if(millis() > (last_time_flash_blew + nBlew_flashing_period))
  {
    last_time_flash_blew = millis();
    if(last_val_blew == 0)
    {
      last_val_blew = 1;
      
    }
    else
    {
      last_val_blew = 0;
    }

    Led_c_set_blue(last_val_blew);
  }
}

void Led_c_set_green_flashing_period(int nPeriod)
{
  nGreen_flashing_period = nPeriod;
}

void Led_c_set_green_flashing()
{
  if(nGreen_flashing_period == 0)
  {
    return;
  } 

  if(millis() > (last_time_flash_green + nGreen_flashing_period))
  {
    last_time_flash_green = millis();
    if(last_val_green == 0)
    {
      last_val_green = 1;
      
    }
    else
    {
      last_val_green = 0;
    }

    Led_c_set_green(last_val_green);
  }
}



void Led_c_set_red(int val)
{
  if(val == 0)
  {
    digitalWrite(pin_red, LOW);
  }
  else
  {
    digitalWrite(pin_red, HIGH);
  }
}
void Led_c_set_blue(int val)
{
  if(val == 0)
  {
    digitalWrite(pin_blue, LOW);
  }
  else
  {
    digitalWrite(pin_blue, HIGH);
  }
}

void Led_c_set_green(int val)
{
  if(val == 0)
  {
    digitalWrite(pin_green, LOW);
  }
  else
  {
    digitalWrite(pin_green, HIGH);
  }
}

void Led_c_loop() {
  Led_c_set_red_flashing();
  Led_c_set_blue_flashing();
  Led_c_set_green_flashing();

}