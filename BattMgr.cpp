#include "BattMgr.h"
#include "Arduino.h"
#include "MsgDbg.h"

const int batteryPin = 2; // select the input pin for the potentiometer

void BattMgr_setup()
{
    // set the resolution to 12 bits (0-4095)
    analogReadResolution(12);
    BattMgr_get_batt_value();
}

float BattMgr_get_batt_value()
{   
    char buff[100];
    // read the  millivolts value for pin 2:
   
    int analogVolts = 2 * analogReadMilliVolts(batteryPin);
    // print out the values you read:
    float batt_voltage = (float)analogVolts / 1000.0;
    sprintf(buff, "Batt  value = %.2f\n", batt_voltage);
    MsgDbg_printMsg(buff);

    return batt_voltage;
    
}