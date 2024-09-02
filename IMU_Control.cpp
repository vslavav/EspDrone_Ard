#include "IMU_Control.h"
#include "AHRS_Mgr.h"
#include <Wire.h>
#include "Arduino.h"
#include "MsgDbg.h"

typedef enum
{
  MPU6050_SCALE_2000DPS = 0b11,
  MPU6050_SCALE_1000DPS = 0b10,
  MPU6050_SCALE_500DPS = 0b01,
  MPU6050_SCALE_250DPS = 0b00
} mpu6050_dps_t;

typedef enum
{
  MPU6050_RANGE_16G = 0b11,
  MPU6050_RANGE_8G = 0b10,
  MPU6050_RANGE_4G = 0b01,
  MPU6050_RANGE_2G = 0b00,
} mpu6050_range_t;


uint8_t IMU_Control_readByte(uint8_t subAddress);
void IMU_Control_readBytes(uint8_t subAddress, uint8_t count, uint8_t *dest);
void IMU_Control_writeByte(uint8_t reg, uint8_t value);

void IMU_Control_read_gyro_rawData(int16_t *destination);
void IMU_Control_read_accel_rawData(int16_t *destination);
void IMU_Control_get_gyro_data();
void IMU_Control_get_accel_data();

void IMU_Control_set_gScale(mpu6050_dps_t scale);
mpu6050_dps_t IMU_Control_get_gScale(void);
void IMU_Control_set_aScale(mpu6050_range_t range);
mpu6050_range_t IMU_Control_get_aScale(void);

void IMU_Control_calibrate();

const int sda = 11;
const int scl = 10;

const int MPU_address = 0x68;              // MPU6050 I2C address
const int MPU6050_REG_GYRO_CONFIG = 0x1B;  // Gyroscope Configuration
const int MPU6050_REG_ACCEL_CONFIG = 0x1C; // Accelerometer Configuration

float gx_current = 0.0; // gyro x value current
float gy_current = 0.0;
float gz_current = 0.0;

float gx_bias = 0.0; // gyro x bias value, after calibration
float gy_bias = 0.0;
float gz_bias = 0.0;


float gScale = 0; // 250.0 / 32768.0; // this is gyro scale


float ax_current = 0.0; // accel x value current
float ay_current = 0.0;
float az_current = 0.0;

float ax_bias = 0.0; // accel x bias value, after calibration
float ay_bias = 0.0;
float az_bias = 0.0;

float aScale = 0;       // this is accel scale

float roll_current = 0.0;
float pitch_current = 0.0;
float yaw_current = 0.0;

void IMU_Control_setup()
{
  // Try to initialize!
  Wire.begin(sda, scl);
  Wire.beginTransmission(MPU_address); // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                    // Talk to the register 6B
  Wire.write(0x00);                    // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);          // end the transmission
  // Call this function if you need to get the IMU error values for your module
  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t WHO_AM_I_MPU6050 = 0x75;                    // Should return 0x68
  uint8_t WHO_AM_I_MPU6050_RESPONCE = IMU_Control_readByte(WHO_AM_I_MPU6050); // Read WHO_AM_I register for MPU-6050
  //Serial.printf("WHO_AM_I = 0x%X \n", WHO_AM_I_MPU6050_RESPONCE);
  char buff[100]; sprintf(buff,"WHO_AM_I = 0x%X \n", WHO_AM_I_MPU6050_RESPONCE);
  MsgDbg_printMsg(buff);
  if(WHO_AM_I_MPU6050_RESPONCE != 0x68)
  {
    while(1)
    {
      MsgDbg_printMsg("mpu 6050 fail ... \n");
      delay(1000);
    }
  }
  uint8_t GYRO_CONFIG = 0x1B;
  uint8_t c = IMU_Control_readByte(GYRO_CONFIG);
  Serial.printf("GYRO_CONFIG = 0x%X \n", c);
  IMU_Control_set_gScale(MPU6050_SCALE_2000DPS);
  IMU_Control_set_aScale(MPU6050_RANGE_16G);
  
  
  delay(20);

  IMU_Control_get_gScale();
  IMU_Control_get_aScale();

  IMU_Control_calibrate();
}

void IMU_Control_calibrate()
{
  int nNumCycle = 500;
  int nTempVal = nNumCycle;

  gx_bias = 0.0; // gyro x bias value, after calibration
  gy_bias = 0.0;
  gz_bias = 0.0;

  MsgDbg_printMsg("start calibrating ..........\n");
  MsgDbg_printMsg("start gyro calibrating ..........\n");

  while (nTempVal > 0)
  {
    int16_t gyroRawData[3];                     // Stores the 16-bit signed gyro sensor output
    IMU_Control_read_gyro_rawData(gyroRawData); // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx_bias += (float)gyroRawData[0] * gScale; // get actual gyro value, this depends on scale being set
    gy_bias += (float)gyroRawData[1] * gScale;
    gz_bias += (float)gyroRawData[2] * gScale;
    nTempVal--;
    delay(50);
  }

  gx_bias = gx_bias / nNumCycle;
  gy_bias = gy_bias / nNumCycle;
  gz_bias = gz_bias / nNumCycle;

  char buff[100];
  sprintf(buff, "gx_bias=%f \n",gx_bias); MsgDbg_printMsg(buff);
  sprintf(buff, "gy_bias=%f \n",gy_bias); MsgDbg_printMsg(buff);
  sprintf(buff, "gz_bias=%f \n",gz_bias); MsgDbg_printMsg(buff);
 
  

  ax_bias = 0.0; // accel x bias value, after calibration
  ay_bias = 0.0;
  az_bias = 0.0;

  nTempVal = 0; //nNumCycle;

  MsgDbg_printMsg("start accel calibrating ..........\n");

  while (nTempVal > 0)
  {
    int16_t accelCount[3];                      // Stores the 16-bit signed accelerometer sensor output
    IMU_Control_read_accel_rawData(accelCount); // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    ax_bias += (float)accelCount[0] * aScale; // get actual g value, this depends on scale being set
    ay_bias += (float)accelCount[1] * aScale;
    az_bias += (float)accelCount[2] * aScale;
    nTempVal--;
    delay(50);
  }

  ax_bias = ax_bias / nNumCycle;
  ay_bias = ay_bias / nNumCycle;
  az_bias = az_bias / nNumCycle;

  sprintf(buff, "ax_bias=%f \n",ax_bias); MsgDbg_printMsg(buff);
  sprintf(buff, "ay_bias=%f \n",ay_bias); MsgDbg_printMsg(buff);
  sprintf(buff, "az_bias=%f \n",az_bias); MsgDbg_printMsg(buff);

  MsgDbg_printMsg("end calibrating ..........\n");

  
}

void IMU_Control_get_gyro_data()
{
  int16_t gyroRawData[3];                     // Stores the 16-bit signed gyro sensor output
  IMU_Control_read_gyro_rawData(gyroRawData); // Read the x/y/z adc values
  gx_current = gyroRawData[0] * gScale - gx_bias;      // gyro x bias value, after calibration
  gy_current = gyroRawData[1] * gScale - gy_bias;
  gz_current = gyroRawData[2] * gScale - gz_bias;
}

void IMU_Control_get_accel_data()
{
  int16_t accelRawData[3];   
  IMU_Control_read_accel_rawData(accelRawData);
  ax_current = accelRawData[0] * aScale - ax_bias;
  ay_current = accelRawData[1] * aScale - ay_bias;
  az_current = accelRawData[2] * aScale - az_bias;
  
}

void IMU_Control_read_gyro_rawData(int16_t *destination)
{
  uint8_t rawData[6];
  int8_t GYRO_XOUT_H = 0x43;                                  // x/y/z gyro register data stored here
  IMU_Control_readBytes(GYRO_XOUT_H, 6, &rawData[0]);         // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

void IMU_Control_read_accel_rawData(int16_t *destination)
{
  uint8_t rawData[6]; // x/y/z accel register data stored here
  int8_t ACCEL_XOUT_H = 0x3B;
  IMU_Control_readBytes(ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

void IMU_Control_loop()
{
  float dt = (float) (1.0 / 400);
  //float dt = (float) (1.0 / 250);
  static int16_t count = 0;


  IMU_Control_get_gyro_data();
  IMU_Control_get_accel_data();
  AHRS_Mgr_update_top(gx_current, gy_current, gz_current, ax_current, ay_current, az_current, dt);

  AHRS_Mgr_GetEulerRPY(&roll_current, &pitch_current, &yaw_current);

  if(count % 250 == 0)
  {
    char buff[100];
    sprintf(buff,"roll_current=%f \n", roll_current);
    MsgDbg_printMsg(buff);
    sprintf(buff,"pitch_current=%f \n", pitch_current);
    MsgDbg_printMsg(buff);
    sprintf(buff,"yaw_current=%f \n", yaw_current);
    MsgDbg_printMsg(buff);
   

    MsgDbg_printMsg("-----------------\n");
  }
  count++;

  
}

void IMU_Control_set_gScale(mpu6050_dps_t scale)
{
  uint8_t value;

  // value = readRegister8(MPU6050_REG_GYRO_CONFIG);
  value = IMU_Control_readByte(MPU6050_REG_GYRO_CONFIG);
  value &= 0b11100111;
  value |= (scale << 3);
  IMU_Control_writeByte(MPU6050_REG_GYRO_CONFIG, value);
}

mpu6050_dps_t IMU_Control_get_gScale(void)
{
  uint8_t value;
  
  value = IMU_Control_readByte(MPU6050_REG_GYRO_CONFIG);
  value &= 0b00011000;
  value >>= 3;
  switch ((mpu6050_dps_t)value)
  {
  case MPU6050_SCALE_250DPS:
    gScale = .007629f; // 250 / 32768
    break;
  case MPU6050_SCALE_500DPS:
    gScale = .015258f; // 500 / 32768
    break;
  case MPU6050_SCALE_1000DPS:
    gScale = .03052f; // 1000/ 32768
    break;
  case MPU6050_SCALE_2000DPS:
    gScale = .060103f; //  2000 / 32768
    Serial.printf("MPU6050_SCALE_2000DPS \n");
    break;
  default:
    break;
  }

  return (mpu6050_dps_t)value;
}

void IMU_Control_set_aScale(mpu6050_range_t range)
{
  uint8_t value;

  value = IMU_Control_readByte(MPU6050_REG_ACCEL_CONFIG);
  
  value &= 0b11100111;
  value |= (range << 3);
  IMU_Control_writeByte(MPU6050_REG_ACCEL_CONFIG, value);
}

mpu6050_range_t IMU_Control_get_aScale(void)
{
  uint8_t value;
  value = IMU_Control_readByte(MPU6050_REG_ACCEL_CONFIG);
  (MPU6050_REG_ACCEL_CONFIG);
  value &= 0b00011000;
  value >>= 3;

  switch ((mpu6050_range_t)value)
  {
  case MPU6050_RANGE_2G:
    aScale = .000061f; // 2 /32768
    Serial.printf("MPU6050_RANGE_2G \n");
    break;
  case MPU6050_RANGE_4G:
    aScale = .000122f; // 4 / 32768
    Serial.printf("MPU6050_RANGE_4G \n");
    break;
  case MPU6050_RANGE_8G:
    aScale = .000244f; // 8 / 32768
    Serial.printf("MPU6050_RANGE_8G \n");
    break;
  case MPU6050_RANGE_16G:
    aScale = .0004882f; // 16/32768
    Serial.printf("MPU6050_RANGE_16G \n");
    break;
  default:
    break;
  }
  return (mpu6050_range_t)value;
}

uint8_t IMU_Control_readByte(uint8_t subAddress)
{
  uint8_t data;                              // `data` will store the register data
  Wire.beginTransmission(MPU_address);       // Initialize the Tx buffer
  Wire.write(subAddress);                    // Put slave register address in Tx buffer
  Wire.endTransmission(false);               // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(MPU_address, (uint8_t)1); // Read one byte from slave register address
  data = Wire.read();                        // Fill Rx buffer with result
  Wire.endTransmission();
  return data; // Return data read from slave register
}

void IMU_Control_readBytes(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  Wire.beginTransmission(MPU_address); // Initialize the Tx buffer
  Wire.write(subAddress);              // Put slave register address in Tx buffer
  Wire.endTransmission(false);         // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(MPU_address, count); // Read bytes from slave register address
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  } // Put read results in the Rx buffer
  Wire.endTransmission();
}

// Write 8-bit to register
void IMU_Control_writeByte(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_address);

  Wire.write(reg);
  Wire.write(value);

  Wire.endTransmission();
}




