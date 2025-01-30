#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "gyro.h"


static Adafruit_MPU6050 mpu;

static float rotation_x_error = -0.05;
static float rotation_y_error = 0.02;
static float rotation_z_error = 0.03;
static float accel_x_error = 0.08;
static float accel_y_error = -0.22;
static float accel_z_error = 0.01;

float rot_x, rot_y, rot_z;


void lecture_gyro()
{
  float accel_x, accel_y, accel_z = 0.0;
  
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  accel_x = a.acceleration.x - accel_x_error;
  accel_y = a.acceleration.y - accel_y_error;
  accel_z = a.acceleration.z - accel_z_error;

  rot_x = atan((accel_z) / sqrt(pow((accel_y), 2) + pow((accel_x), 2))) * 180 / 3.14;
  rot_y = atan((accel_y) / sqrt(pow((accel_z), 2) + pow((accel_x), 2))) * 180 / 3.14;
}

bool gyro_moved () {
    static float rot_x_old, rot_y_old, rot_z_old;

    if ((fabs(rot_x_old - rot_x) > 3) || (fabs(rot_y_old - rot_y) > 3)) {
        rot_x_old = rot_x;
        rot_y_old = rot_y;
        return true;
    } else {
        return false;
    }
}

void motion () {
  Serial.println("moved");
}

void setup_gyro () {
  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

}

void writeRegister(uint16_t reg, byte value)
{
  Wire.beginTransmission(MPU6050_DEVICE_ID);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


void setInterrupt(byte threshold)
{
  writeRegister(MPU6050_INT_PIN_CONFIG, 1 << MPU6050_ACTL); 
  writeRegister(MPU6050_ACCEL_CONFIG, 0b00000001);
  writeRegister(MPU6050_MOT_THR, threshold);
  writeRegister(MPU6050_MOT_DUR, 0b00000001);    // set duration (LSB = 1 ms)
  writeRegister(MPU6050_INT_ENABLE, 1 << MPU6050_WOM_EN);
}

void setup_gyro_int() {

  setInterrupt(10); // set Wake on Motion Interrupt / Sensitivity; 1(highest sensitivity) - 255

//  esp_sleep_enable_gpio_wakeup();
//  gpio_wakeup_enable(GYRO_INT, GPIO_INTR_LOW_LEVEL);
  pinMode(3, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(3), motion, RISING); 
  
}
