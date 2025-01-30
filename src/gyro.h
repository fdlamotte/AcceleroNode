#ifndef _GYRO_H_
#define _GYRO_H_

#define GYRO_INT GPIO_NUM_3

#define MPU6050_WOM_EN 0x06            // Wake on Motion Enable bit
#define MPU6050_ACTL 0x07              // Active-Low Enable Bit
#define MPU6050_ACCEL_INTEL_CTRL 0x69  // Accelaration Interrupt Control Register

extern float rot_x, rot_y, rot_z;

void lecture_gyro();
bool gyro_moved ();
void setup_gyro ();
void setup_gyro_int ();

#endif