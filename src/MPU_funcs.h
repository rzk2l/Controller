#ifndef _MPU_FUNCS_H
#define _MPU_FUNCS_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define CALIB_NB 1500

Adafruit_MPU6050 mpu;
sensors_event_t accelero, gyroscope, tempurature;


float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccXangleError, AccYangleError, GyroXError, GyroYError, GyroZError;
float PreviousYaw, PreviousGyroX, PreviousGyroY, PreviousGyroZ = 0; 
float AngleFromAccX, AngleFromAccY;

float currentTime, previousTime, elapsedTime; 
 


void imuSetup(){
  while (!Serial)
    delay(10); 

  Serial.println("MPU 6050");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
}

void resetAngles(float *pitch_angle, float *roll_angle, float *yaw_angle){
    mpu.getEvent(&accelero, &gyroscope, &tempurature);
    *pitch_angle = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
    *roll_angle = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
    *yaw_angle = GyroZ;
}

void imuCalibration(){
  Serial.println("Calibrating, KEEP IMU STATIONARY!");
  float prevYawError = 0;
  int iter = 0;
  AccXangleError = 0; 
  AccYangleError = 0;
  while (iter<CALIB_NB){
    mpu.getEvent(&accelero,&gyroscope,&tempurature);
    AccX = (accelero.acceleration.x);
    AccY = (accelero.acceleration.y);
    AccZ = (accelero.acceleration.z);
    AccXangleError += (atan(AccY/sqrt(pow(AccX,2)+pow(AccZ,2)))* 180/PI);
    AccYangleError += (atan(-1*AccX/sqrt(pow(AccY,2)+pow(AccZ,2)))* 180/PI);
    iter++;
  }
  AccXangleError = AccXangleError/CALIB_NB;
  AccYangleError = AccYangleError/CALIB_NB;

  iter = 0;
  GyroXError, GyroYError, GyroZError, prevYawError = 0;
   while (iter<CALIB_NB){
    mpu.getEvent(&accelero,&gyroscope,&tempurature);
    GyroX = gyroscope.gyro.x;
    GyroY = gyroscope.gyro.y;
    GyroZ = gyroscope.gyro.z;

    GyroZ = 0.2 * GyroZ + 0.8 * prevYawError;
    prevYawError = GyroZ;

    GyroXError += GyroX;
    GyroYError += GyroY;
    GyroZError += GyroZ;
    iter++;
  }
  GyroXError = GyroXError/CALIB_NB;
  GyroYError = GyroYError/CALIB_NB;
  GyroZError = GyroZError/CALIB_NB;

  Serial.println("IMU CALIBRATED!");
}

void findAngles(float *pitch_angle, float *roll_angle, float *yaw_angle){
  mpu.getEvent(&accelero,&gyroscope,&tempurature);
  //Gyro values 
  GyroX = gyroscope.gyro.x - (GyroXError);
  GyroY = gyroscope.gyro.y - (GyroYError);
  GyroZ = gyroscope.gyro.z - (GyroZError);

  // Accelerometer values 
  AccX = (accelero.acceleration.x);
  AccY = (accelero.acceleration.y);
  AccZ = (accelero.acceleration.z);

  AngleFromAccX = (atan(AccY/sqrt(pow(AccX,2)+pow(AccZ,2)))* 180/PI) - AccXangleError;
  AngleFromAccY = (atan(-1*AccX/sqrt(pow(AccY,2)+pow(AccZ,2)))* 180/PI) - AccYangleError;


  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime-previousTime)/1000; //convert to seconds
  

  *pitch_angle = 0.9996*(*pitch_angle+(GyroY*elapsedTime*180)/PI) + 0.0004*AngleFromAccY;
  *roll_angle = 0.9996*(*roll_angle+(GyroX*elapsedTime*180)/PI) + 0.0004*AngleFromAccX;
  *yaw_angle = 0.8*(*yaw_angle+(GyroZ*elapsedTime*180)/PI) + 0.2*PreviousYaw;

  PreviousYaw = *yaw_angle;

  /* Serial.print(*pitch_angle);
  Serial.print(" ////// ");
  Serial.print(*roll_angle);
  Serial.print(" /////// ");
  Serial.println(*yaw_angle); */
  
}



#endif