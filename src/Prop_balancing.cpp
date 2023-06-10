#include<Arduino.h>
#include<MPU_funcs.h>
#include<ESP32Servo.h>
#include<Comms_funcs_gui.h>

#define MIN_SPEED 1000
#define MAX_SPEED 2000  
#define MOTOR_PIN_FR 19
#define MOTOR_PIN_FL 18
#define MOTOR_PIN_BR 17 
#define MOTOR_PIN_BL 16

Servo ESCFR;
Servo ESCFL;
Servo ESCBR;
Servo ESCBL;

float pitch, roll, yaw;
long accTotalVec[20], accAvgVec;
int vibeCounter, vibeResult, motorTest;

void escInit(){
  ESCFR.attach(MOTOR_PIN_FR,MIN_SPEED,MAX_SPEED);
  ESCFL.attach(MOTOR_PIN_FL,MIN_SPEED,MAX_SPEED);
  ESCBR.attach(MOTOR_PIN_BR,MIN_SPEED,MAX_SPEED);
  ESCBL.attach(MOTOR_PIN_BL,MIN_SPEED,MAX_SPEED);

}

void motorControl(){
    ESCFR.writeMicroseconds(throttleCmd);
    ESCFL.writeMicroseconds(throttleCmd);
    ESCBR.writeMicroseconds(throttleCmd);
    ESCBL.writeMicroseconds(throttleCmd);
}

void setup(){
    Serial.begin(9600);
    connectToWifi();
    imuSetup();
    escInit();
}

void loop(){
    WiFiClient client = server.available();
    if (client) {
      handleClientRequest(client);
    }
    findAngles(&pitch,&roll,&yaw);
    motorTest = Serial.read();
    switch (motorTest)
    {
    case 1:
      ESCFR.writeMicroseconds(throttleCmd);
      ESCFL.writeMicroseconds(MIN_SPEED);
      ESCBR.writeMicroseconds(MIN_SPEED);
      ESCBL.writeMicroseconds(MIN_SPEED);
      break;
    case 2:
      ESCFR.writeMicroseconds(MIN_SPEED);
      ESCFL.writeMicroseconds(throttleCmd);
      ESCBR.writeMicroseconds(MIN_SPEED);
      ESCBL.writeMicroseconds(MIN_SPEED);
      break;
    case 3:
      ESCFR.writeMicroseconds(MIN_SPEED);
      ESCFL.writeMicroseconds(MIN_SPEED);
      ESCBR.writeMicroseconds(throttleCmd);
      ESCBL.writeMicroseconds(MIN_SPEED);
      break;
    case 4:
      ESCFR.writeMicroseconds(MIN_SPEED);
      ESCFL.writeMicroseconds(MIN_SPEED);
      ESCBR.writeMicroseconds(MIN_SPEED);
      ESCBL.writeMicroseconds(throttleCmd);
      break;
    default:
      ESCFR.writeMicroseconds(MIN_SPEED);
      ESCFL.writeMicroseconds(MIN_SPEED);
      ESCBR.writeMicroseconds(MIN_SPEED);
      ESCBL.writeMicroseconds(MIN_SPEED);
      break;
    }
    accTotalVec[0] = sqrt((AccX*AccX)+(AccY*AccY)+(AccZ*AccZ));

    accAvgVec = accTotalVec[0];
    for(int start = 16; start > 0; start--){                                            //Do this loop 16 times to create an array of accelrometer vectors.
          accTotalVec[start] = accTotalVec[start - 1];                        //Shift every variable one position up in the array.
          accAvgVec += accTotalVec[start];                                     //Add the array value to the accAvgVec variable.
        }

        accAvgVec /= 17;                                                            //Divide the accAvgVec by 17 to get the avarage total accelerometer vector.

        if(vibeCounter < 20){                                                     //If the vibeCounter is less than 20 do this.
          vibeCounter ++;                                                         //Increment the vibeCounter variable.
          vibeResult += abs(accTotalVec[0] - accAvgVec);                          //Add the absolute difference between the avarage vector and current vector to the vibeResult variable.
        }
        else{
          vibeCounter = 0;                                                        //If the vibeCounter is equal or larger than 20 do this.
          Serial.println(vibeResult/50);                                    //Print the total accelerometer vector divided by 50 on the serial monitor.
          vibeResult = 0;                                                   //Reset the vibeResult variable.
        }
}
