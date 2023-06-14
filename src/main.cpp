#include <Arduino.h>
#include <ESP32Servo.h> 
#include <Wire.h>
#include <MPU_funcs.h>

///////////////// COMMS /////////////////////////////
#include <Comms_funcs_gui.h>
//////////////// COMMS //////////////////////////////
#define KILL_SPEED 950
#define MIN_SPEED 1000
#define MAX_SPEED 2000  
#define MOTOR_PIN_FR 19
#define MOTOR_PIN_FL 18
#define MOTOR_PIN_BR 17 
#define MOTOR_PIN_BL 16
//-------------------
#define YAW_INDEX 0
#define PITCH_INDEX 1
#define ROLL_INDEX 2
#define THROTTLE_INDEX 3
//--------------------
#define LED_PIN 26
//--------------------
volatile unsigned int pulseLength[4] = {1500, 1500, 1500, 1200};
unsigned long ESCFRspeed, ESCFLspeed, ESCBRspeed, ESCBLspeed = 1000;
volatile int throttle;

// GLOBAL VARS FOR PID CONTROLLER 
float angle_Setpoints[3] = {0,0,0}; // YAW, PITCH, ROLL
float errors[3]; 
float errorsSum[3] = {0,0,0}; // FOR THE INTEGRAL PART
float errorsDelta[3] = {0,0,0}; // FOR THE DERIVATIVE PART
float previousError[3] = {0,0,0}; // FOR THE DERIVATIVE PART

//PID COEFFICIENTS YAW,PITCH, ROLL
volatile float Kp[3] = {3 ,0 ,0};
volatile float Ki[3] = {0.02 ,0 ,0};
volatile float Kd[3] = {0 ,0 ,0};

Servo ESCFR;
Servo ESCFL;
Servo ESCBR;
Servo ESCBL;

float pitch, roll, yaw;
float yawCommand, pitchCommand, rollCommand = 0;
//float batteryVoltage;

void escInit(){
  ESCFR.attach(MOTOR_PIN_FR,MIN_SPEED,MAX_SPEED);
  ESCFL.attach(MOTOR_PIN_FL,MIN_SPEED,MAX_SPEED);
  ESCBR.attach(MOTOR_PIN_BR,MIN_SPEED,MAX_SPEED);
  ESCBL.attach(MOTOR_PIN_BL,MIN_SPEED,MAX_SPEED);

}

void getAngles(){
  findAngles(&pitch,&roll,&yaw);
}

void escCalibration()
{
  // SEND MAX OUTPUT
  Serial.print("Now writing maximum output: (");
  Serial.print(MAX_SPEED);
  Serial.print(" us in this case)");
  Serial.print("\n");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  ESCFR.writeMicroseconds(MAX_SPEED);
  ESCFL.writeMicroseconds(MAX_SPEED);
  ESCBR.writeMicroseconds(MAX_SPEED);
  ESCBL.writeMicroseconds(MAX_SPEED);

  // WAIT FOR INPUT
  while (Serial.available() <= 0)
  {
    delay(100);
  }

  // SEND MIN OUTPUT
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Sending minimum output: (");
  Serial.print(MIN_SPEED);
  Serial.print(" us in this case)");
  Serial.print("\n");

  ESCFR.writeMicroseconds(MIN_SPEED);
  ESCFL.writeMicroseconds(MIN_SPEED);
  ESCBR.writeMicroseconds(MIN_SPEED);
  ESCBL.writeMicroseconds(MIN_SPEED);

  Serial.println("The ESCs are calibrated");
}

float minmax(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    value = max_value;
  }

  else if (value < min_value)
  {
    value = min_value;
  }

  return value;
}

void calculateErrors()
{
  // Calculate current errors
  errors[YAW_INDEX] = yaw - yawCommand;
  errors[PITCH_INDEX] = pitch - pitchCommand;
  errors[ROLL_INDEX] = roll - rollCommand;
 /*  Serial.print(errors[PITCH_INDEX]);
  Serial.print(" ////// ");
  Serial.print(errors[ROLL_INDEX]);
  Serial.print(" /////// ");
  Serial.println(errors[YAW_INDEX]); */

  // Calculate sum of errors : Integral coefficients

  errorsSum[YAW_INDEX] += errors[YAW_INDEX];
  errorsSum[PITCH_INDEX] += errors[PITCH_INDEX];
  errorsSum[ROLL_INDEX] += errors[ROLL_INDEX];

  // Keep values in acceptable range

  errorsSum[YAW_INDEX] = minmax(errorsSum[YAW_INDEX], -200, 200);
  errorsSum[PITCH_INDEX] = minmax(errorsSum[PITCH_INDEX], -200, 200);
  errorsSum[ROLL_INDEX] = minmax(errorsSum[ROLL_INDEX], -200, 200);

  // Calculate error delta : Derivative coefficients
  errorsDelta[YAW_INDEX] = errors[YAW_INDEX] - previousError[YAW_INDEX];
  errorsDelta[PITCH_INDEX] = errors[PITCH_INDEX] - previousError[PITCH_INDEX];
  errorsDelta[ROLL_INDEX] = errors[ROLL_INDEX] - previousError[ROLL_INDEX];

  // Save current error as previous_error for next time
  previousError[YAW_INDEX] = errors[YAW_INDEX];
  previousError[PITCH_INDEX] = errors[PITCH_INDEX];
  previousError[ROLL_INDEX] = errors[ROLL_INDEX];
}

void resetPidController()
{
  errors[YAW_INDEX] = 0;
  errors[PITCH_INDEX] = 0;
  errors[ROLL_INDEX] = 0;

  errorsSum[YAW_INDEX] = 0;
  errorsSum[PITCH_INDEX] = 0;
  errorsSum[ROLL_INDEX] = 0;

  previousError[YAW_INDEX] = 0;
  previousError[PITCH_INDEX] = 0;
  previousError[ROLL_INDEX] = 0;
}

void pidController(){
  float yawPID = 0;
  float pitchPID = 0;
  float rollPID = 0;
  ///////////////// COMMS /////////////////////////////
  Kp[PITCH_INDEX] = newKp;
  Kp[ROLL_INDEX] = newKp;
  Ki[PITCH_INDEX] = newKi;
  Ki[ROLL_INDEX] = newKi;
  Kd[PITCH_INDEX] = newKd;
  Kd[ROLL_INDEX] = newKd;
  ///////////////// COMMS /////////////////////////////
  throttle  = throttleCmd; // Received throttle_pulse 
    // Initialize motor commands with throttle
  ESCFRspeed = throttle;
  ESCFLspeed = throttle;
  ESCBRspeed = throttle;
  ESCBLspeed = throttle;

  if(throttle >= 1100){
    // PID = e.Kp + ∫e.Ki + Δe.Kd
    yawPID = (errors[YAW_INDEX]*Kp[YAW_INDEX])+(errorsSum[YAW_INDEX]*Ki[YAW_INDEX])+(errorsDelta[YAW_INDEX]*Kd[YAW_INDEX]);
    pitchPID = (errors[PITCH_INDEX]*Kp[PITCH_INDEX])+(errorsSum[PITCH_INDEX]*Ki[PITCH_INDEX])+(errorsDelta[PITCH_INDEX]*Kd[PITCH_INDEX]);
    rollPID = (errors[ROLL_INDEX]*Kp[ROLL_INDEX])+(errorsSum[ROLL_INDEX]*Ki[ROLL_INDEX])+(errorsDelta[ROLL_INDEX]*Kd[ROLL_INDEX]);
    // LIMIT THE VALUES OF THE CORRECTION
    yawPID = minmax(yawPID,-200,200);
    pitchPID = minmax(pitchPID,-200,200);
    rollPID = minmax(rollPID,-200,200);
    // Calculate pulse duration for each ESC
    ESCFRspeed = throttle + rollPID + pitchPID - yawPID;
    ESCFLspeed = throttle - rollPID + pitchPID + yawPID;
    ESCBRspeed = throttle + rollPID - pitchPID + yawPID;
    ESCBLspeed = throttle - rollPID - pitchPID - yawPID;
    // Prevent out-of-range-values
    ESCFRspeed = minmax(ESCFRspeed, 1050, 1800);
    ESCFLspeed = minmax(ESCFLspeed, 1050, 1800);
    ESCBRspeed = minmax(ESCBRspeed, 1050, 1800);
    ESCBLspeed = minmax(ESCBLspeed, 1050, 1800);
  }
}

/* float calculateSetPoint(int desiredPulse)
{
  float set_point = 0;

  // Need a dead band of 16µs for better result
  if (desiredPulse > 1510){
    set_point = desiredPulse - 1510;
  }
  else if (desiredPulse < 1490){
    set_point = desiredPulse - 1490;
  }
  set_point /= 15; // Value 15 limits maximum angle value to ±32.8°

  return set_point;
} */

/* void calculateSetPoints()
{
  angle_Setpoints[YAW_INDEX] = calculateYawSetPoint(pulseLength[YAW_INDEX], pulseLength[THROTTLE_INDEX]);
  angle_Setpoints[PITCH_INDEX] = calculateSetPoint(pulseLength[PITCH_INDEX]);
  angle_Setpoints[ROLL_INDEX] = calculateSetPoint(pulseLength[ROLL_INDEX]);
} */

/* float calculateYawSetPoint(int desired_yaw, int desired_throttle_pulse)
{
  float set_point = 0;
  // Do not yaw when turning off the motors
  if (desired_throttle_pulse > 1050){
    // There is no notion of angle on this axis as the quadcopter can turn on itself
    set_point = calculateSetPoint(desired_yaw);
  }

  return set_point;
} */

void secMeasure(){
  //ESCFLspeed, ESCFRspeed, ESCBRspeed, ESCBLspeed = 1000;
  ESCFR.writeMicroseconds(1000);
  ESCFL.writeMicroseconds(1000);
  ESCBR.writeMicroseconds(1000);
  ESCBL.writeMicroseconds(1000);
  pinMode(LED_PIN,0);
  delay(15000);
}

void motorSpeed(){
  ESCFR.writeMicroseconds(ESCFRspeed);
  ESCFL.writeMicroseconds(ESCFLspeed);
  ESCBR.writeMicroseconds(ESCBRspeed);
  ESCBL.writeMicroseconds(ESCBLspeed);
}

//float now, previousUpdate, deltaTime = 0;


void setup(){
  pinMode(LED_PIN,OUTPUT);
  Serial.begin(9600);
  ///////////////// COMMS /////////////////////////////
  connectToWifi();
  server.begin();
  ///////////////// COMMS /////////////////////////////
  imuSetup();
  escInit(); 
  //escCalibration();
  imuCalibration();

  resetPidController();
}
void loop(){
  while (killFlag!=1){
    /* now = millis();
    deltaTime = now - previousTime; */
    ///////////////// ADDED FOR COMMS /////////////////////////////
    WiFiClient client = server.available();
    if (client) {
      handleClientRequest(client);
    }
    ///////////////// ADDED FOR COMMS /////////////////////////////
    digitalWrite(LED_PIN,1);
    getAngles();
    

    calculateErrors();
    pidController();
    motorSpeed();
    ///////////////// ADDED FOR COMMS /////////////////////////////
    /* log("Throttle :");
    logln(throttleCmd);
    log("Kp :");
    logln(Kp[PITCH_INDEX]);
    log("Ki :");
    logln(Ki[PITCH_INDEX]);
    log("Kd :");
    logln(Kd[PITCH_INDEX]);
    log("FLAG");
    logln(killFlag); */
    ///////////////// ADDED FOR COMMS /////////////////////////////
    
    ////////////////////////////////// ESCs DEBUG //////////////////////////////////
    /* int i = 0;
    if (i%500 == 0) {
      log("FRONT RIGHT : ");
      log(ESCFRspeed);
      logln("///////////");
      log("FRONT LEFT : ");
      log(ESCFLspeed);
      logln("///////////");
      log("BACK RIGHT : ");
      log(ESCBRspeed);
      logln("///////////");
      log("BACK LEFT : ");
      log(ESCBLspeed);
      logln("///////////");
    }
    i++; */
    ////////////////////////////////// ESCs DEBUG //////////////////////////////////
    // previousTime = now;
    /* logln(Kp[PITCH_INDEX]);
    logln(Kp[ROLL_INDEX]);
    logln(Ki[PITCH_INDEX]);
    logln(Ki[ROLL_INDEX]);
    logln(Kd[PITCH_INDEX]);
    logln(Kd[ROLL_INDEX]); */
  }
  secMeasure(); exit(0);
  /* if(deltaTime > 8){
    calculateErrors();
    pidController();
    motorSpeed();
  } */
}


// ROLL RIGHT IS GOOD, RIGHT MOTORS ARE ACCELERATING.
// ROLL LEFT IS GOOD, LEFT MOTORS ARE ACCELERATING.
// PITCH FORWARD IS GOOD, FRONT MOTORS ARE ACCELERATING.
// PITCH BACK IS GOOD, BACK MOTORS ARE ACCELERATING. 