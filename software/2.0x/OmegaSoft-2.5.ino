/* Delta Space Systems
      Version 2.5
    October, 12th 2020 */

/*System State: 
 * 0 = Go/No Go before launch
 * 1 = PID Controlled Ascent
 * 2 = MECO
 * 3 = Chute Deployment
 * 4 = Descent
 * 5 = Abort
 */

//Libraries
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "BMI088.h"
#include <BMP280_DEV.h>                           

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

float temperature, pressure, altitude;            
BMP280_DEV bmp280;     

double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, OreX, OreY, OreZ;
double PreviousGyroX, PreviousGyroY, PreviousGyroZ, IntGyroX, IntGyroY, RADGyroX, RADGyroY, RADGyroZ, DifferenceGyroX, DifferenceGyroY, DifferenceGyroZ, matrix1, matrix2, matrix3;
double matrix4, matrix5, matrix6, matrix7, matrix8, matrix9, PrevGyX, PrevGyY, PrevGyZ, RawGyX, RawGyY, GyroAngleX, GyroAngleY, GyroAngleZ, GyroRawX, GyroRawY, GyroRawZ; 
double OUTXRAD, OUTYRAD, finalpwmX, finalpwmY, OUTXDEG, OUTYDEG;

//Upright Angle of the Flight Computer
int desired_angleX = 0;//servoY
int desired_angleY = 0;//servoX

//Offsets for tuning 
int servoY_offset = 25;
int servoX_offset = 145;

//Position of servos through the startup function
int servoXstart = servoY_offset;
int servoYstart = servoX_offset;

//The amount the servo moves by in the startup function
int servo_start_offset = 35;

//Ratio between servo gear and TVC mount
float servoX_gear_ratio = 6;
float servoY_gear_ratio = 6;

double OrientationX = 0;
double OrientationY = 0;
double OrientationZ = 1;
double Ax;
double Ay;

int ledred = 2;    // LED connected to digital pin 9
int ledblu = 5;    // LED connected to digital pin 9
int ledgrn = 6;    // LED connected to digital pin 9
int pyro1 = 25;
int buzzer = 21;
int teensyled = 13;
int state;
float motorThrust;

Servo servoX;
Servo servoY;

//Time variables
double dt, currentTime, previousTime;
unsigned long previousLog = 0; 
 
//Timer settings for log in Hz
const long logInterval = 250;  


//SD CARD CS (BUILTIN_SDCARD for MK5/MK6)
const int chipSelect = BUILTIN_SDCARD;

//"P" Constants
float pidX_p = 0;
float pidY_p = 0;

//"I" Constants
float pidY_i = 0;
float pidX_i = 0;

//"D" Constants
float pidX_d = 0;
float pidY_d = 0;

//PID Gains
double kp = 0.08;
double ki = 0.03;
double kd = 0.0225;

//Launch Site Altitude in Meters(ASL)
int launchsite_alt = 88.3;

//Rocket mass in kilograms
float rocketmass = 0.5;

//Distance between TVC pivot and Center of Mass in meters
float momentArm = 0.3;


void setup(){
 
  Serial.begin(9600);
  Wire.begin();
  servoX.attach(29);
  servoY.attach(30);
  bmp280.begin(BMP280_I2C_ALT_ADDR);              
  bmp280.startNormalConversion();  
  
  // Setting digital pins to output
  pinMode(ledblu, OUTPUT);
  pinMode(ledgrn, OUTPUT);
  pinMode(ledred, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(pyro1, OUTPUT);
  pinMode(teensyled, OUTPUT);
  
  startup();
  sdstart();
  launchpoll();
  
}
void loop() {
  //Defining Time Variables      
  currentTime = millis();            
  dt = (currentTime - previousTime) / 1000; 
  launchdetect();
  datadump();
  burnout();
  previousTime = currentTime;  
  
}

void rotationmatrices () {
  //Change Variable so its easier to refrence later on
  GyroRawX = (gyro.getGyroY_rads());
  GyroRawY = (gyro.getGyroZ_rads());
  GyroRawZ = (gyro.getGyroX_rads());

  //Integrate over time to get Local Orientation
  GyroAngleX += GyroRawX * dt;
  GyroAngleY += GyroRawY * dt;
  GyroAngleZ += GyroRawZ * dt;

  PreviousGyroX = RADGyroX;
  PreviousGyroY = RADGyroY;
  PreviousGyroZ = RADGyroZ;
  
  RADGyroX = GyroAngleX;
  RADGyroY = GyroAngleY;
  RADGyroZ = GyroAngleZ;
  
  DifferenceGyroX = (RADGyroX - PreviousGyroX);
  DifferenceGyroY = (RADGyroY - PreviousGyroY);
  DifferenceGyroZ = (RADGyroZ - PreviousGyroZ);

  OreX = OrientationX;
  OreY = OrientationY;
  OreZ = OrientationZ;
  
 //X Matrices
  matrix1 = (cos(DifferenceGyroZ) * cos(DifferenceGyroY));
  matrix2 = (((sin(DifferenceGyroZ) * -1) * cos(DifferenceGyroX) + (cos(DifferenceGyroZ)) * sin(DifferenceGyroY) * sin(DifferenceGyroX)));
  matrix3 = ((sin(DifferenceGyroZ) * sin(DifferenceGyroX) + (cos(DifferenceGyroZ)) * sin(DifferenceGyroY) * cos(DifferenceGyroX)));
  
 //Y Matrices
  matrix4 = sin(DifferenceGyroZ) * cos(DifferenceGyroY);
  matrix5 = ((cos(DifferenceGyroZ) * cos(DifferenceGyroX) + (sin(DifferenceGyroZ)) * sin(DifferenceGyroY) * sin(DifferenceGyroX)));
  matrix6 = (((cos(DifferenceGyroZ) * -1) * sin(DifferenceGyroX) + (sin(DifferenceGyroZ)) * sin(DifferenceGyroY) * cos(DifferenceGyroX)));

 //Z Matrices
  matrix7 = (sin(DifferenceGyroY)) * -1;
  matrix8 = cos(DifferenceGyroY) * sin(DifferenceGyroX);
  matrix9 = cos(DifferenceGyroY) * cos(DifferenceGyroX);

 OrientationX = ((OreX * matrix1)) + ((OreY * matrix2)) + ((OreZ * matrix3));
 OrientationY = ((OreX * matrix4)) + ((OreY * matrix5)) + ((OreZ * matrix6));
 OrientationZ = ((OreX * matrix7)) + ((OreY * matrix8)) + ((OreZ * matrix9));

Ax = asin(OrientationX) * (-180 / PI);
Ay = asin(OrientationY) * (180 / PI);

pidcompute();

}

void pidcompute () {
previous_errorX = errorX;
previous_errorY = errorY; 

errorX = Ax - desired_angleX;
errorY = Ay - desired_angleY;

//Defining "P" 
pidX_p = kp * errorX;
pidY_p = kp * errorY;

//Defining "D"
pidX_d = kd*((errorX - previous_errorX)/dt);
pidY_d = kd*((errorY - previous_errorY)/dt);

//Defining "I"
pidX_i = ki * (pidX_i + errorX * dt);
pidY_i = ki * (pidY_i + errorY * dt);

//Adding it all up
PIDX = pidX_p + pidX_i + pidX_d;
PIDY = pidY_p + pidY_i + pidY_d;

/*
pwmY = ((PIDY * servoY_gear_ratio) + servoX_offset);
pwmX = ((PIDX * servoX_gear_ratio) + servoY_offset); 
*/

//Disable lines 225-232 & change lines 236 & 237 to pwmY or pwmX for standard PID
OUTXRAD = asin((PIDX / (0.3 * motorThrust)) * (PI / 180));
OUTYRAD = asin((PIDY / (0.3 * motorThrust)) * (PI / 180));

OUTXDEG = OUTXRAD * (180 / PI);
OUTYDEG = OUTYRAD * (180 / PI);

finalpwmX = ((OUTXDEG * servoX_gear_ratio) + servoY_offset);
finalpwmY = ((OUTYDEG * servoY_gear_ratio) + servoX_offset);


//Servo outputs
servoX.write(finalpwmX);
servoY.write(finalpwmY);

}

void startup () {
  digitalWrite(ledblu, HIGH);
  digitalWrite(ledred, HIGH);
  digitalWrite(ledgrn, HIGH);
  tone(buzzer, 1050);
  delay(800);
  digitalWrite(ledgrn, LOW);
  tone(buzzer, 1150);
  delay(400);
  tone(buzzer, LOW);
  digitalWrite(ledred, LOW);
  digitalWrite(ledblu, HIGH);
  servoX.write(servoXstart);
  servoY.write(servoYstart);
  delay(500);
  digitalWrite(ledblu, LOW);
  digitalWrite(ledgrn, HIGH);
  servoX.write(servoXstart + servo_start_offset);
  delay(400);
  digitalWrite(teensyled, HIGH);
  digitalWrite(ledgrn, LOW);
  digitalWrite(ledred, HIGH);
  servoX.write(servoXstart - servo_start_offset);
  delay(200);
  servoX.write(servoXstart);
  delay(200);
  digitalWrite(ledred, LOW);
  digitalWrite(ledblu, HIGH);
  servoY.write(servoYstart + servo_start_offset);
  delay(400);
  servoY.write(servoYstart - servo_start_offset);
  delay(200);
  tone(buzzer, 1200, 250);
  servoY.write(servoYstart);
  delay(200);

  
 }
 
void launchdetect () {  
  accel.readSensor();
  if (state == 0 && accel.getAccelX_mss() > 12) {
  state++;
  }
  if (state == 1) {
  gyro.readSensor();
  digitalWrite(ledred, LOW);
  digitalWrite(ledgrn, LOW);
  digitalWrite(ledblu, HIGH);
  abortsystem();
  motorthrust();
  sdwrite();
  rotationmatrices();
  
 }
}
void datadump () {
  if (state >= 1) { 
    if (bmp280.getMeasurements(temperature, pressure, altitude))  {  
      
Serial.print("Pitch:  ");
Serial.println(Ax);
Serial.print(" Roll:  ");
Serial.print(Ay);
Serial.print(" Yaw:  ");
Serial.print(motorThrust);
Serial.print(" Motor Thrust:  ");
Serial.print(altitude - launchsite_alt);
Serial.print(" Altitude:  ");

    }
  }
Serial.print(" System State:  ");
Serial.println(state);

}

void sdstart () { 
if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
}

void sdwrite () {
String datastring = "";

 datastring += "Pitch,"; 
 datastring += String(Ax);
 datastring += ",";

 datastring += "Yaw,";
 datastring += String(Ay);
 datastring += ",";
 
 datastring += "System_State,";
 datastring += String(state);
 datastring += ",";
 
 datastring += "Z_Axis_Accel,";
 datastring += String(accel.getAccelZ_mss());
 datastring += ",";

 datastring += "Time,";
 datastring += String(millis());
 datastring += ",";

 datastring += "Altitude,";
 datastring += String(altitude);
 datastring += ",";



 
  File dataFile = SD.open("log001.txt", FILE_WRITE);
  
  if (dataFile) {
    if(currentTime - previousLog > logInterval){
    previousLog = currentTime;
    dataFile.println(datastring);
     dataFile.close();
    }
    
  }
  }

void burnout () { 
if (state == 1 && accel.getAccelX_mss() <= 2) {
  state++;
  digitalWrite(teensyled, LOW);
  digitalWrite(ledred, LOW);
  digitalWrite(ledblu, LOW);
  digitalWrite(ledgrn, HIGH);
  tone(buzzer, 1200, 200);
  Serial.println("Burnout Detected");
}

if (state == 1 || state == 2) {
  Serial.println(altitude - launchsite_alt);
}

if (state == 2 && (altitude - launchsite_alt) <= 0) {
  state++;
}
  
if(state == 3) {
  digitalWrite(pyro1, HIGH);
  digitalWrite(ledgrn, LOW);
  digitalWrite(ledblu, HIGH);
  Serial.println("Chute Deployment");
  state++;
}

if (state == 4) {

}
}

void launchpoll () {
  state == 0;
  delay(750);
  Serial.println("Omega is Armed");
     int status;
  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    while (1) {}
  }
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error"); 
    while (1) {}
  }
 
  float totalAccelVec = sqrt(sq(accel.getAccelZ_mss()) + sq(accel.getAccelY_mss()) + sq(accel.getAccelX_mss()));
  Ax = -asin(accel.getAccelZ_mss() / totalAccelVec);
  Ay = asin(accel.getAccelY_mss() / totalAccelVec);

  delay(500);
  digitalWrite(ledgrn, HIGH);
  digitalWrite(ledred, HIGH);
  digitalWrite(ledblu, LOW);
  
}

void abortsystem () {
  if (state == 1 && (Ax > 45 || Ax < -45) || (Ay > 45 || Ay < -45)){
    Serial.println("Abort Detected");
    digitalWrite(ledblu, HIGH);
    digitalWrite(ledgrn, LOW);
    digitalWrite(ledred, HIGH);
    digitalWrite(teensyled, LOW);
    tone(buzzer, 1200, 400);
    state++;
    state++;
    state++;
    state++; //Trasistion to State 5
    Serial.print("Abort");
    digitalWrite(pyro1, HIGH);
    
  }
}

void motorthrust() {
  motorThrust = rocketmass * accel.getAccelX_mss();

// Max and Min values for motor thrust
if (motorThrust > ((rocketmass * 10) * 1.5)) {
  motorThrust = ((rocketmass * 10) * 1.5);
}

if (motorThrust < ((rocketmass * 10) / 1.5)) {
  motorThrust = ((rocketmass * 10) / 1.5);
  
}
}
