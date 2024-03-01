#include <QTRSensors.h>
QTRSensors qtr;
int leftMotor1 = 8;
int leftMotor2 =7;
int rightMotor1 = 6;
int rightMotor2 = 4;
#define enA 10
#define enB 5

unsigned long cTime, pTime;
float eTime;

int lastError=0;
int motorbasespeed=100;

float Kp=60;
float Ki=1.0;
float Kd=0.0;

float Pvalue;
float Ivalue;
float Dvalue;

//int M1_Speed = 120; // speed of motor 1
//int M2_Speed = 120; // speed of motor 2
const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];


void setup() {
  // put your setup code here, to run once:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,2},SensorCount);
  Serial.begin(9600);

  delay(500);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);


  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  for (uint16_t i=0;i<400;i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN,LOW);
}

void loop() {
  // Check with ultrasonic sensor
  // Check for Left or right turn

  // Check for matrix
  // put your main code here, to run repeatedly:
  PID_Controll();
  

}

void PID_Controll() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  
  int error = 3500 - positionLine;
  //Serial.print(error);
  Serial.println(error);

  char pntX[100];
  char floX[10];

  int P = error;
  cTime = millis();
  eTime = (float)(cTime - pTime) / 1000;


  int I = I+error;
  int D = (error - lastError) / eTime;

  lastError = error;

  float motorspeed = (P*Kp)+(I*Ki)+(D*Kd);

  lastError = P;
  pTime = cTime;


  //int motorSpeedChange = (P*Kp)+(I*Ki)+(D*Kd);
  int motorSpeedA = motorbasespeed + motorspeed;
  int motorSpeedB = motorbasespeed - motorspeed;

  if (P <= 0) {
    moveForward(motorSpeedB, motorSpeedA);
  } else if (P > 0) {
    moveForward(motorSpeedA, motorSpeedB);
  }
}

void moveForward(int M1_Speed,int M2_Speed) {
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);
}

void stopMotors() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
