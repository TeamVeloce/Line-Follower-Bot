#include <L298N.h>
#include <QTRSensors.h>

QTRSensors qtr;
const int trigPin = 12;
const int echoPin = 11;
int flag=0;
int leftMotor1 = 8;
int leftMotor2 =7;
int rightMotor1 = 6;
int rightMotor2 = 4;
#define enA 10
#define enB 5

float duration;
float distance;

L298N motor1(enA, leftMotor1, leftMotor2);
L298N motor2(enB, rightMotor1, rightMotor2);

unsigned long cTime, pTime;
float eTime;

uint16_t positionLine;

int lastError=0;
int motorbasespeed=150;

float Kp=0.8;
float Ki=0.1;
float Kd=1.8;

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;

float Pvalue;
float Ivalue;
float Dvalue;

//int M1_Speed = 120; // speed of motor 1
//int M2_Speed = 120; // speed of motor 2
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


void setup() {
  // put your setup code here, to run once:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,2,3},SensorCount);
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

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
  delay(2000);
}

void loop() {
  // Check with ultrasonic sensor
  // Check for Left or right turn

  // Check for matrix
  // put your main code here, to run repeatedly:
  /*distance = obt();
  if (distance>=30){
    PID_Controll();
  }
  else{
    motor1.setSpeed(120);
    motor2.setSpeed(120);
    motor1.backward();
    motor2.backward();
    delay(500);
    motor1.setSpeed(200);
    motor2.setSpeed(200);
    motor1.forward();
    motor2.backward();
    delay(500);
    while(sensorValues[0]<900){
      motor1.setSpeed(200);
      motor2.setSpeed(200);
      motor1.forward();
      motor2.backward();
    }
  }*/

  PID_Controll();
    
}


void PID_Controll() {
  positionLine = qtr.readLineBlack(sensorValues);
  
  distance=obt();
 while (distance<=15){
    motor_drive(0,0);
    delay(1000);
    motor1.setSpeed(150);
    motor2.setSpeed(150);
    motor1.backward();
    motor2.backward();
    delay(1000);
    motor1.setSpeed(200);
    motor2.setSpeed(200);
    motor1.forward();
    motor2.backward();
    delay(500);
    motor_drive(0,0);
    delay(3000);
    distance=obt();
    positionLine= qtr.readLineBlack(sensorValues);
    Serial.println(distance);
    }
  
  int error = 3500 - positionLine;
  //Serial.print(error);
  //Serial.println(error);
  while(sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980 && sensorValues[5]>=980 && sensorValues[6]>=980 && sensorValues[7]>=980){ // A case when the line follower leaves the line
    if(lastError>0){       //Turn left if the line was to the left before
      motor_drive(-230,230);
    }
    else{
      motor_drive(230,-230); // Else turn right
    }
    positionLine = qtr.readLineBlack(sensorValues);
  }

  PID_line_follower(error);
}

void PID_line_follower(int error)
{
  int P = error;
  int I = I+error;
  int D = error - lastError;

  Pvalue = (Kp/pow(10,multiP))*P;
  Ivalue = (Ki/pow(10,multiI))*I;
  Dvalue = (Kd/pow(10,multiD))*D; 

  float PIDvalue = Pvalue + Ivalue + Dvalue;

  lastError = error;

  float motorspeed = (P*Kp)+(I*Ki)+(D*Kd);

  lastError = P;
  pTime = cTime;


  //int motorSpeedChange = (P*Kp)+(I*Ki)+(D*Kd);
  int motorSpeedA = motorbasespeed + PIDvalue;
  int motorSpeedB = motorbasespeed - PIDvalue;

   if (motorSpeedB > 255) {
      motorSpeedB = 255;
    }
    if (motorSpeedB < -255) {
      motorSpeedB = -255;
    }
    if (motorSpeedA > 255) {
      motorSpeedA = 255;
    }
    if (motorSpeedA < -255) {
      motorSpeedA = -255;
    }


  motor_drive(motorSpeedB, motorSpeedA);
}

void motor_drive(int left, int right){
  
  if(right>0)
  {
    motor2.setSpeed(right);
    motor2.forward();
  }
  else 
  {
    motor2.setSpeed(right);
    motor2.backward();
  }
  
 
  if(left>0)
  {
    motor1.setSpeed(left);
    motor1.forward();
  }
  else 
  {
    motor1.setSpeed(left);
    motor1.backward();
  }
}

int obt(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  //Serial.print("Distance: ");
  //Serial.println(distance);
  delay(100);
  return distance;
}
