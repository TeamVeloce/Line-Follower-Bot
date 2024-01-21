//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"
#include <QTRSensors.h>
#include <math.h>
QTRSensors qtr;

//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234";  // Change this to more secure PIN.

String device_name = "ESP32-BT-Slave";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define leftMotor1 'D26'
#define leftMotor2 'D27'
#define rightMotor1 'D14'
#define rightMotor2 'D12'
#define enA 'D25'
#define enB 'D13'
#define leftSensor1  'D19'
#define  leftSensor2 'D18'
#define centreleft  'D5'
#define centreright  'D4'
#define rightSensor1 'D2'
#define rightSensor2 'D15'

unsigned long cTime, pTime;
float eTime;
int lastError = 0;

BluetoothSerial SerialBT;
float Kp = 0;
float Ki = 0;
float Kd = 0;
float multiplier=0;
float divisor=1;
float P=1;
float I=1;
float D=1;
float Pvalue;
float Ivalue;
float Dvalue;

/*int leftSensor1 = D19;
int leftSensor2 = D18;
int centreleft = D5;
int centreright = D4;
int rightSensor1 = D2;
int rightSensor2 = D15;*/


const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 'D19', 'D18', 'D5', 'D4', 'D2', 'D15' }, SensorCount);
  Serial.begin(115200);

  SerialBT.begin(device_name);  //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
//Serial.printf("The device with name \"%s\" and MAC address %s is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), SerialBT.getMacString()); // Use this after the MAC method is implemented
delay(500);

  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  pinMode(leftSensor1, INPUT);
  pinMode(leftSensor2, INPUT);
  pinMode(rightSensor1, INPUT);
  pinMode(rightSensor2, INPUT);

  pinMode(enA,OUTPUT);
  pinMode(enB,OUTPUT);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(SerialBT.read());
  }
  if (SerialBT.available()) {
    //SerialBT.write(SerialBT.read());
    //Serial.println(SerialBT.read());
    //Serial.write(SerialBT.read());
    switch (SerialBT.read()) 
    {
      case 1:
        
        Kp= SerialBT.read();
       //Serial.println("Kp");
        //Serial.println(Kp);
        break;

      case 2:
        multiplier=SerialBT.read();
        divisor=pow(10,multiplier);
        Serial.println("kp ");
        P = Kp/divisor;
        Serial.println(P,5);
        break;

      case 3:
        Ki= SerialBT.read();
       // Serial.print("Ki");
        //Serial.println(Ki);
        break;

      case 4:
        multiplier=SerialBT.read();
        divisor=pow(10,multiplier);
        Serial.println("ki ");
        I = Ki/divisor;
        Serial.println(I, 5);
        break;

      case 5:
        Kd= SerialBT.read();
        //Serial.print("Kd");
        //Serial.print(Kd);
        break;

      case 6:
        multiplier=SerialBT.read();
        divisor=pow(10,multiplier);
        Serial.println("kd ");
        D = Kd/divisor;
        Serial.println(D, 5);
        break;
        
        
      default:
        Kp = 0;
        Ki = 0;
        Kd = 0;
        break;
    }
  }
  delay(20);
}

  void PID_Controll() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  Serial.println(positionLine);
  int error = 2500 - positionLine;

  char pntX[100];
  char floX[10];

  int p = error;
  cTime = millis();
  eTime = (float)(cTime - pTime) / 1000;


  int i = i * 2 / 3 + p * eTime;
  int d = (error - lastError) / eTime;

  Pvalue = P * p;
  Ivalue = I * i;
  Dvalue = D * d;

  lastError = error;

  float motorspeed = Pvalue + Ivalue + Dvalue;

  lastError = p;
  pTime = cTime;


  //int motorSpeedChange = (P*Kp)+(I*Ki)+(D*Kd);
  int motorSpeedA = 100 + motorspeed;
  int motorSpeedB = 100 - motorspeed;

  if (p <= 0) {
    moveForward(motorSpeedB, motorSpeedA);
  } else if (p > 0) {
    moveForward(motorSpeedA, motorSpeedB);
  
  }

}
void moveForward(motorSpeedA, motorSpeedB) {
   digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);
}

void stopMotors() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
