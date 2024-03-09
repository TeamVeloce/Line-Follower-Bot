#include <L298N.h>
#include <QTRSensors.h>

QTRSensors qtr;

int leftMotor1 = 8;
int leftMotor2 =7;
int rightMotor1 = 6;
int rightMotor2 = 4;
#define enA 10
#define enB 5

L298N motor1(enA, leftMotor1, leftMotor2);
L298N motor2(enB, rightMotor1, rightMotor2);

unsigned long cTime, pTime;
float eTime;

uint16_t positionLine;

int lastError=0;
int motorbasespeed=120;

float Kp=1;
float Ki=0.0;
float Kd=0.0;

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

int matrix[4];

int flag = 0;
int passes = 0;

void setup() {
  // put your setup code here, to run once:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7},SensorCount);
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
  //delay(10000);
}

void loop() {
  // Check with ultrasonic sensor
  // Check for Left or right turn

  // Check for matrix
  // put your main code here, to run repeatedly:
  PID_Controll();
  

}

void matrixCalc(int error1)
{
  
  //Serial.println(error1);
  positionLine = qtr.readLineBlack(sensorValues);
  
  int error2 = 3500 - positionLine;
  //Serial.println(error2);
  unsigned long start = millis();

  unsigned long error_change = error2 - error1 ;

  if(error_change < -4000 || error_change > -5500)
  {
    unsigned long stop = millis();
    if((stop - start) < 100000)
    {
      if(error1>3450 || error1 <3550)
      {
        matrix[1] = 1;
        matrix[0] = 0;
        if(error2>-1850 || error2 <-1450)
          {
            matrix[3] = 0;
            matrix[2] = 1;
          }
          flag = 1;
      }
    if(passes == 0)
    {
    for(int i=3 ; i>=0 ; i--)
    {
      Serial.print(matrix[i]);
    }
    Serial.println();
    passes=1;
    }
    //continue;

  }
  }
  /*
  if(error_change >950 || error_change < 1050 ||error_change <-950 || error_change > -1050 || error_change > -50 || error_change <50 )
  {
    unsigned long stop = millis();
    if((stop - start) < 50)
    {
      if(error1>450 || error1 <550)
      {
        matrix[1] = '1';
        matrix[0] = '0';
        if(error2>-50 || error2 <50)
          {
            matrix[1] = '1';
            matrix[0] = '0';
          }
      }

      for(int i=0 ; i<4 ; i++)
    {
      Serial.print(matrix[i]);
    }
    Serial.println();

    //fla
      
    }
    
  }*/
}

void PID_Controll() {
  positionLine = qtr.readLineBlack(sensorValues);

  unsigned long pass_time = millis();
  
  int error = 3500 - positionLine;
  if(error== 3500 || error == -1500)
  {
      matrixCalc(error);

  }

  //matrixCalc(error);
  //Serial.print(error);
  //Serial.println(error);
  while(sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980 && sensorValues[5]>=980 && sensorValues[6]>=980){ // A case when the line follower leaves the line
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
