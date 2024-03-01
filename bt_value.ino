#include<SoftwareSerial.h>

/* Create object named bt of the class SoftwareSerial */ 
SoftwareSerial bt(0,1); /* (Rx,Tx) */ 

float kp,ki,kd;
void setup(){
  bt.begin(9600); /* Define baud rate for software serial communication */
 Serial.begin(9600);  /* Define baud rate for serial communication */
}

void loop(){
    if (bt.available()) /* If data is available on serial port */
    {
    	Serial.write(bt.read());
      //Serial.println(bt.read());
      String a = bt.readString();
      String val="";
      
      //Serial.println(b);
      
      //Serial.println(a);

      //Serial.println(a[0]);
      char ch=a[0];
      for(int i=1;i<a.length();i++)
        {val += a[i];
          }

      float floatValue;
      floatValue = atof(val.c_str());
      

       float roundedValue =  round(floatValue * 100000.0) / 100000.0;

      //long intValue = val.toInt();
      //Serial.println(roundedValue,3 );  
      switch(ch){
        case 'i':
        {
          ki= roundedValue;
          break;
        }
        case 'd':
        {
          kd= roundedValue;
          break;
        }
        case 'p':
        {
          kp= roundedValue;
          break;
        }
        default:
        {
          ki= 0.0;
          kp=0.0;
          kd=0.0;
          break;
        }
      }
      Serial.println(kp);
      Serial.println(ki);
      Serial.println(kd);
      delay(10);
    }
}