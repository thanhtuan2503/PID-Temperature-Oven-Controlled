#include <SimpleKalmanFilter.h>
SimpleKalmanFilter bo_loc(1, 1, 1);
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "PIDver1.h"
//#define mySetp  45
double mySetp = 30;
#define myKp  900
#define myKi  0
#define myKd  0
#define mySetp  50
#define myOutMax  1001
#define myOutMin  0
#define myTime 0.1
#define RELAY_PIN 46 
PID myPID(myKp, myKi, myKd, myTime, myOutMax, myOutMin,1);
unsigned long time;
int maxSO =  7;
int maxCS = 6;
int maxSCK = 5;
//Create a MAX31855 reference and tell it what pin does what
unsigned long windowStartTime;
int WindowSize = 1000;
Adafruit_MAX31855 kTC(maxSCK, maxCS, maxSO);
double Out = 0;
double Output2;
unsigned long a = 100;
void setup() {
  Serial.begin(9600);
  pinMode(46, OUTPUT);
  // The MAX31855 needs a little time to stabilize
  windowStartTime = millis();
  digitalWrite(46, 0);
  delay(500);
}
unsigned long preTime = millis();
void loop() {  
  float T_filter;
  float Tfinal;
  unsigned long AnsW = millis() - preTime;
  long a = AnsW;
//  if(a < 100000)  mySetp = 35 + 15*a/100000;
//  else mySetp = 50;
//  Serial.println(mySetp);
//  Serial.println("hihi");
 if (Out > (millis() - windowStartTime)) Output2 = Out/10; 
 else Output2 = 0;
 //Serial.println("hihi");
 unsigned long value = millis() - windowStartTime;
 unsigned long soDu = value%100;
 unsigned long soDu2 = value%300;
 //Serial.println(soDu);
// if (!soDu) Serial.println(soDu);
 if (!soDu){
//    Serial.println("hihi");
  if(!soDu2){
    Serial.print("C = "); 
    Serial.print(mySetp);   
    Serial.print(",");  
   }
    Tfinal = kTC.readCelsius();
    T_filter = bo_loc.updateEstimate(Tfinal);
    Out = myPID.Calculate(mySetp, T_filter );  
//    Serial.print("C = "); 
//    Serial.print(mySetp);   
//    Serial.print(",");  
  if(!soDu2){
    Serial.print(T_filter);
    Serial.print(",");
    Serial.print(Output2);
    Serial.println();
  }
    time = millis();
//    a = a + 100;
    if ((millis() - windowStartTime) > WindowSize)
    { //time to shift the Relay Window
      windowStartTime = windowStartTime + WindowSize;
//      a = 100;
    }

 }
  if (Out > millis() - windowStartTime) digitalWrite(RELAY_PIN, 1);
  else digitalWrite(RELAY_PIN, 0);

/*  Serial.print("F = ");
  Serial.println(kTC.readFahrenheit());*/
  // delay so it doesn't scroll too fast.
}
