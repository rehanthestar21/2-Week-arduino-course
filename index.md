## Welcome to the code Documentation for the 2 week Arduino Course. 

So i have listed the code documentation for all the components we will learn or have learned in the course.


## LED / Buzzer

```
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
```


##  Ultrasonic Sensor

```
const unsigned int TRIG_PIN=3;//trigger pin attached to digital pin 13
const unsigned int ECHO_PIN=2;//echo pin attached to digital pin 12
const unsigned int BAUD_RATE=9600;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(BAUD_RATE);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  

 const unsigned long duration= pulseIn(ECHO_PIN, HIGH);
 int distance= duration/29/2;
 if(duration==0){
   Serial.println("Warning: no pulse from sensor");
   } 
  else{
      Serial.print("distance to nearest object:");
      Serial.println(distance);
      Serial.println(" cm");
  }
 delay(100);
 }
```

## Light Sensor

```
void setup() {
  Serial.begin(9600);
}


void loop() {
  int value = analogRead(A0);
  Serial.println("Analog value : ");
  Serial.println(value);
  delay(250);
}
```

## RFID with TAG


## Humidity Sensor

```
#include "DHT.h"        // including the library of DHT11 temperature and humidity sensor
#define DHTTYPE DHT11   // DHT 11
#define dht_dpin 8	//data pin of DHT11 sensor attached to digital pin 2 of arduino
DHT dht(dht_dpin, DHTTYPE); 
void setup(){
  Serial.begin(9600);
  dht.begin();
  
} 
void loop(){
  float h = dht.readHumidity();
  float t = dht.readTemperature(); 
  Serial.println("Humidity and temperature\n\n");
  Serial.print("Current humidity = ");
  Serial.print(h);
  Serial.print("%  ");
  Serial.print("temperature = ");
  Serial.print(t);
}
```
## IR Sensor

```
int IRSensor = 2; // connect ir sensor to arduino pin 2
int LED = 13; // conect Led to arduino pin 13



void setup() 
{



  pinMode (IRSensor, INPUT); // sensor pin INPUT
  pinMode (LED, OUTPUT); // Led pin OUTPUT
}

void loop()
{
  int statusSensor = digitalRead (IRSensor);
  
  if (statusSensor == 1)
    digitalWrite(LED, LOW); // LED LOW
  }
  
  else
  {
    digitalWrite(LED, HIGH); // LED High
  }
  
}
```


## Servo Motor

```
#include <Servo.h>
int servoPin = 3; 	//servo motor data pin attached to digital pin 3 of arduino 
// Create a servo object 
Servo Servo1; 
void setup() { 
   // We need to attach the servo to the used pin number 
   Servo1.attach(servoPin); 
}
void loop(){ 
   // Make servo go to 0 degrees 
   Servo1.write(0); 
   delay(1000); 
   // Make servo go to 90 degrees 
   Servo1.write(90); 
   delay(1000); 
   // Make servo go to 180 degrees 
   Servo1.write(180); 
   delay(1000); 
}
```

## Soil Moisture Sensor

```
int sense=0;	//soil sensor input at analog pin A0
int value=0;
int led=13;		//led attached at digital pin 13 of arduino
void setup(){
Serial.begin(9600);
}
void loop(){
value=analogRead(sense);
value=value/10;
Serial.println(value);
if(value<50)
digitalWrite(led,HIGH);
else
digitalWrite(led,LOW);
}
```

## Second Code of the Soil Moisture Sensor

```
int sensorPin = A0; 
int sensorValue;  
int limit = 300; 

void setup() {
 Serial.begin(9600);
 pinMode(13, OUTPUT);
}

void loop() {

 sensorValue = analogRead(sensorPin); 
 Serial.println("Analog Value : ");
 Serial.println(sensorValue);
 
 if (sensorValue<limit) {
 digitalWrite(13, HIGH); 
 }
 else {
 digitalWrite(13, LOW); 
 }
 
 delay(1000); 
}
```


## Relay 

```
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
```


## Motor Driver

This is used as any other led which just inputs an digital output from the Arduino.


## Bluetooth Module 

```
#include<SoftwareSerial.h>
SoftwareSerial BT(10,11); //(Tx,Rx)
String readData;

void setup()
{
  BT.begin(9600);
  Serial.begin(9600);
 pinMode(A0, OUTPUT);
}

void loop()
{
  while(BT.available())
  {
  delay(10);
  char c = BT.read();
  readData+=c;
}

if(readData.length()>0)
{
  Serial.println(readData);

if(readData == "ON")
{
  analogWrite(A0, 180);
}

  
if(readData=="OFF")
  {
    analogWrite(A0, 0);
  }
  
  else
  {
  readData="";
}
}
}
```

## Final Code of the Bluetooth Controlled Car.


```
#include<SoftwareSerial.h>
SoftwareSerial BT(10,11); //(Tx,Rx)
String readData;
void setup()
{
  BT.begin(9600);
  Serial.begin(9600);
 pinMode(A0, OUTPUT);
 pinMode(A1, OUTPUT);
 pinMode(A3, OUTPUT);
 pinMode(A4, OUTPUT);
}
void loop()
{
  while(BT.available())
  {
  delay(10);
  char c = BT.read();
  readData+=c;
}

if(readData.length()>0)
{
  Serial.println(readData);
  if(readData=="Front")
  {
    analogWrite(A0, 180);
    analogWrite(A1, 0);
    analogWrite(A3, 0);
    analogWrite(A4, 180);
  }

  if(readData=="Back")
  {
    analogWrite(A0, 0);
    analogWrite(A1, 180);
    analogWrite(A3, 180);
    analogWrite(A4, 0);
  }
  
  if(readData=="Right")
  {
    analogWrite(A0, 0);
    analogWrite(A1, 180);
    analogWrite(A3, 0);
    analogWrite(A4, 180);
  }
  if(readData=="Left")
  {
    analogWrite(A0, 180);
    analogWrite(A1, 0);
    analogWrite(A3, 180);
    analogWrite(A4, 0);
  }
   
  if(readData=="OFF")
  {
    analogWrite(A0, 0);
    analogWrite(A1, 0);
    analogWrite(A3, 0);
    analogWrite(A4, 0);
  }
  
  else
  {
    digitalWrite(13 ,LOW);
  readData="";
}
}
}
```






