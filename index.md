## Welcome to the code Documentation for the 2 week Arduino Course. 

So i have listed the code documentation for all the components we will learn or have learned in the course.


## 1. Ultrasonic Sensor

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



