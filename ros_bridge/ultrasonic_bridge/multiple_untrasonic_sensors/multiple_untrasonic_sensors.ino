// This code is for checking the distance between a number of arduino ultrasonic sensors and the object directly infront of the sensors

//trig and echo pins for each ultrasonic sensor (no pwm needed)
int trigPin1 = 8;
int echoPin1 = 9;
int trigPin2 = 50;
int echoPin2 = 51;
int trigPin3 = 48;
int echoPin3 = 49;
double duration, cm;
String distance = "";

void setup() {
  Serial.begin(115200);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
}

void loop() {
  //need to turn on/off each ultrasonic sensor one at a time to avoid interference

  //turn on/off the first ultrasonic
  distance = "";
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration = pulseIn(echoPin1, HIGH);
  cm = (duration/2) / 29.1;
  distance += String(cm) + ", ";

  //turn on/off the first ultrasonic
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration = pulseIn(echoPin2, HIGH);
  cm = (duration/2) / 29.1;
  distance += String(cm) + ", ";

  //turn on/off the first ultrasonic
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration = pulseIn(echoPin3, HIGH);
  cm = (duration/2) / 29.1;
  distance += String(cm) + ", ";

  //print the distance for each untrasonic sensor
  //starting with sensor 1, each distance is recorded in cm and each ultrasonic sensor is seperated by a comma
  Serial.println(distance);
  
}
