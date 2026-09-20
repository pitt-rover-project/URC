
#define channelA 22
#define channelB 23

#define DIR2 52
#define PWM2 3

int dir = 0;  // cw = 1; ccw = -1
int countA = 0;
int countB = 0;
double pwmspeed = 255;
int rotations = 5;

void setup() {
  Serial.begin(9600);

  pinMode (DIR2, OUTPUT); 
  pinMode (PWM2, OUTPUT); 
  pinMode (channelA, INPUT); 
  pinMode (channelB, INPUT);
  digitalWrite(DIR2, HIGH);
  attachInterrupt(digitalPinToInterrupt(channelA), checkQuadA, RISING);

  countA = 0;
}

//find counts per second
void loop() {
  double rps = 130 / 60;
  double rotationsAtSpeed = rps * (pwmspeed/255);
  double timePerRotation = 1.00/rotationsAtSpeed;
//  Serial.println(rps);
//  Serial.println(rotationsAtSpeed);
  double timePassed = micros() / 1e6;
  if(timePassed < (timePerRotation * rotations)) {
//    Serial.println(timePerRotation * rotations);
    Serial.println(timePassed);
    analogWrite(PWM2, pwmspeed);
    
  } else {
    Serial.println();
    analogWrite(PWM2, 0);
    Serial.println(countA);
    Serial.print("Count per rotation: ");
    Serial.println(countA / rotations);
  }

  
  
}

double revolutions(int count) {
  //encoder count per revolution (CPR) is equal to the (PPR * 4) i think this is wrong though
        //for the gobilda motor its 3895.9 PPR
  //encoder count was 11383 at 24 volts and after 10 seconds it turned 11.5 times
  //1348 1354 1347
//  count = count*4;
  
  return count;
}

void checkDirection() {
  if(digitalRead(channelB) == HIGH) {
    dir = 1;
  } else {
    dir = -1;
  }
}

void checkQuadA() {
  checkDirection();
  countA += dir;
}
