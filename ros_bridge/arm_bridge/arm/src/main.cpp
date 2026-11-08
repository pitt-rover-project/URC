#include <Arduino.h>
#include "stepper_motor_class.h"

//stepper1 =right wrist
//stepper2 = left wrist
StepperMotor stepper1(38, A0, A1, 3, 0b00, Serial1);
StepperMotor stepper2(38, A2, A3, 4, 0b01, Serial1);

bool done = false; 
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);          // TMC2209 UART (stepper motor drivers serial communication)
  Serial.println("Beginning arm test");
  stepper1.setup();
  stepper2.setup();
  Serial.print("  Stepper1 comms_ok="); Serial.println(stepper1.testConnection() == true ? "yes" : "NO");
  Serial.print("  Stepper2 comms_ok="); Serial.println(stepper2.testConnection() == true ? "yes" : "NO");
}

void loop() {
  stepper1.run();
  if (done){return;}
  
  stepper1.move(10);
  done = true;
}