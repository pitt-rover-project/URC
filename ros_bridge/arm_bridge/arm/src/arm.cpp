#include <Arduino.h>
#include "stepper_motor_class.h"

StepperMotor stepper1(38, A0, A1, 3, 0b00, Serial1);
bool done = false; 
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);          // TMC2209 UART (stepper motor drivers serial communication)
  Serial.println("Beginning arm test");
  stepper1.setup();
  Serial.print("  comms_ok="); Serial.println(stepper1.testConnection() == true ? "yes" : "NO");
}

void loop() {
  stepper1.run();
  if (done){return;}
  
  stepper1.move(10);
  done = true;


}