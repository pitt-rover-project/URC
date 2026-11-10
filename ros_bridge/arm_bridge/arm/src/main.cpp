#include <Arduino.h>
#include "stepper_motor_class.h"
#include "arm.h"

bool done = false; 
 Arm arm;
 unsigned long lastPrint = 0;
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);          // TMC2209 UART (stepper motor drivers serial communication)

  Serial.println("Beginning arm test");
 
  if (!arm.armSetup())
  {
    Serial.println("Stepper driver commds FAILED");
    return;
  }
  arm.rotateWrist(-45);
  WristPosition pos = arm.getPosition();
  Serial.println("Starting positions:");
  Serial.print("Stepper 1: ");
  Serial.println(pos.stepper1);
  Serial.print("Stepper 2: ");
  Serial.println(pos.stepper2);

}

void loop() {
  arm.runWrist();
  if (!done && arm.wristDone())
  {
    WristPosition pos = arm.getPosition();
    Serial.print("Move complete: ");
    Serial.print(pos.stepper1);
    Serial.print(", ");
    Serial.println(pos.stepper2);
    done = true;
  }
}