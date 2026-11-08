#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include "stepper_motor_class.h"

class Arm {
  public:
    Arm()
    bool armSetup();
    bool moveWrist(float target, bool rotate);
    bool readSerial(String message);
    bool writeSerial(String message);

  private:
    float wristTarget;
    float wristAction; 
    float gripperTarget;
    StepperMotor stepper1;
    StepperMotor stepper2;
};

#endif