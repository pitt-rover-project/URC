#include "stepper_motor_class.h"
#include "arm.h"

Arm::Arm()
  : stepper1(38, A0, A1, 3, 0b00, Serial1),   // right wrist
    stepper2(38, A2, A3, 4, 0b01, Serial1)    // left wrist
{}

bool Arm::armSetup()
{  
  stepper1.setup();
  stepper2.setup();
  return stepper1.testConnection() && stepper2.testConnection();
}

bool Arm::moveWrist(float target, bool rotate)
{
    return false;
}

bool Arm::readSerial(String message)
{
    return false;
}

bool Arm::writeSerial(String message)
{
    return false;
}