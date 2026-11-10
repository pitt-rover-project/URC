#include "stepper_motor_class.h"
#include "arm.h"

Arm::Arm()
  : pitchTarget(0), rollTarget(0),
    stepper1(38, A0, A1, 3, 0b00, Serial1), //right wrist
    stepper2(38, A2, A3, 4, 0b01, Serial1)  //left wrist
{}

bool Arm::armSetup()
{  
  stepper1.setup();
  stepper2.setup();
  return stepper1.testConnection() && stepper2.testConnection();
}

void Arm::runWrist()
{
    stepper1.run();
    stepper2.run();
}

void Arm::angleWrist(float degrees)
{
  pitchTarget = degrees;
  updateMotors();
}

void Arm::rotateWrist(float degrees)
{
  rollTarget = degrees;
  updateMotors();
}

WristPosition Arm::getPosition()
{
  WristPosition pos;
  pos.stepper1 = stepper1.getCurrentPosition();
  pos.stepper2 = stepper2.getCurrentPosition();
  return pos;
}

bool Arm::readSerial(String message)
{
    return false;
}

bool Arm::writeSerial(String message)
{
    return false;
}

bool Arm::wristDone()
{
    return stepper1.isDone() && stepper2.isDone();
}

void Arm::updateMotors()
{
  stepper1.moveTo(rollTarget + pitchTarget);
  stepper2.moveTo(-(rollTarget - pitchTarget));
}
