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

WristPosition Arm::getPosition()
{
  WristPosition pos;
  pos.stepper1 = stepper1.getCurrentPosition();
  pos.stepper2 = stepper2.getCurrentPosition();
  return pos;
}

bool Arm::parseSerial(String message)
{
  // message format: "pitch,roll"   e.g. "30,-45"
  int comma = message.indexOf(',');
  if (comma < 0) return false;                 // no comma -> bad message

  String pitchStr = message.substring(0, comma);
  String rollStr  = message.substring(comma + 1);
  pitchStr.trim();                             // remove spaces / \r
  rollStr.trim();
  if (pitchStr.length() == 0 || rollStr.length() == 0) return false;

  pitchTarget = pitchStr.toFloat();
  rollTarget  = rollStr.toFloat();
  updateMotors();
  return true;                                 // success
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
