// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() {
  this->motorR1.Follow(this->motorL1, true);
  // this->motorL2.Follow(this->motorL1, false);
  // this->motorR2.Follow(this->motorL1, true);
}

void Shooter::setMotors(double percentPower) {
  this->motorL1.Set(this->speedLimiter.Calculate(percentPower));
}

double Shooter::getMotorOut() { return this->motorL1.GetAppliedOutput(); }

void Shooter::Periodic() {
  // Implementation of subsystem periodic method goes here.
}