// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() {
  this->motorR1.Follow(this->motorL1, true);
  this->motorL1.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 500);
  this->motorR1.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 500);
  this->motorL1.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 500);
  this->motorR1.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 500);
  this->motorL1.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);
  this->motorR1.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);
}

void Shooter::setMotors(double percentPower) {
  this->motorL1.Set(percentPower);
}

double Shooter::getMotorOut() { return this->motorL1.GetAppliedOutput(); }

void Shooter::Periodic() {
  // Implementation of subsystem periodic method goes here.
}