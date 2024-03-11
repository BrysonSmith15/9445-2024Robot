// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {
  // Implementation of subsystem constructor goes here.
  this->motor.SetCANTimeout(75);
  this->motor.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 100);
  this->motor.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 500);
  this->motor.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 500);
  this->motor.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);
  this->motor.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 500);
  this->motor.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 500);
}

void Intake::run(double speed) {
  this->motor.Set(this->limiter.Calculate(speed));
}

bool Intake::limitPressed() { return !this->limit.Get(); }

void Intake::stop() { this->motor.Set(0); }

void Intake::Periodic() {
  // Implementation of subsystem periodic method goes here.
}