// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {
  // Implementation of subsystem constructor goes here.
}

void Intake::run(double speed) {
  this->motor.Set(this->limiter.Calculate(speed));
}

bool Intake::limitPressed() { return !this->limit.Get(); }

void Intake::stop() { this->motor.Set(0); }

void Intake::Periodic() {
  // Implementation of subsystem periodic method goes here.
}