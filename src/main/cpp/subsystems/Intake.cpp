// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() {
  // Implementation of subsystem constructor goes here.
}

void Intake::run(bool forward) {
  this->motor.Set(this->limiter.Calculate(forward ? 1 : -1));
}

void Intake::stop() { this->motor.Set(this->limiter.Calculate(0)); }

void Intake::Periodic() {
  // Implementation of subsystem periodic method goes here.
}