// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

Elevator::Elevator()
    : encoder{this->motorL.GetEncoder(
          rev::SparkRelativeEncoder::Type::kQuadrature, 8192)} {
  this->motorR.Follow(this->motorL, true);
  this->elevationController.SetTolerance(250);
}

bool Elevator::topPressed() { return this->topLimit.Get(); }
bool Elevator::botPressed() { return this->botLimit.Get(); }

void Elevator::setMotors(double powerPercent) {
  // do not over reach the limit switches
  if (!((this->botPressed() && powerPercent < 0) ||
        (this->topPressed() && powerPercent > 0))) {
    this->motorL.Set(powerPercent);
  }
}

double Elevator::calcPID(double setpoint) {
  return this->elevationController.Calculate(this->getTopTicks(), setpoint);
}

frc2::CommandPtr Elevator::manual(double powerPercent) {
  return this->RunOnce([this, powerPercent] { this->setMotors(powerPercent); });
}

int Elevator::getTopTicks() {
  return (int)(this->encoder.GetPosition() * ElevatorConstants::gearRatio);
}

void Elevator::Periodic() {
  frc::SmartDashboard::PutNumber("Elevator Top Ticks", this->getTopTicks());
  if (this->botPressed()) {
    this->encoder.SetPosition(ElevatorConstants::bottomTicks);
  } else if (this->topPressed()) {
    // TODO: fix the actual encoder value for the top
    // this->encoder.SetPosition(ElevatorConstants::climbTicks);
    std::cout << "Top Pressed\n";
  }
}