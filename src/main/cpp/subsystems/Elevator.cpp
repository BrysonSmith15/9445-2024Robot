// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

Elevator::Elevator()
/*: encoder{this->motorL.GetEncoder(
      rev::SparkRelativeEncoder::Type::kQuadrature, 8192)} */
{
  this->elevationController.SetTolerance(250);
}

bool Elevator::topPressed() { return this->topLimit.Get(); }
bool Elevator::botPressed() { return this->botLimit.Get(); }

void Elevator::setMotors(double powerPercent) {
  powerPercent = -powerPercent;
  // do not over reach the limit switches
  if (powerPercent > 0) {
    if (this->topPressed()) {
      frc::SmartDashboard::PutNumber("out", 0);
      this->motorL1.Set(0);
      this->motorR1.Set(0);
      this->motorL2.Set(0);
      this->motorR2.Set(0);
    } else {
      frc::SmartDashboard::PutNumber("out", powerPercent);
      this->motorL1.Set(powerPercent);
      this->motorL2.Set(powerPercent);
      this->motorR1.Set(powerPercent);
      this->motorR2.Set(powerPercent);
    }
  } else {
    if (this->botPressed()) {
      frc::SmartDashboard::PutNumber("out", 0);
      this->motorL1.Set(0);
      this->motorR1.Set(0);
      this->motorL2.Set(0);
      this->motorR2.Set(0);
    } else {
      frc::SmartDashboard::PutNumber("out", powerPercent);
      this->motorL1.Set(powerPercent);
      this->motorL2.Set(powerPercent);
      this->motorR1.Set(powerPercent);
      this->motorR2.Set(powerPercent);
    }
  }
}

double Elevator::calcPID(double setpoint) {
  return this->elevationController.Calculate(this->getTopTicks(), setpoint);
}

frc2::CommandPtr Elevator::manual(double powerPercent) {
  frc::SmartDashboard::PutNumber("elevator power", powerPercent);
  return this->RunOnce([this, powerPercent] { this->setMotors(powerPercent); });
}

int Elevator::getTopTicks() {
  // return (int)(this->encoder.GetPosition() * ElevatorConstants::gearRatio);
  return 0;
}

void Elevator::Periodic() {
  frc::SmartDashboard::PutNumber("Elevator Top Ticks", this->getTopTicks());
  // frc::SmartDashboard::PutBoolean("Going", false);
  if (this->botPressed()) {
    // this->encoder.SetPosition(ElevatorConstants::bottomTicks);
  } else if (this->topPressed()) {
    // TODO: fix the actual encoder value for the top
    // this->encoder.SetPosition(ElevatorConstants::climbTicks);
    std::cout << "Top Pressed\n";
    frc::SmartDashboard::PutBoolean("top", true);
  } else {
    frc::SmartDashboard::PutBoolean("top", false);
  }
}