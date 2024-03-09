// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>

Elevator::Elevator() {
  this->elevationController.SetTolerance(2.5);
  // TODO: check
  this->motorL1.SetCANTimeout(75);

  this->motorL1.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 10);
  this->motorL2.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 500);
  this->motorR1.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 500);
  this->motorR2.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 500);

  this->motorL2.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 500);
  this->motorR1.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 500);
  this->motorR2.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 500);

  this->motorL2.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);
  this->motorR1.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);
  this->motorR2.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);

  this->encoder.SetReverseDirection(true);
}

bool Elevator::topPressed() { return this->topLimit.Get(); }
bool Elevator::botPressed() { return this->botLimit.Get(); }

void Elevator::setMotors(double powerPercent) {
  powerPercent = -powerPercent;
  // do not over reach the limit switches
  if ((powerPercent > 0 && this->topPressed()) ||
      (powerPercent < 0 && this->botPressed())) {
    powerPercent = 0;
  } else {
    powerPercent = this->limiter.Calculate(powerPercent);
  }
  this->motorL1.Set(powerPercent);
  this->motorL2.Set(powerPercent);
  this->motorR1.Set(powerPercent);
  this->motorR2.Set(powerPercent);
}

int Elevator::getTicks() { return this->encoder.Get(); }

double Elevator::calcPID(double setpoint) {
  return !this->elevationController.AtSetpoint()
             ? this->elevationController.Calculate(this->getTicks(), setpoint)
             : 0.0;
}

void Elevator::Periodic() {
  if (this->botPressed()) {
    this->encoder.Reset();
  }
  if (this->elevationController.AtSetpoint()) {
    this->elevationController.Reset();
  }
}