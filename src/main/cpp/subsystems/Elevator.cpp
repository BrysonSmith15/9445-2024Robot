// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

Elevator::Elevator()
    : encoder{this->motorL1.GetEncoder(
          rev::SparkRelativeEncoder::Type::kQuadrature, 8192)} {
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
  // TODO: Replace if above breaks elevator
  /*
  if (powerPercent > 0) {
    if (this->topPressed()) {
      this->motorL1.Set(0);
      this->motorR1.Set(0);
      this->motorL2.Set(0);
      this->motorR2.Set(0);
    } else {
      this->motorL1.Set(powerPercent);
      this->motorL2.Set(powerPercent);
      this->motorR1.Set(powerPercent);
      this->motorR2.Set(powerPercent);
    }
  } else {
    if (this->botPressed()) {
      this->motorL1.Set(0);
      this->motorR1.Set(0);
      this->motorL2.Set(0);
      this->motorR2.Set(0);
    } else {
      this->motorL1.Set(powerPercent);
      this->motorL2.Set(powerPercent);
      this->motorR1.Set(powerPercent);
      this->motorR2.Set(powerPercent);
    }
  }
  */
}

double Elevator::calcPID(double setpoint) {
  return !this->elevationController.AtSetpoint()
             ? this->elevationController.Calculate(this->getTopDegs().value(),
                                                   setpoint)
             : 0.0;
}

units::degree_t Elevator::getTopDegs() {
  // https://www.desmos.com/calculator/4vcdc3fi7o
  // TODO: Make sure encoder measures rotations, not ticks
  // ticks -> 0=>8192+, rotations-> 0=>4+
  if (this->lastTouchedBottom) {
    return (90_deg * ElevatorConstants::gearRatio * this->encoder.Get());
  } else {
    return 90_deg * (1 - ElevatorConstants::gearRatio * this->encoder.Get());
  }
}

void Elevator::Periodic() {
  if (this->botPressed()) {
    this->encoder.SetPosition(0);
  } else if (this->topPressed()) {
    this->encoder.SetPosition(8192 / ElevatorConstants::gearRatio);
  }
  this->prevBot = this->botPressed();
  this->prevTop = this->topPressed();
  std::cout << this->getTopDegs().value() << '\n';
}