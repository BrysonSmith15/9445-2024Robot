// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

Elevator::Elevator() { this->elevationController.SetTolerance(2.5); }

bool Elevator::topPressed() { return this->topLimit.Get(); }
bool Elevator::botPressed() { return this->botLimit.Get(); }

void Elevator::setMotors(double powerPercent) {
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
    this->encoder.Reset();
    this->encoder.SetReverseDirection(false);
    this->lastTouchedBottom = true;
  } else if (this->topPressed()) {
    this->encoder.Reset();
    this->encoder.SetReverseDirection(true);
    this->lastTouchedBottom = false;
  }
  if (this->elevationController.AtSetpoint()) {
    this->elevationController.Reset();
  }
}