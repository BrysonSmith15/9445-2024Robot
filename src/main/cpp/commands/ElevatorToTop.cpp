// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorToTop.h"

ElevatorToTop::ElevatorToTop(Elevator* elevator) : elevator{elevator} {
  // Register that this command requires the subsystem.
  AddRequirements(this->elevator);
}

void ElevatorToTop::Initialize() {}

void ElevatorToTop::Execute() {
  if (this->elevator->getTicks() < ElevatorConstants::stableTicks) {
    this->elevator->setMotors(-ElevatorConstants::speed);
  } else {
    this->elevator->setMotors(-ElevatorConstants::stableUpSpeed);
  }
  if (this->elevator->topPressed()) {
    this->elevator->setMotors(-ElevatorConstants::speed / 4);
  }
}

bool ElevatorToTop::IsFinished() { return false; }

void ElevatorToTop::End(bool _) { this->elevator->setMotors(0); }
