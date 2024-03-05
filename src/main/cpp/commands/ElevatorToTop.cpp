// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorToTop.h"

#include <iostream>

ElevatorToTop::ElevatorToTop(Elevator* elevator) : elevator{elevator} {
  // Register that this command requires the subsystem.
  AddRequirements(this->elevator);
}

void ElevatorToTop::Initialize() {
  this->elevator->setMotors(-ElevatorConstants::speed);
  std::cout << "GoingUp\n";
}

void ElevatorToTop::Execute() {
  this->elevator->setMotors(-ElevatorConstants::speed);
}

bool ElevatorToTop::IsFinished() {
  std::cout << this->elevator->topPressed();
  return this->elevator->topPressed();
}

void ElevatorToTop::End(bool _) { this->elevator->setMotors(0); }
