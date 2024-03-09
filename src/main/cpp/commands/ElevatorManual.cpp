// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorManual.h"

ElevatorManual::ElevatorManual(Elevator* elevator, double power)
    : elevator{elevator}, power{power} {
  // Register that this command requires the subsystem.
  AddRequirements(elevator);
}

void ElevatorManual::Initialize() { this->elevator->setMotors(this->power); }

bool ElevatorManual::IsFinished() {
  return (this->elevator->topPressed() && this->power > 0) ||
         (this->elevator->botPressed() && this->power < 0);
}

void ElevatorManual::End(bool interrupted) { this->elevator->setMotors(0.0); }