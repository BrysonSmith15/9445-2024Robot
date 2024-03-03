// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorToBottom.h"

ElevatorToBottom::ElevatorToBottom(Elevator* elevator) : elevator{elevator} {
  // Register that this command requires the subsystem.
  AddRequirements(this->elevator);
}

void ElevatorToBottom::Initialize() { this->elevator->setMotors(-.3); }

bool ElevatorToBottom::IsFinished() { return this->elevator->botPressed(); }

void ElevatorToBottom::End(bool _) { this->elevator->setMotors(0); }
