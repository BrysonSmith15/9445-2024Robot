// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveToShooter.h"

MoveToShooter::MoveToShooter(Intake* intake, double speed)
    : intake{intake}, speed{speed} {
  // Register that this command requires the subsystem.
  AddRequirements(this->intake);
}

void MoveToShooter::Initialize() {}

void MoveToShooter::Execute() { this->intake->run(this->speed); }

bool MoveToShooter::IsFinished() { return false; }

void MoveToShooter::End(bool interrupeted) { this->intake->stop(); }