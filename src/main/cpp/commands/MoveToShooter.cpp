// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveToShooter.h"

MoveToShooter::MoveToShooter(Intake* intake) : intake{intake} {
  // Register that this command requires the subsystem.
  AddRequirements(this->intake);
}

void MoveToShooter::Initialize() {
  this->timer.Restart();
  this->intake->run(true);
}

// TODO: Test if an execute function is needed or if a CANSparkMAX.Set() call is
// set and forget
// void MoveToShooter::Execute() { this->intake->run(true); }

bool MoveToShooter::IsFinished() { return this->timer.HasElapsed(2_s); }

void MoveToShooter::End(bool interrupeted) { this->intake->stop(); }