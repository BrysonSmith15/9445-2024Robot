// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"

Shoot::Shoot(Shooter* shooter) : m_shooter{shooter} {
  // Register that this command requires the subsystem.
  AddRequirements(shooter);
}

void Shoot::Initialize() {}

void Shoot::Execute() { this->m_shooter->setMotors(-this->speed); }

bool Shoot::IsFinished() { return false; }

void Shoot::End(bool interrupted) { this->m_shooter->setMotors(0); }