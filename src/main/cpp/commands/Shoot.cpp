// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"

Shoot::Shoot(Shooter* shooter) : m_shooter{shooter} {
  // Register that this command requires the subsystem.
  AddRequirements(shooter);
}

void Shoot::Initialize() {
  this->timer.Restart();
  this->m_shooter->setMotors(1.0);
}

// void Shoot::Execute() { this->m_shooter->setMotors(1.0); }

bool Shoot::IsFinished() { return this->timer.HasElapsed(10_s); }

void Shoot::End(bool interrupted) { this->m_shooter->setMotors(0); }