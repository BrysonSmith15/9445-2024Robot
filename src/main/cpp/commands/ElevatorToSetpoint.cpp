// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorToSetpoint.h"

#include <frc/smartdashboard/SmartDashboard.h>

ElevatorToSetpoint::ElevatorToSetpoint(
    Elevator* elevator, ElevatorConstants::setpointOptions setpoint)
    : m_elevator{elevator} {
  // Register that this command requires the subsystem.
  AddRequirements(m_elevator);
  switch (setpoint) {
    case ElevatorConstants::bottom:
      this->m_setpoint = ElevatorConstants::bottomTicks.value();
      break;
    case ElevatorConstants::amp:
      this->m_setpoint = ElevatorConstants::ampTicks.value();
      break;
    case ElevatorConstants::source:
      this->m_setpoint = ElevatorConstants::sourceTicks.value();
      break;
    case ElevatorConstants::speaker:
      this->m_setpoint = ElevatorConstants::speakerTicks.value();
      break;
    case ElevatorConstants::climb:
      this->m_setpoint = ElevatorConstants::climbTicks.value();
      break;
  }
}

void ElevatorToSetpoint::Execute() {
  double newVal = this->m_elevator->calcPID(this->m_setpoint);
  this->m_elevator->setMotors(newVal);
  this->goingUp = newVal > 0;
}

bool ElevatorToSetpoint::IsFinished() {
  return this->m_elevator->elevationController.AtSetpoint() ||
         (this->goingUp ? this->m_elevator->topPressed()
                        : this->m_elevator->botPressed());
}

void ElevatorToSetpoint::End(bool interrupted) {
  this->m_elevator->setMotors(0.0);
}
