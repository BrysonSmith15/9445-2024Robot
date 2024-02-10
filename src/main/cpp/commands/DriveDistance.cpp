// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/kinematics/SwerveModuleState.h>

#include "commands/DriveDistance.h"

DriveDistance::DriveDistance(Drivetrain* drivetrain, units::foot_t distance)
    : drivetrain{drivetrain}, distance{distance} {
  // Register that this command requires the subsystem.
  AddRequirements(this->drivetrain);
  this->speedController.SetTolerance(this->tolerance.value());
}

void DriveDistance::Initialize() {
  // -gyroAngle for field oriented
  this->desiredState.angle = -this->drivetrain->getGyroAngle();
  this->desiredState.speed = 0_ft / 1_s;
  this->drivetrain->setStates(desiredState, desiredState, desiredState,
                              desiredState);
}

void DriveDistance::Execute() {
  this->desiredState.speed =
      this->speedController.Calculate(this->drivetrain->getDistance().value(),
                                      this->distance.value()) *
      1_ft / 1_s;
  this->drivetrain->setStates(desiredState, desiredState, desiredState,
                              desiredState);
}

bool DriveDistance::IsFinished() { return this->speedController.AtSetpoint(); }

void DriveDistance::End(bool interrupted) {
  this->desiredState.speed = 0_ft / 1_s;
  this->desiredState.angle = 0_rad;
  this->drivetrain->setStates(desiredState, desiredState, desiredState,
                              desiredState);
}