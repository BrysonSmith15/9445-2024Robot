// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>

#include <functional>
#include <utility>

#include "commands/DriveCommand.h"

DriveCommand::DriveCommand(
    Drivetrain* drivetrain,
    std::function<units::meters_per_second_t()> xTranslationSupplier,
    std::function<units::meters_per_second_t()> yTranslationSupplier,
    std::function<units::radians_per_second_t()> thetaSupplier)
    : drivetrain{drivetrain},
      xTranslation{xTranslationSupplier},
      yTranslation{yTranslationSupplier},
      theta{thetaSupplier} {
  // Register that this command requires the subsystem.
  AddRequirements(this->drivetrain);
}

void DriveCommand::Execute() {
  // assumes field oriented
  frc::SmartDashboard::PutNumber("XTranslation", this->xTranslation().value());
  frc::SmartDashboard::PutNumber("YTranslation", this->yTranslation().value());
  frc::SmartDashboard::PutNumber("Theta", this->theta().value());
  auto states = this->drivetrain->m_kinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          this->xTranslation(), this->yTranslation(), this->theta(),
          frc::Rotation2d{this->drivetrain->getGyroAngle()}));

  this->drivetrain->m_kinematics.DesaturateWheelSpeeds(
      &states, this->drivetrain->MAXSPEED);

  this->drivetrain->setStates(states);
}

void DriveCommand::End(bool interrupted) { this->drivetrain->Stop(); }