// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommand.h"

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>

#include <functional>
#include <utility>

#include "Constants.h"

DriveCommand::DriveCommand(Drivetrain* drivetrain,
                           std::function<double()> xTranslationSupplier,
                           std::function<double()> yTranslationSupplier,
                           std::function<double()> thetaSupplier)
    : drivetrain{drivetrain},
      xTranslation{xTranslationSupplier},
      yTranslation{yTranslationSupplier},
      theta{thetaSupplier} {
  // Register that this command requires the subsystem.
  AddRequirements(this->drivetrain);
}

void DriveCommand::Execute() {
  // mps or rad_ps -> percent
  double x = this->xTranslation();
  double y = this->yTranslation();
  double t = this->theta();

  frc::Translation2d tVector =
      this->rotateByAngle(x, y, this->drivetrain->getGyroAngle());
  x = tVector.X().value();
  y = -tVector.Y().value();

  double a =
      x - t * (DrivetrainConstants::length / DrivetrainConstants::diagonal);
  double b =
      x + t * (DrivetrainConstants::length / DrivetrainConstants::diagonal);
  double c =
      y - t * (DrivetrainConstants::width / DrivetrainConstants::diagonal);
  double d =
      y + t * (DrivetrainConstants::width / DrivetrainConstants::diagonal);
  units::meters_per_second_t flSpeed =
      std::sqrt((b * b) + (c * c)) * this->drivetrain->MAXSPEED;
  units::meters_per_second_t frSpeed =
      std::sqrt((b * b) + (d * d)) * this->drivetrain->MAXSPEED;
  units::meters_per_second_t blSpeed =
      std::sqrt((a * a) + (c * c)) * this->drivetrain->MAXSPEED;
  units::meters_per_second_t brSpeed =
      std::sqrt((a * a) + (d * d)) * this->drivetrain->MAXSPEED;

  units::radian_t flAngle = units::radian_t{std::atan2(b, c)};
  units::radian_t frAngle = units::radian_t{std::atan2(b, d)};
  units::radian_t blAngle = units::radian_t{std::atan2(a, c)};
  units::radian_t brAngle = units::radian_t{std::atan2(a, d)};

  frc::SwerveModuleState flState;
  frc::SwerveModuleState frState;
  frc::SwerveModuleState blState;
  frc::SwerveModuleState brState;

  frc::SwerveModuleState* states[4] = {&flState, &frState, &blState, &brState};
  units::meters_per_second_t speeds[4] = {flSpeed, frSpeed, blSpeed, brSpeed};
  units::radian_t angles[4] = {flAngle, frAngle, blAngle, brAngle};

  for (int i = 0; i < 4; i++) {
    states[i]->speed = -speeds[i];
    states[i]->angle = -angles[i];
  }
  /*
  auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      this->xTranslation() * this->drivetrain->MAXSPEED,
      this->yTranslation() * this->drivetrain->MAXSPEED,
      units::angular_velocity::radians_per_second_t{this->theta() *
                                                    this->drivetrain->MAXROT},
      frc::Rotation2d(this->drivetrain->getGyroAngle()));
  auto moduleStates =
      this->drivetrain->m_kinematics.ToSwerveModuleStates(speeds);
  this->drivetrain->m_kinematics.DesaturateWheelSpeeds(&moduleStates, 5.0_mps);
  */
  this->drivetrain->setStates(flState, frState, blState, brState);
}

void DriveCommand::End(bool interrupted) { this->drivetrain->Stop(); }

frc::Translation2d DriveCommand::rotateByAngle(double x, double y,
                                               units::radian_t deltaTheta) {
  // based on https://www.desmos.com/calculator/mumuwjpmdv
  units::radian_t theta = units::radian_t{std::atan2(y, x)} - deltaTheta;
  double xP = std::sin(theta.value());
  double yP = std::cos(theta.value());
  units::meter_t magnitude = units::meter_t{std::sqrt((y * y) + (x * x))};
  return frc::Translation2d{xP * magnitude, yP * magnitude};
}