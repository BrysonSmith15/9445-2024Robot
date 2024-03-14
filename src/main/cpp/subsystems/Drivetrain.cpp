// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

Drivetrain::Drivetrain() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr Drivetrain::Stop() {
  return this->RunOnce([this] {
    flModule.stop();
    frModule.stop();
    blModule.stop();
    brModule.stop();
  });
}

void Drivetrain::setStates(wpi::array<frc::SwerveModuleState, 4U> states) {
  auto [fl, fr, bl, br] = states;
  flModule.setState(fl);
  frModule.setState(fr);
  blModule.setState(bl);
  brModule.setState(br);
}

void Drivetrain::setStates(frc::SwerveModuleState fl, frc::SwerveModuleState fr,
                           frc::SwerveModuleState bl,
                           frc::SwerveModuleState br) {
  flModule.setState(fl);
  frModule.setState(fr);
  blModule.setState(bl);
  brModule.setState(br);
}

frc2::CommandPtr Drivetrain::setIdleMode(bool coast) {
  return this
      ->RunOnce([this, coast] {
        this->flModule.setIdleMode(coast);
        this->frModule.setIdleMode(coast);
        this->blModule.setIdleMode(coast);
        this->brModule.setIdleMode(coast);
      })
      .IgnoringDisable(true);
}

units::angle::radian_t Drivetrain::getGyroAngle() {
  return units::angle::degree_t{this->gyro.GetYaw()};
  // return 0_rad;
}

frc2::CommandPtr Drivetrain::resetYaw() {
  std::cout << "Reset\n";
  return this->RunOnce([this] { this->gyro.Reset(); })
      .IgnoringDisable(true)
      .WithInterruptBehavior(
          frc2::Command::InterruptionBehavior::kCancelIncoming);
}

void Drivetrain::resetDistance() {
  // do not really need to do all four since only one is read, but it is okay
  this->flModule.resetDriveDistance();
  this->frModule.resetDriveDistance();
  this->blModule.resetDriveDistance();
  this->brModule.resetDriveDistance();
}

units::foot_t Drivetrain::getDistance() {
  // technically maybe bad, but could be worse
  return this->flModule.getDriveDistance();
}

void Drivetrain::Periodic() {
  m_odometry.Update(this->getGyroAngle(),
                    {flModule.GetPosition(), frModule.GetPosition(),
                     blModule.GetPosition(), brModule.GetPosition()});
  frc::SmartDashboard::PutNumber("Angle",
                                 units::degree_t{this->getGyroAngle()}.value());
}
