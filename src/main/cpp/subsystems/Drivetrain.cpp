// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include <frc/smartdashboard/SmartDashboard.h>

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

units::angle::radian_t Drivetrain::getGyroAngle() {
  // return units::angle::degree_t{this->gyro.GetYaw()};
  return 0_rad;
}

frc2::CommandPtr Drivetrain::resetYaw(units::degree_t newOff) {
  return this->RunOnce([this, &newOff] { this->angleOff = newOff; });
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
  frc::SmartDashboard::PutNumber("GyroAngle", this->getGyroAngle().value());
}
