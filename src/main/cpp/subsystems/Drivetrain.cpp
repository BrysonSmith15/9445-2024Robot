// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Drivetrain.h"

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
  return units::angle::degree_t{this->gyro.GetYaw()};
  // return 0_rad;
}

void Drivetrain::Periodic() {
  m_odometry.Update(this->getGyroAngle(),
                    {flModule.GetPosition(), frModule.GetPosition(),
                     blModule.GetPosition(), brModule.GetPosition()});
  frc::SmartDashboard::PutNumber("GyroAngle", this->getGyroAngle().value());
}
