// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>

// frc
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

// NavX
#include <AHRS.h>

// units
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>

// custom
#include "SwerveModule.h"

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();
  /**
   * Example command factory method.
   */
  frc2::CommandPtr Stop();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void setStates(wpi::array<frc::SwerveModuleState, 4U> states);
  void setStates(frc::SwerveModuleState fl, frc::SwerveModuleState fr,
                 frc::SwerveModuleState bl, frc::SwerveModuleState br);

  units::angle::radian_t getGyroAngle();
  void resetYaw();
  units::velocity::meters_per_second_t MAXSPEED = 1_mps;
  units::angular_velocity::radians_per_second_t MAXROT =
      (std::numbers::pi * 1_rad) / 2_s;

  frc::SwerveDriveKinematics<4> m_kinematics{flLocation, frLocation, blLocation,
                                             brLocation};

 private:
  AHRS gyro{frc::SerialPort::kUSB1};

  const units::length::foot_t forwardDist = 10.5_in;
  const units::length::foot_t horizontalDist = 10.5_in;

  frc::Translation2d flLocation{forwardDist, forwardDist};
  frc::Translation2d frLocation{forwardDist, -forwardDist};
  frc::Translation2d blLocation{-forwardDist, forwardDist};
  frc::Translation2d brLocation{-forwardDist, -forwardDist};

  const bool leftInverted = false;
  // drive, turnMotor, turnEncoder, driveInverted, turnInverted
  SwerveModule flModule{14, 12, 13, leftInverted, true};
  SwerveModule frModule{11, 9, 10, !leftInverted, !leftInverted};
  SwerveModule blModule{17, 15, 16, leftInverted, true};
  SwerveModule brModule{8, 6, 7, !leftInverted, !leftInverted};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      this->getGyroAngle(),
      {flModule.GetPosition(), frModule.GetPosition(), blModule.GetPosition(),
       brModule.GetPosition()}};
};
