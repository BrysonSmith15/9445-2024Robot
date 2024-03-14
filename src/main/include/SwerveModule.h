// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// frc
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveModulePosition.h>

// units
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/velocity.h>

// rev
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>

// ctre
#include <ctre/phoenix6/CANcoder.hpp>

class SwerveModule {
 public:
  SwerveModule(int driveMotorCANID, int turnMotorCANID, int turningEncoderCANID,
               bool driveInverted, bool turnInverted);
  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();
  void setState(const frc::SwerveModuleState& state);
  units::angle::radian_t getTurnAngle();
  units::length::foot_t getDriveDistance();
  units::velocity::feet_per_second_t getDriveRate();
  void setIdleMode(bool coast);
  void resetTurnAngle();
  void resetDriveDistance();
  void stop();

 private:
  static constexpr units::length::meter_t kWheelRadius = 2_in;
  static constexpr int turnEncoderResolution = 4096;
  static constexpr int driveEncoderResolution = 42;
  static constexpr float gearRatio = 1.0 / 8.14;

  static constexpr auto kModuleMaxAngularVelocity =
      std::numbers::pi * (1_rad / 2_s);  // radians per second
                                         // 1/4 rot/sec max speed
  static constexpr auto kModuleMaxAngularAcceleration =
      std::numbers::pi * 2_rad / 1_s /
      1_s;  // 2 radians per second^2 = full accel in 1/2 sec (?)

  rev::CANSparkMax turnMotor;
  rev::CANSparkMax driveMotor;

  ctre::phoenix6::hardware::CANcoder turnEncoder;
  rev::SparkRelativeEncoder driveEncoder = this->driveMotor.GetEncoder(
      rev::SparkRelativeEncoder::Type::kHallSensor, driveEncoderResolution);

  frc::SlewRateLimiter<units::scalar> driveLimiter{1 / 2_s, -5 / 1_s};

  frc::PIDController turningPIDController{6e-1, 0.0, 0.0};
  units::radian_t prevTheta = 0_rad;
  // the feedforward controllers have been removed
  // because they are going to be a pain to tune
  // and potentially not worth it at all.
};