// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkLowLevel.h>
#include <units/frequency.h>
#include <units/time.h>

SwerveModule::SwerveModule(int driveMotorCANID, int turnMotorCANID,
                           int turnEncoderCANID, bool driveInverted,
                           bool turnInverted)
    : driveMotor(driveMotorCANID, rev::CANSparkLowLevel::MotorType::kBrushless),
      turnMotor(turnMotorCANID, rev::CANSparkLowLevel::MotorType::kBrushless),
      turnEncoder(turnEncoderCANID) {
  this->turningPIDController.EnableContinuousInput(-std::numbers::pi,
                                                   std::numbers::pi);
  this->resetDriveDistance();
  // could tune based on
  // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/C%2B%2B/Velocity%20PID%20Control/src/main/cpp/Robot.cpp
  this->driveEncoder.SetPositionConversionFactor(this->gearRatio);
  this->driveEncoder.SetVelocityConversionFactor(this->gearRatio);
  this->driveMotor.SetInverted(driveInverted);
  this->turnMotor.SetInverted(turnInverted);

  this->turnMotor.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 100);

  this->turnMotor.SetPeriodicFramePeriod(
      rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);
  this->turnEncoder.OptimizeBusUtilization(1_s);
  this->turnEncoder.GetAbsolutePosition().SetUpdateFrequency(100_Hz, 75_ms);
}

void SwerveModule::stop() {
  auto stopState = frc::SwerveModuleState{};
  stopState.angle = 0_rad;
  stopState.speed = 0_mps;
  this->setState(stopState);
}

units::angle::radian_t SwerveModule::getTurnAngle() {
  // this is normalized to [-pi, pi]
  return -this->turnEncoder.GetAbsolutePosition().GetValue();
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {this->getDriveDistance(), this->getTurnAngle()};
}

void SwerveModule::resetTurnAngle() { this->turnEncoder.SetPosition(0_rad); }

units::length::foot_t SwerveModule::getDriveDistance() {
  // (number of rotations) * (wheel circumfrence)
  return (this->driveEncoder.GetPosition() / this->driveEncoderResolution) *
         (2 * std::numbers::pi * this->kWheelRadius);
}

void SwerveModule::resetDriveDistance() { this->driveEncoder.SetPosition(0); }

units::velocity::feet_per_second_t SwerveModule::getDriveRate() {
  return -(this->driveEncoder.GetVelocity() / this->driveEncoderResolution) *
         (2 * std::numbers::pi * this->kWheelRadius) / 1_s;
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {this->getDriveRate(), frc::Rotation2d{this->getTurnAngle()}};
}

void SwerveModule::setState(const frc::SwerveModuleState &refState) {
  // ! turn > pi/2_rad
  const auto state = frc::SwerveModuleState::Optimize(
      refState, frc::Rotation2d(this->getTurnAngle()));
  /*
  // uses PID for Drive
  double driveOut = this->drivePIDController.Calculate(
      this->getDriveRate().value(),
      this->driveLimiter.Calculate(state.speed.value()));
  */
  // ! use PID for Drive
  double driveOut = state.speed.value();
  double turnOut = 0.0;
  if (std::abs(state.angle.Radians().value() - this->getTurnAngle().value()) >
      std::numbers::pi / 36) {
    turnOut = this->turningPIDController.Calculate(
        this->getTurnAngle().value(), state.angle.Radians().value());
  } else {
    this->turningPIDController.Calculate(this->getTurnAngle().value(),
                                         state.angle.Radians().value());
  }
  driveOut = driveOut > 1.0 ? 1.0 : driveOut;
  driveOut = driveOut < -1.0 ? -1.0 : driveOut;

  turnOut = turnOut > 1.0 ? 1.0 : turnOut;
  turnOut = turnOut < -1.0 ? -1.0 : turnOut;
  this->driveMotor.Set(driveOut);
  this->turnMotor.Set(turnOut);
}

/*
void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t{m_turningEncoder.GetDistance()});

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetRate(), state.speed.value());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetDistance()}, state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}
*/

void SwerveModule::setIdleMode(bool coast) {
  this->driveMotor.SetIdleMode(coast ? rev::CANSparkBase::IdleMode::kCoast
                                     : rev::CANSparkBase::IdleMode::kBrake);
}