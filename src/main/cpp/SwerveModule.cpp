// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkLowLevel.h>
#include <units/time.h>

#include "SwerveModule.h"

SwerveModule::SwerveModule(int driveMotorCANID, int turnMotorCANID,
                           int turnEncoderCANID)
    : driveMotor(driveMotorCANID, rev::CANSparkLowLevel::MotorType::kBrushless),
      turnMotor(turnMotorCANID, rev::CANSparkLowLevel::MotorType::kBrushless),
      turnEncoder(turnEncoderCANID) {
  this->turningPIDController.EnableContinuousInput(-std::numbers::pi * 1_rad,
                                                   std::numbers::pi * 1_rad);
  this->resetDriveDistance();
  this->drivePIDController.SetTolerance(10);
  this->drivePIDController.SetIZone(1.0);
  this->driveEncoder.SetPositionConversionFactor(this->gearRatio);
  this->driveEncoder.SetVelocityConversionFactor(this->gearRatio);
}

void SwerveModule::stop() {
  auto stopState = frc::SwerveModuleState{};
  stopState.angle = 0_rad;
  stopState.speed = 0_mps;
  this->setState(stopState);
}

units::angle::radian_t SwerveModule::getTurnAngle() {
  // this is normalized to [-pi, pi]
  return this->turnEncoder.GetAbsolutePosition().GetValue();
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
  return {this->getDriveRate(), this->getTurnAngle()};
}

void SwerveModule::setState(const frc::SwerveModuleState& refState) {
  // ! turn > pi/2_rad
  const auto state =
      frc::SwerveModuleState::Optimize(refState, this->getTurnAngle());
  auto driveOut = this->drivePIDController.Calculate(
      this->getDriveRate().value(), state.speed.value());
  // could do feedforward stuff later, but it is not implemented here.
  auto turnOut = this->turningPIDController.Calculate(this->getTurnAngle(),
                                                      state.angle.Radians());
  frc::SmartDashboard::PutNumber("PreDriveOut", driveOut);
  if (!this->drivePIDController.AtSetpoint()) {
    driveOut = driveOut > 1.0 ? 1.0 : driveOut;
    driveOut = driveOut < -1.0 ? -1.0 : driveOut;
    driveOut = this->driveLimiter.Calculate(driveOut);
  } else {
    driveOut = 0.0;
    this->drivePIDController.Reset();
  }
  frc::SmartDashboard::PutNumber("DriveOut", driveOut);
  frc::SmartDashboard::PutNumber("TurnOut", turnOut);
  frc::SmartDashboard::PutNumber("driveSetpoint", state.speed.value());
  frc::SmartDashboard::PutNumber("turnSetpoint", state.angle.Degrees().value());
  frc::SmartDashboard::PutNumber("driveRate", this->getDriveRate().value());
  frc::SmartDashboard::PutNumber("turnAngle", this->getTurnAngle().value());
  frc::SmartDashboard::PutNumber("Encoder Value",
                                 this->driveEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Encoder Velocity",
                                 this->driveEncoder.GetVelocity());
  this->driveMotor.Set(driveOut);
  // this->turnMotor.Set(turnOut);
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