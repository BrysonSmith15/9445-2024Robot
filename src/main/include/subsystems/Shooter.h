// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
// rev
#include <rev/CANSparkMax.h>
#include <units/dimensionless.h>

#include "Constants.h"

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void setMotors(double percentPower);
  double getMotorOut();
  units::second_t secondsToFull = 0.25_s;

 private:
  rev::CANSparkMax motorL1{21, rev::CANSparkLowLevel::MotorType::kBrushed};
  rev::CANSparkMax motorR1{30, rev::CANSparkLowLevel::MotorType::kBrushed};
  // slow down motor acceleration a little for motor safety
  frc::SlewRateLimiter<units::scalar> speedLimiter{1 / secondsToFull};
};
