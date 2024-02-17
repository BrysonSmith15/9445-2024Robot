// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <units/dimensionless.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void run(bool forward);
  void stop();

 private:
  rev::CANSparkMax motor{24, rev::CANSparkLowLevel::MotorType::kBrushed};
  frc::SlewRateLimiter<units::scalar> limiter{1 / 1_s};
  frc::DigitalInput limit{3};
};
