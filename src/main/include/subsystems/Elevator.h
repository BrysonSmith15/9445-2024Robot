// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/controller/PIDController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

// rev
#include <rev/CANSparkMax.h>

#include "Constants.h"

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc::PIDController elevationController{6e-5, 0.0, 0.0};
  double calcPID(double setpoint);
  bool topPressed();
  bool botPressed();

  void setMotors(double powerPercent);

 private:
  // the motorL will be the master and R will be the follower motor
  // both of these are CIMs
  rev::CANSparkMax motorL{18, rev::CANSparkLowLevel::MotorType::kBrushed};
  rev::CANSparkMax motorR{19, rev::CANSparkLowLevel::MotorType::kBrushed};

  frc::DigitalInput topLimit{0};
  frc::DigitalInput botLimit{1};

  rev::SparkRelativeEncoder encoder;
};
