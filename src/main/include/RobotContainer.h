// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/PneumaticHub.h>
#include <frc/Solenoid.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>

#include "Constants.h"
#include "LEDs.h"
#include "subsystems/Drivetrain.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  // frc2::CommandPtr GetAutonomousCommand();
  units::velocity::meters_per_second_t getXState();
  units::velocity::meters_per_second_t getYState();
  units::angular_velocity::radians_per_second_t getThetaState();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandJoystick driverController{
      OperatorConstants::kDriverControllerPort};

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
  // basically rate of change is limited by argument
  // just a fancy ramper
  /*
  frc::SlewRateLimiter<units::scalar> xLimiter{1 / 1_s};
  frc::SlewRateLimiter<units::scalar> yLimiter{1 / 1_s};
  frc::SlewRateLimiter<units::scalar> thetaLimiter{1 / 1_s};
*/
  // The robot's subsystems are defined here...
  Drivetrain drivetrain;
  // the LED strip is here
  LEDs led{0, 300};
  void ConfigureBindings();

  // janky CANCoder power as a solenoid
  frc::PneumaticHub pneuHub{2};
  // TODO: Change the solenoid ports
  frc::Solenoid flEncoder = this->pneuHub.MakeSolenoid(9);
  frc::Solenoid frEncoder = this->pneuHub.MakeSolenoid(8);
  frc::Solenoid blEncoder = this->pneuHub.MakeSolenoid(0);
  frc::Solenoid brEncoder = this->pneuHub.MakeSolenoid(1);
};
