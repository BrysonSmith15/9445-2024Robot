// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/PneumaticHub.h>
#include <frc/Solenoid.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/Trigger.h>

#include "Constants.h"
// subsystems
#include "LEDs.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
// commands
#include "commands/DriveCommand.h"
#include "commands/ElevatorToSetpoint.h"
#include "commands/MoveToShooter.h"
#include "commands/Shoot.h"

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
  double getXState();
  double getYState();
  double getThetaState();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandJoystick driverController{
      OperatorConstants::kDriverControllerPort};
  frc2::CommandJoystick secondController{
      OperatorConstants::kSecondControllerPort};

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
  // basically rate of change is limited by argument
  // just a fancy ramper
  /*
  frc::SlewRateLimiter<units::scalar> xLimiter{1 / 1_s};
  frc::SlewRateLimiter<units::scalar> yLimiter{1 / 1_s};
  */
  frc::SlewRateLimiter<units::scalar> thetaLimiter{std::numbers::pi / 4_s};
  frc::PIDController thetaController{3e-5, 0.0, 0.0};
  units::radian_t prevTheta = 0_rad;
  // The robot's subsystems are defined here...
  Drivetrain drivetrain;
  /*  Elevator elevator;
    Intake intake;
    Shooter shooter;
    */
  // the LED strip is here
  // LEDs led{0, 0, 299};
  void ConfigureBindings();
};