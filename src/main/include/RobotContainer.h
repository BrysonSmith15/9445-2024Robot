// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cameraserver/CameraServer.h>
#include <frc/PneumaticHub.h>
#include <frc/Solenoid.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/Trigger.h>

#include <thread>

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
// vision
#include <cameraserver/CameraServer.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

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

  frc2::CommandPtr GetAutonomousCommand();
  double getXState();
  double getYState();
  double getThetaState();

 private:
  int sourceCenterId = 7;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandJoystick driverController{
      OperatorConstants::kDriverControllerPort};
  frc2::CommandJoystick secondController{
      OperatorConstants::kSecondControllerPort};

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
  // basically rate of change is limited by argument
  // just a fancy ramper
  frc::SlewRateLimiter<units::scalar> xLimiter{1 / 1_s, -3 / 1_s};
  frc::SlewRateLimiter<units::scalar> yLimiter{1 / 1_s, -3 / 1_s};
  frc::SlewRateLimiter<units::scalar> thetaLimiter{std::numbers::pi / 4_s};
  frc::PIDController thetaController{5e-5, 0.0, 0.0};
  units::radian_t prevTheta = 0_rad;
  // The robot's subsystems are defined here...
  Drivetrain drivetrain;
  Elevator elevator;
  Intake intake;
  Shooter shooter;
  // the LED strip is here
  LEDs led{9, 80};
  void ConfigureBindings();

  static void VisionThread();
};