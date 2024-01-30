// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>

#include "Constants.h"
#include "RobotContainer.h"
// commands
#include "commands/Autos.h"
#include "commands/DriveCommand.h"
#include "commands/ElevatorToSetpoint.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  this->drivetrain.SetDefaultCommand(DriveCommand(
      &drivetrain, [this] { return this->getXState(); },
      [this] { return this->getYState(); },
      [this] { return this->getThetaState(); }));
  // this->elevator.SetDefaultCommand(ElevatorToSetpoint(&elevator));
  // set the leds all to the BSHS orange color (from their website)
  // this->led.set(0, 299, 216, 80, 36);
  // tmp jank
  this->flEncoder.Set(true);
  this->frEncoder.Set(true);
  this->blEncoder.Set(true);
  this->brEncoder.Set(true);
}

units::velocity::meters_per_second_t RobotContainer::getXState() {
  /*
  return this->xLimiter
             .Calculate(frc::ApplyDeadband(this->driverController.GetX(),
  0.1)) .value() * this->drivetrain.MAXSPEED;
         */
  return frc::ApplyDeadband(this->driverController.GetX(), 0.1) *
         this->drivetrain.MAXSPEED;
}
units::velocity::meters_per_second_t RobotContainer::getYState() {
  /*
  return this->yLimiter
             .Calculate(frc::ApplyDeadband(this->driverController.GetY(), 0.1))
             .value() *
         this->drivetrain.MAXSPEED;
         */
  return frc::ApplyDeadband(this->driverController.GetY(), 0.1) *
         this->drivetrain.MAXSPEED;
}
units::angular_velocity::radians_per_second_t RobotContainer::getThetaState() {
  if (OperatorConstants::usingFieldOrientedTurn) {
    // find out how much we plan to turn
    units::angle::radian_t desired{
        -atan2(frc::ApplyDeadband(this->driverController.GetTwist(), 0.1),
               frc::ApplyDeadband(this->driverController.GetZ(), 0.1))};
    units::angle::radian_t actual{this->drivetrain.getGyroAngle()};

    // if the magnitude of the joystick is <= 0.3, return 0.
    if (sqrt(pow(this->driverController.GetTwist(), 2) +
             pow(this->driverController.GetZ(), 2)) <= 0.3) {
      return 0_rad_per_s;
    }

    // math below this point comes from
    // https://www.desmos.com/calculator/vkoc1gtneh
    // -(desired - actual)
    units::angle::radian_t delta = actual - desired;
    // if within 7.5_deg of setpoint, just leave it
    delta = frc::ApplyDeadband(delta.value(), std::numbers::pi / 24) * 1_rad;
    // Optomize angle
    if (abs(delta.value()) > std::numbers::pi) {
      delta = 2_rad * std::numbers::pi - abs(delta.value()) * 1_rad;
    }
    delta *= -1;
    // this returns a field oriented theta based on which direction the driver
    // points the right joystick
    /*
    return -this->thetaLimiter.Calculate(delta / (std::numbers::pi * 1_rad))
                .value() *
           this->drivetrain.MAXROT;
           */
    return delta / (std::numbers::pi * 1_rad) * this->drivetrain.MAXROT;
  } else {
    // below is just turn based on how much driver says
    /*
        return this->thetaLimiter
                   .Calculate(
                       frc::ApplyDeadband(this->driverController.GetZ(), 0.1))
                   .value() *
               this->drivetrain.MAXROT;
               */
    return frc::ApplyDeadband(this->driverController.GetZ(), 0.1) *
           this->drivetrain.MAXROT;
  }
}

void RobotContainer::ConfigureBindings() {
  // Logitech Controller on XInput
  this->driverController.SetXChannel(0);
  this->driverController.SetYChannel(1);
  this->driverController.SetZChannel(5);
  this->driverController.SetTwistChannel(4);
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  /*
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());
*/
  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}
/*
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return;
}
*/
