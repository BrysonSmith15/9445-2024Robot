// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
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
  this->thetaController.EnableContinuousInput(-1.0, 1.0);

  // if (this->driverController.Button(1).Get()) {
  // this->drivetrain.resetYaw();
  // }
  // this->elevator.SetDefaultCommand(ElevatorToSetpoint(&elevator));
  // set the leds all to the BSHS orange color (from their website)
  // this->led.set(0, 299, 216, 80, 36);
}

double RobotContainer::getXState() {
  /*
  return this->xLimiter
             .Calculate(frc::ApplyDeadband(this->driverController.GetX(),
  0.1)) .value() * this->drivetrain.MAXSPEED;
         */
  return frc::ApplyDeadband(this->driverController.GetX(), 0.1);
}
double RobotContainer::getYState() {
  /*
  return this->yLimiter
             .Calculate(frc::ApplyDeadband(this->driverController.GetY(), 0.1))
             .value() *
         this->drivetrain.MAXSPEED;
         */
  return -frc::ApplyDeadband(this->driverController.GetY(), 0.1);
}

double RobotContainer::getThetaState() {
  if (OperatorConstants::usingFieldOrientedTurn) {
    // turn to the same angle on the field as the right joystick is pointed at
    // no reset for thetaController because it does not have an integral term
    units::angle::radian_t desired = units::radian_t{
        std::atan2(frc::ApplyDeadband(this->driverController.GetTwist(), 0.1),
                   frc::ApplyDeadband(this->driverController.GetZ(), 0.1))};
    frc::SmartDashboard::PutNumber("Desired Rotation",
                                   units::degree_t{desired}.value());
    // if magnitude is less than 0.3, keep prev theta
    if (std::sqrt(
            this->driverController.GetX() * this->driverController.GetX() +
            this->driverController.GetY() * this->driverController.GetY()) <
        0.3) {
      desired = this->prevTheta;
    }
    double out = this->thetaController.Calculate(
        this->drivetrain.getGyroAngle().value() / std::numbers::pi,
        desired.value() / std::numbers::pi);
    out = out > 1.0 ? 1.0 : out;
    out = out < -1.0 ? -1.0 : out;
    return out;
  } else {
    // below is just turn based on how much driver says
    /*
        return this->thetaLimiter
                   .Calculate(
                       frc::ApplyDeadband(this->driverController.GetZ(), 0.1))
                   .value() *
               this->drivetrain.MAXROT;
               */
    return this->thetaLimiter.Calculate(
        frc::ApplyDeadband(this->driverController.GetZ(), 0.1));
  }
}

void RobotContainer::ConfigureBindings() {
  // Logitech Controller on XInput
  this->driverController.SetXChannel(1);
  this->driverController.SetYChannel(0);
  this->driverController.SetZChannel(4);
  this->driverController.SetTwistChannel(5);

  // Move the note to the shooter
  frc2::Trigger([this] {
    return this->secondController.GetRawButton(
        BindingConstants::moveToShooterButton);
  }).OnTrue(MoveToShooter(&this->intake).ToPtr());
  // Make the shooter run
  frc2::Trigger([this] {
    return this->secondController.GetRawButton(BindingConstants::shootButton);
  }).OnTrue(Shoot(&this->shooter).ToPtr());
  // spin the
  frc2::Trigger([this] {
    return this->secondController.GetRawButton(
        BindingConstants::shootCompositionButton);
  })
      .OnTrue(
          Shoot(&this->shooter)
              .ToPtr()
              .AlongWith(frc2::WaitCommand(this->shooter.secondsToFull + 0.5_s)
                             .ToPtr())
              .AndThen(MoveToShooter(&this->intake).ToPtr()));
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
