// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include "Constants.h"

// commands
#include "commands/Autos.h"
#include "commands/DriveCommand.h"
#include "commands/DriveDistance.h"
#include "commands/ElevatorManual.h"
#include "commands/ElevatorToBottom.h"
#include "commands/ElevatorToSetpoint.h"
#include "commands/ElevatorToTop.h"
#include "commands/LEDChase.h"
#include "commands/LEDSet.h"
// nt
#include <networktables/DoubleTopic.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  // not really a thread; just publishes the video
  VisionThread();
  this->drivetrain.SetDefaultCommand(DriveCommand(
      &drivetrain, [this] { return this->getXState(); },
      [this] { return this->getYState(); },
      [this] { return this->getThetaState(); },
      [this] { return this->driverController.GetRawAxis(3) < 0.25; }));

  // if (this->driverController.Button(1).Get()) {
  // this->drivetrain.resetYaw();
  // }
  // this->elevator.SetDefaultCommand(ElevatorToSetpoint(&elevator));
  // set the leds all to the BSHS orange color (from their website)
  // this->led.SetDefaultCommand(LEDSet(&this->led, 255, 50, 0));
  this->led.SetDefaultCommand(
      LEDChase(&this->led, 255, 50, 0, 150, 150, 150, 40));
  // this->elevator.SetDefaultCommand(
  // ElevatorManual(&this->elevator, 0.0).ToPtr());
  // this->intake.SetDefaultCommand(MoveToShooter(&this->intake, 0.0).ToPtr());
}

double RobotContainer::getXState() {
  // return this->xLimiter.Calculate(
  // frc::ApplyDeadband(this->driverController.GetX(), 0.1));
  return frc::ApplyDeadband(this->driverController.GetX(), 0.1);
}
double RobotContainer::getYState() {
  // return this->yLimiter.Calculate(
  // -frc::ApplyDeadband(this->driverController.GetY(), 0.1));
  return -frc::ApplyDeadband(this->driverController.GetY(), 0.1);
}

double RobotContainer::getThetaState() {
  return -frc::ApplyDeadband(this->driverController.GetTwist(), 0.1);
}

void RobotContainer::ConfigureBindings() {
  // Logitech Controller on XInput
  this->driverController.SetXChannel(1);
  this->driverController.SetYChannel(0);
  this->driverController.SetZChannel(4);
  this->driverController.SetTwistChannel(5);
  // shooter to top
  frc2::Trigger([this] {
    return this->secondController.GetPOV() ==
           BindingConstants::elevatorManualUpAngle;
  })
      .WhileTrue(
          ElevatorManual(&this->elevator, ElevatorConstants::speed).ToPtr());
  // shooter to bottom
  frc2::Trigger([this] {
    return this->secondController.GetPOV() ==
           BindingConstants::elevatorManualDownAngle;
  });
  frc2::JoystickButton(&this->secondController,
                       BindingConstants::elevatorUpButton)
      .OnTrue(ElevatorToTop(&this->elevator).ToPtr());
  frc2::JoystickButton(&this->secondController,
                       BindingConstants::elevatorDownButton)
      .OnTrue(ElevatorToBottom(&this->elevator).ToPtr());
  frc2::JoystickButton(&this->secondController, BindingConstants::intakeButton)
      .WhileTrue(MoveToShooter(&this->intake, 0.1).ToPtr());
  frc2::JoystickButton(&this->secondController,
                       BindingConstants::intakeReverseButton)
      .WhileTrue(MoveToShooter(&this->intake, -0.1).ToPtr());
  frc2::Trigger([this] {
    return this->secondController.GetRawAxis(
               BindingConstants::moveToShooterAxis) > 0.85;
  }).WhileTrue(MoveToShooter(&this->intake, 1.0).ToPtr());
  frc2::Trigger([this] {
    return this->secondController.GetRawAxis(
               BindingConstants::moveToShooterAxis) > 0.15;
  }).WhileTrue(Shoot(&this->shooter).ToPtr());

  // brake while enabled and coast while disabled
  frc2::Trigger([this] { return frc::DriverStation::IsEnabled(); })
      .OnTrue(this->drivetrain.setIdleMode(false))
      .OnTrue(LEDChase(&this->led, 255, 255, 255,
                       frc::DriverStation::GetAlliance() ==
                               frc::DriverStation::Alliance::kRed
                           ? 255
                           : 0,
                       0,
                       frc::DriverStation::GetAlliance() ==
                               frc::DriverStation::Alliance::kBlue
                           ? 255
                           : 0,
                       40)
                  .ToPtr())
      .OnFalse(this->drivetrain.setIdleMode(true));
  frc2::Trigger([this] {
    return this->driverController.GetRawButton(5);
  }).OnTrue(this->drivetrain.resetYaw());
}
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return ElevatorToTop(&this->elevator)
      .ToPtr()
      .WithTimeout(1_s)
      .AndThen(frc2::WaitCommand(1_s).ToPtr())
      .AndThen(MoveToShooter(&this->intake, -0.1).ToPtr().WithTimeout(1_s))
      .AndThen(frc2::WaitCommand(1_s).ToPtr())
      .AndThen(
          Shoot(&this->shooter)
              .ToPtr()
              .WithTimeout(3_s)
              .AndThen(frc2::WaitCommand(1_s).ToPtr())
              .AlongWith(
                  MoveToShooter(&this->intake, 1.0).ToPtr().WithTimeout(3_s)))
      .AndThen(DriveDistance(&this->drivetrain, -6.0_ft).ToPtr())
      .WithTimeout(1_s);
}

void RobotContainer::VisionThread() {
  cs::UsbCamera outCamera = frc::CameraServer::StartAutomaticCapture(0);
  cs::CvSource outStream = frc::CameraServer::PutVideo("front", 640, 480);
}