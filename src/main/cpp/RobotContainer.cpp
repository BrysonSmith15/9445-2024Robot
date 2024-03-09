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
  // VisionThread();
  this->drivetrain.SetDefaultCommand(DriveCommand(
      &drivetrain, [this] { return this->getXState(); },
      [this] { return this->getYState(); },
      [this] { return this->getThetaState(); }));
  this->thetaController.EnableContinuousInput(-180, 180);
  this->thetaController.SetTolerance(30);

  // if (this->driverController.Button(1).Get()) {
  // this->drivetrain.resetYaw();
  // }
  // this->elevator.SetDefaultCommand(ElevatorToSetpoint(&elevator));
  // set the leds all to the BSHS orange color (from their website)
  // this->led.SetDefaultCommand(LEDSet(&this->led, 255, 50, 0));
  // this->led.SetDefaultCommand(
  // LEDChase(&this->led, 255, 50, 0, 150, 150, 150, 40));
  // this->elevator.SetDefaultCommand(
  // ElevatorManual(&this->elevator, 0.0).ToPtr());
  // this->intake.SetDefaultCommand(MoveToShooter(&this->intake, 0.0).ToPtr());
}

double RobotContainer::getXState() {
  return this->xLimiter.Calculate(
      frc::ApplyDeadband(this->driverController.GetX(), 0.1));
}
double RobotContainer::getYState() {
  return this->yLimiter.Calculate(
      -frc::ApplyDeadband(this->driverController.GetY(), 0.1));
}

double RobotContainer::getThetaState() {
  if (this->driverController.GetRawAxis(BindingConstants::trackSpeakerAxis) <
      0.3) {
    if (OperatorConstants::usingFieldOrientedTurn) {
      // turn to the same angle on the field as the right joystick is pointed at
      // no reset for thetaController because it does not have an integral term
      units::angle::radian_t desired = units::radian_t{std::atan2(
          frc::ApplyDeadband(-this->driverController.GetZ(), 0.1),
          frc::ApplyDeadband(-this->driverController.GetTwist(), 0.1))};
      // if magnitude is less than 0.3, keep prev theta
      if (std::sqrt(
              (this->driverController.GetZ() * this->driverController.GetZ()) +
              (this->driverController.GetTwist() *
               this->driverController.GetTwist())) < 0.3) {
        desired = this->prevTheta;
      } else {
        this->prevTheta = desired;
      }
      double out;
      /*
if (!this->thetaController.AtSetpoint()) {
out = this->thetaController.Calculate(
units::degree_t{this->drivetrain.getGyroAngle()}.value(),
units::degree_t{desired}.value());
} else {
out = 0.0;
this->thetaController.Calculate(
units::degree_t{this->drivetrain.getGyroAngle()}.value(),
units::degree_t{desired}.value());
}
*/
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
                 frc::ApplyDeadband(this->driverController.GetZ(), 0.1)) /
             4;
    }
  } else {
    /*
    auto ntInst = nt::NetworkTableInstance::GetDefault();
    auto table = ntInst.GetTable("visionTable");
    nt::DoublePublisher xPublisher =
        table->GetDoubleTopic("sourceCenterX").Publish();
    int x = xPublisher.GetTopic().GetEntry(0).Get();
    double out = 0.0;
    if (x != 0) {
      out = this->trackPIDController.Calculate(x, 0);
    }
    out = out > 1.0 ? 1.0 : out;
    out = out < -1.0 ? -1.0 : out;
    return out;
    */
    return 0.0;
  }
}

void RobotContainer::ConfigureBindings() {
  // Logitech Controller on XInput
  this->driverController.SetXChannel(1);
  this->driverController.SetYChannel(0);
  this->driverController.SetZChannel(4);
  this->driverController.SetTwistChannel(5);
  /*
  // intake note
  frc2::Trigger([this] {
    return this->secondController.GetRawButton(BindingConstants::intakeButton);
  }).WhileTrue(MoveToShooter(&this->intake, 0.4).ToPtr());
  // move note to shooter
  frc2::Trigger([this] {
    return this->secondController.GetRawAxis(
               BindingConstants::moveToShooterAxis) > 0.3;
  }).WhileTrue(MoveToShooter(&this->intake, 1.0).ToPtr());
  */
  /*
   // manual shooter up
   frc2::JoystickButton(&this->secondController,
                        BindingConstants::elevatorManualUpButton)
       .WhileTrue(
           ElevatorManual(&this->elevator, ElevatorConstants::speed).ToPtr());
   // manual shooter down
   frc2::JoystickButton(&this->secondController,
                        BindingConstants::elevatorManualDownButton)
       .WhileTrue(
           ElevatorManual(&this->elevator, -ElevatorConstants::speed).ToPtr());
           */
  // shooter to top
  /*
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
  */
  /*
    .WhileTrue(
      */
  // ElevatorManual(&this->elevator, -ElevatorConstants::speed).ToPtr());
  frc2::JoystickButton(&this->secondController,
                       BindingConstants::elevatorUpButton)
      .OnTrue(ElevatorToTop(&this->elevator).ToPtr());
  frc2::JoystickButton(&this->secondController,
                       BindingConstants::elevatorDownButton)
      .OnTrue(ElevatorToBottom(&this->elevator).ToPtr());
  frc2::JoystickButton(&this->secondController, BindingConstants::intakeButton)
      .WhileTrue(MoveToShooter(&this->intake, 0.1).ToPtr());
  frc2::JoystickButton(&this->secondController, BindingConstants::intakeReverseButton)
      .WhileTrue(MoveToShooter(&this->intake, -0.1).ToPtr());
  frc2::Trigger([this] {
    return this->secondController.GetRawAxis(
               BindingConstants::moveToShooterAxis) > 0.75;
  }).WhileTrue(MoveToShooter(&this->intake, 1.0).ToPtr());
  frc2::Trigger([this] {
    return this->secondController.GetRawAxis(
               BindingConstants::moveToShooterAxis) > 0.25;
  }).WhileTrue(Shoot(&this->shooter).ToPtr());
  /*
  // spin the shooter and then run the intake
  frc2::Trigger([this] {
  return this->secondController.GetRawButton(
  BindingConstants::shootCompositionTrigger) >= 0.3;
  })
  .OnTrue(
  Shoot(&this->shooter)
  .ToPtr()
  .AlongWith(frc2::WaitCommand(this->shooter.secondsToFull +
  0.25_s) .ToPtr()) .AndThen(MoveToShooter(&this->intake).ToPtr()));
  }
  */
  /*
    frc2::Trigger([this] {
      return this->intake.limitPressed();
    }).WhileTrue(LEDSet(&this->led, 0, 255, 0).ToPtr());
    */
}
// frc2::Trigger([this] { return Driver });
/*
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto ntInst = nt::NetworkTableInstance::GetDefault();
  auto table = ntInst.GetTable("visionTable");
  nt::DoublePublisher sourceXPublisher =
      table->GetDoubleTopic("sourceCenterX").Publish();
  return this->drivetrain
      .resetYaw(180_deg - ((sourceXPublisher.GetTopic().GetEntry(0.0).Get() /
                            VisionConstants::frontCameraXRes) *
                           VisionConstants::frontCameraHFOV))
      .AndThen(ElevatorToSetpoint(&this->elevator, ElevatorConstants::speaker)
                   .ToPtr())
      .AndThen(Shoot(&this->shooter).ToPtr())
      .AlongWith(
          frc2::WaitCommand(this->shooter.secondsToFull + 0.25_s).ToPtr())
      .AndThen(MoveToShooter(&this->intake).ToPtr())
      .AndThen(DriveDistance(&this->drivetrain, 6_ft + units::inch_t{4 + 1 /
8}) .ToPtr());
}
*/

void RobotContainer::VisionThread() {
  cs::UsbCamera outCamera = frc::CameraServer::StartAutomaticCapture(0);
  cs::CvSource outStream = frc::CameraServer::PutVideo("front", 640, 480);
}