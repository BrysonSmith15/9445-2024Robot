// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>

#include "Constants.h"

// commands
#include "commands/Autos.h"
#include "commands/DriveCommand.h"
#include "commands/DriveDistance.h"
#include "commands/ElevatorToSetpoint.h"
#include "commands/LEDChase.h"
#include "commands/LEDSet.h"
// nt
#include <networktables/DoubleTopic.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  /*
  this->drivetrain.SetDefaultCommand(DriveCommand(
      &drivetrain, [this] { return this->getXState(); },
      [this] { return this->getYState(); },
      [this] { return this->getThetaState(); }));
      */
  this->thetaController.EnableContinuousInput(-180, 180);
  this->thetaController.SetTolerance(30);

  auto ntInst = nt::NetworkTableInstance::GetDefault();
  auto table = ntInst.GetTable("visionTable");
  nt::DoublePublisher sourceIDPublisher;
  if (auto alliance = frc::DriverStation::GetAlliance()) {
    if (alliance.value() = frc::DriverStation::Alliance::kRed) {
      this->sourceCenterId = 4;
      sourceIDPublisher = table->GetDoubleTopic("sourceCenterID").Publish();
    }
  }
  sourceIDPublisher.Set(this->sourceCenterId);
  std::thread visionThread(VisionThread);
  visionThread.detach();

  // if (this->driverController.Button(1).Get()) {
  // this->drivetrain.resetYaw();
  // }
  // this->elevator.SetDefaultCommand(ElevatorToSetpoint(&elevator));
  // set the leds all to the BSHS orange color (from their website)
  // this->led.SetDefaultCommand(LEDSet(&this->led, 255, 50, 0));
  this->led.SetDefaultCommand(
      LEDChase(&this->led, 255, 50, 0, 255, 255, 255, 40));
  this->intake.SetDefaultCommand(MoveToShooter(&this->intake, 0.0).ToPtr());
}

double RobotContainer::getXState() {
  return frc::ApplyDeadband(this->driverController.GetX(), 0.1);
}
double RobotContainer::getYState() {
  return -frc::ApplyDeadband(this->driverController.GetY(), 0.1);
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
      frc::SmartDashboard::PutNumber("Desired Rotation",
                                     units::degree_t{desired}.value());
      // if magnitude is less than 0.3, keep prev theta
      if (std::sqrt(
              (this->driverController.GetZ() * this->driverController.GetZ()) +
              (this->driverController.GetTwist() *
               this->driverController.GetTwist())) < 0.3) {
        desired = this->prevTheta;
        frc::SmartDashboard::PutBoolean("InDeadband", true);
      } else {
        this->prevTheta = desired;
        frc::SmartDashboard::PutBoolean("InDeadband", false);
      }
      double out;
      frc::SmartDashboard::PutNumber("error",
                                     this->thetaController.GetPositionError());
      frc::SmartDashboard::PutNumber("setpoint",
                                     this->thetaController.GetSetpoint());
      /*
frc::SmartDashboard::PutNumber(
"Angle_deg",
units::degree_t{this->drivetrain.getGyroAngle()}.value());
if (!this->thetaController.AtSetpoint()) {
out = this->thetaController.Calculate(
units::degree_t{this->drivetrain.getGyroAngle()}.value(),
units::degree_t{desired}.value());
frc::SmartDashboard::PutBoolean("PIDDeadband", false);
} else {
out = 0.0;
this->thetaController.Calculate(
units::degree_t{this->drivetrain.getGyroAngle()}.value(),
units::degree_t{desired}.value());
frc::SmartDashboard::PutBoolean("PIDDeadband", true);
}
*/
      frc::SmartDashboard::PutNumber("Out", out);
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
  // intake note
  frc2::Trigger([this] {
    return this->secondController.GetRawButton(BindingConstants::intakeButton);
  }).WhileTrue(MoveToShooter(&this->intake, 0.4).ToPtr());
  // move note to shooter
  frc2::Trigger([this] {
    return this->secondController.GetRawAxis(
               BindingConstants::moveToShooterAxis) > 0.3;
  }).WhileTrue(MoveToShooter(&this->intake, 1.0).ToPtr());
  // manual shooter up
  frc2::Trigger([this] {
    return this->secondController.GetRawButton(
        BindingConstants::elevatorManualUpButton);
  }).WhileTrue(this->elevator.manual(0.25));
  // manual shooter down
  frc2::Trigger([this] {
    return this->secondController.GetRawButton(
        BindingConstants::elevatorManualUpButton);
  }).WhileTrue(this->elevator.manual(-0.25));
  // Make the shooter run
  /*
  frc2::Trigger([this] {
    return this->secondController.GetRawButton(BindingConstants::shootButton);
  }).ToggleOnTrue(Shoot(&this->shooter).ToPtr());
  */
  /*
  // move shooter to bottom
  frc2::Trigger([this] {
    return this->secondController.GetPOV(0) ==
           BindingConstants::elevatorDownAngle;
  })
      .OnTrue(ElevatorToSetpoint(&this->elevator,
                                 ElevatorConstants::setpointOptions::bottom)
                  .ToPtr());
  // move shooter to amp
  frc2::Trigger([this] {
    return this->secondController.GetRawButton(
        BindingConstants::elevatorAmpButton);
  })
      .OnTrue(ElevatorToSetpoint(&this->elevator,
                                 ElevatorConstants::setpointOptions::amp)
                  .ToPtr());
  // move shooter to source
  frc2::Trigger([this] {
    return this->secondController.GetRawButton(
        BindingConstants::elevatorSourceButton);
  })
      .OnTrue(ElevatorToSetpoint(&this->elevator,
                                 ElevatorConstants::setpointOptions::source)
                  .ToPtr());
  // move shooter to top position
  frc2::Trigger([this] {
    return this->secondController.GetPOV(0) ==
           BindingConstants::elevatorUpAngle;
  })
      .OnTrue(ElevatorToSetpoint(&this->elevator,
                                 ElevatorConstants::setpointOptions::climb)
                  .ToPtr());
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
              */
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
      .AndThen(DriveDistance(&this->drivetrain, 6_ft + units::inch_t{4 + 1 / 8})
                   .ToPtr());
}
*/

void RobotContainer::VisionThread() {
  cs::UsbCamera outCamera = frc::CameraServer::StartAutomaticCapture(0);
  frc::AprilTagDetector detector;
  cs::CvSource outStream =
      frc::CameraServer::PutVideo("front", VisionConstants::frontCameraXRes,
                                  VisionConstants::frontCameraYRes);
  cv::Mat mat;
  cv::Mat grayMat;
  std::vector<int> tags;
  cs::CvSink cvSink = frc::CameraServer::GetVideo("front");
  auto ntInst = nt::NetworkTableInstance::GetDefault();
  auto table = ntInst.GetTable("visionTable");
  nt::DoublePublisher sourceIDPublisher =
      table->GetDoubleTopic("sourceCenterID").Publish();
  nt::DoublePublisher xPublisher =
      table->GetDoubleTopic("sourceCenterX").Publish();
  nt::DoublePublisher yPublisher =
      table->GetDoubleTopic("sourceCenterY").Publish();
  xPublisher.SetDefault(0);
  yPublisher.SetDefault(0);

  detector.AddFamily(VisionConstants::tagFamily);
  outCamera.SetResolution(640, 480);
  while (true) {
    int sourceCenterId = sourceIDPublisher.GetTopic().GetEntry(4).Get();
    if (cvSink.GrabFrame(mat) == 0) {
      outStream.NotifyError(cvSink.GetError());
      continue;
    }
    cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);
    cv::Size size = grayMat.size();
    frc::AprilTagDetector::Results detections =
        detector.Detect(size.width, size.height, grayMat.data);
    tags.clear();
    for (const frc::AprilTagDetection* detection : detections) {
      auto position = detection->GetCenter();
      if (detection->GetId() == sourceCenterId) {
        xPublisher.Set(position.x);
        yPublisher.Set(position.y);
      }
    }
  }
}