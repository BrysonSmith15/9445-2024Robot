// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveDistance : public frc2::CommandHelper<frc2::Command, DriveDistance> {
 public:
  /**
   * Creates a new DriveDistance.
   *
   * @param drivetrain The subsystem used by this command.
   */
  explicit DriveDistance(Drivetrain* drivetrain, units::foot_t distance);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

 private:
  Drivetrain* drivetrain;
  units::foot_t distance;
  frc::SwerveModuleState desiredState;
  frc::PIDController speedController{6e-5, 0.0, 0.0};
  const units::foot_t tolerance = 6_in;
};
