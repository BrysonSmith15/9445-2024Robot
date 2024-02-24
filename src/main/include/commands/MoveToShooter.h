// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveToShooter : public frc2::CommandHelper<frc2::Command, MoveToShooter> {
 public:
  /**
   * Creates a new MoveToShooter.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit MoveToShooter(Intake* intake, double speed);
  void Initialize() override;
  // void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

 private:
  Intake* intake;
  frc::Timer timer;
  double speed = 0.0;
};
