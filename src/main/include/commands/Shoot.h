// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Shoot : public frc2::CommandHelper<frc2::Command, Shoot> {
 public:
  /**
   * Creates a new Shoot.
   *
   * @param shooter The subsystem used by this command.
   */
  Shoot(Shooter* shooter);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

 private:
  Shooter* m_shooter;
  const double speed = 1;
};
