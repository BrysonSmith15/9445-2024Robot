// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Elevator.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ElevatorManual
    : public frc2::CommandHelper<frc2::Command, ElevatorManual> {
 public:
  /**
   * Creates a new ElevatorManual.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit ElevatorManual(Elevator* elevator, double power);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

 private:
  Elevator* elevator;
  double power;
};
