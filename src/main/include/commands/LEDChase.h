// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "LEDs.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LEDChase : public frc2::CommandHelper<frc2::Command, LEDChase> {
 public:
  /**
   * Creates a new LEDChase.
   *
   * @param subsystem The subsystem used by this command.
   * @param m* the main color
   * @param c* the small color that "moves"
   * @param width the width of the chased section
   */
  explicit LEDChase(LEDs* led, int mr, int mg, int mb, int cr, int cg, int cb,
                    int width);
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;

  virtual bool RunsWhenDisabled() const override;

 private:
  LEDs* led;
  int mr;
  int mg;
  int mb;
  int cr;
  int cg;
  int cb;
  int width;
  int chasePoint = 0;
  int counter = 0;
};
