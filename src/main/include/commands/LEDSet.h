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
class LEDSet : public frc2::CommandHelper<frc2::Command, LEDSet> {
 public:
  /**
   * Creates a new LEDSet.
   *
   * @param strip The LED Strip used by this command.
   */
  explicit LEDSet(LEDs* strip, int r, int g, int b);
  void Execute() override;
  bool IsFinished() override;

 private:
  LEDs* leds;
  int r;
  int g;
  int b;
};
