// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LEDSet.h"

LEDSet::LEDSet(LEDs* strip, int r, int g, int b)
    : leds{strip}, r{r}, g{g}, b{b} {
  // Register that this command requires the subsystem.
  AddRequirements(this->leds);
}

void LEDSet::Execute() {
  this->leds->set(0, this->leds->stripLen, this->r, this->g, this->b);
}

bool LEDSet::IsFinished() { return true; }