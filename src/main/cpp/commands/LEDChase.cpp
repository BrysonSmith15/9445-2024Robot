// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LEDChase.h"

LEDChase::LEDChase(LEDs* leds, int mr, int mg, int mb, int cr, int cg, int cb,
                   int width)
    : led{leds}, mr{mr}, mg{mg}, mb{mb}, cr{cr}, cg{cg}, cb{cb}, width{width} {
  // Register that this command requires the subsystem.
  AddRequirements(this->led);
}

void LEDChase::Initialize() {
  this->counter = 0;
  this->chasePoint = 0;
}

void LEDChase::Execute() {
  this->counter++;
  // slow it down a little bit
  if (this->counter % 1 == 0) {
    this->chasePoint++;
    this->chasePoint %= this->led->stripLen;
  }
  int chaseEnd = this->chasePoint + this->width;
  if (chaseEnd > this->led->stripLen) {
    // chase section
    this->led->set(this->chasePoint, this->led->stripLen, this->cr, this->cg,
                   this->cb);
    this->led->set(0, chaseEnd - this->led->stripLen, this->cr, this->cg,
                   this->cb);
    // main section
    this->led->set(chaseEnd - this->led->stripLen + 1, this->chasePoint,
                   this->mr, this->mg, this->mb);
  } else {
    // chase section
    this->led->set(this->chasePoint, chaseEnd, this->cr, this->cg, this->cb);
    // main section
    this->led->set(0, this->chasePoint, this->mr, this->mg, this->mb);
    this->led->set(chaseEnd, this->led->stripLen, this->mr, this->mg, this->mb);
  }
}

void LEDChase::End(bool interrupted) {
  this->led->set(0, this->led->stripLen, this->mr, this->mg, this->mb);
}

bool LEDChase::RunsWhenDisabled() const { return true; }