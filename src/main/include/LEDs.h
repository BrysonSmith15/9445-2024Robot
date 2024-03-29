#pragma once

#include <frc/AddressableLED.h>
#include <frc2/Command/SubsystemBase.h>

class LEDs {
 public:
  LEDs(int pwmID, int stripStart, int stripEnd);
  void set(int start, int end, int r, int g, int b);

 private:
  frc::AddressableLED led;
  std::array<frc::AddressableLED::LEDData, 80> ledBuf;

  int stripStart;
  int stripEnd;
};