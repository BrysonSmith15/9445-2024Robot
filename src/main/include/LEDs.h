#pragma once

#include <frc/AddressableLED.h>
#include <frc2/Command/SubsystemBase.h>

class LEDs : public frc2::SubsystemBase {
 public:
  LEDs(int pwmID, int stripLen);
  void set(int start, int end, int r, int g, int b);
  int stripLen;

 private:
  frc::AddressableLED led;
  std::array<frc::AddressableLED::LEDData, 80> ledBuf;
};