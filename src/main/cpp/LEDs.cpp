#include <LEDs.h>

LEDs::LEDs(int pwmID, int stripLen) : led{pwmID} {
  this->stripLen = stripLen;

  this->led.SetLength(stripLen);
  this->led.SetData(this->ledBuf);
  this->led.Start();
}

void LEDs::set(int start, int end, int r, int g, int b) {
  for (int i = start; i < end; i++) {
    this->ledBuf[i].SetRGB(r, g, b);
  }
}

void LEDs::Periodic() { this->led.SetData(this->ledBuf); }