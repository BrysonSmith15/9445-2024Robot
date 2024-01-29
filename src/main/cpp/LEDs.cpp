#include <LEDs.h>

LEDs::LEDs(int pwmID, int stripStart, int stripEnd) : led{pwmID} {
  this->stripStart = stripStart;
  this->stripEnd = stripEnd;

  this->led.SetLength(stripEnd - stripStart);
  this->led.SetData(this->ledBuf);
  this->led.Start();
}

void LEDs::set(int start, int end, int r, int g, int b) {
  for (int i = start; i < end; i++) {
    this->ledBuf[i].SetRGB(r, g, b);
  }
}