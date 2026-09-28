#pragma once

#include <Arduino.h>

/*
 * Reads distance frames from a MaxBotix MB7389 on a hardware serial port.
 * The sensor streams "RXXXX\r" at 9600 baud, where XXXX is the distance in mm.
 */
class MB7389 {
  Stream* _serial = NULL;
  char _buf[4];        // the 4 digits of the frame currently being received
  int _len = -1;       // digits received so far, -1 = waiting for 'R'
  uint16_t _distance_mm = 0;
  unsigned long _last_read_at = 0;

public:
  void begin(Stream& serial) { _serial = &serial; }

  // Call often from loop(). Returns true each time a complete reading arrives.
  bool poll() {
    bool got_reading = false;
    while (_serial && _serial->available()) {
      char c = _serial->read();
      if (c == 'R') {
        _len = 0;                    // start of a new frame
      } else if (_len < 0) {
        // not inside a frame, ignore
      } else if (c >= '0' && c <= '9' && _len < 4) {
        _buf[_len++] = c;
      } else if (c == '\r' && _len == 4) {
        _distance_mm = (_buf[0] - '0') * 1000 + (_buf[1] - '0') * 100 + (_buf[2] - '0') * 10 + (_buf[3] - '0');
        _last_read_at = millis();
        got_reading = true;
        _len = -1;
      } else {
        _len = -1;                   // malformed frame, wait for the next 'R'
      }
    }
    return got_reading;
  }

  uint16_t getDistanceMM() const { return _distance_mm; }
  unsigned long getLastReadAt() const { return _last_read_at; }   // millis() of last good reading, 0 = never
};
