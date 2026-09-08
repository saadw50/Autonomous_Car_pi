#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define HEX 16
#define LED_BUILTIN 13

#define constrain(value, low, high) \
  ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

class HardwareSerial {
 public:
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }
  void print(const char *) {}
  void print(char) {}
  void print(unsigned int, int = 10) {}
  void println(unsigned int, int = 10) {}
};

extern HardwareSerial Serial;

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return HIGH; }
inline void analogWrite(uint8_t, int) {}
inline unsigned long millis() { return 0; }
inline long map(long value, long in_min, long in_max, long out_min, long out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// ---------------------------------------------------------------------------
// Additions for the ultrasonic reflex (host-side syntax checking only).
// ---------------------------------------------------------------------------
inline unsigned long micros() { return 0; }
inline void delayMicroseconds(unsigned int) {}
inline void noInterrupts() {}
inline void interrupts() {}

// Pin-change interrupt registers (ATmega328P). Stubbed as plain bytes so the
// register writes in setupUltrasonic() type-check on the host.
static uint8_t PCICR = 0;
static uint8_t PCMSK0 = 0;
#define PCIE0 0
#define PCINT0_vect pcint0_vect
#define ISR(vec) void isr_handler_##vec(void)
