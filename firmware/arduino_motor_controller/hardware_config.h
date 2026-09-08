#pragma once

// -----------------------------------------------------------------------------
// EDIT THIS FILE TO MATCH YOUR MOTOR DRIVER AND WIRING BEFORE CONNECTING MOTORS.
// Default target: Arduino Uno + two brushed DC motors + L298N/TB6612-style
// differential-drive H-bridge.
// -----------------------------------------------------------------------------

// Serial link to the Raspberry Pi (normally through the Uno USB cable).
constexpr unsigned long SERIAL_BAUD = 115200UL;

// Left motor channel.
constexpr uint8_t LEFT_PWM_PIN = 5;   // Must be a PWM-capable pin on Uno.
constexpr uint8_t LEFT_IN1_PIN = 7;
constexpr uint8_t LEFT_IN2_PIN = 8;

// Right motor channel.
constexpr uint8_t RIGHT_PWM_PIN = 6;  // Must be a PWM-capable pin on Uno.
constexpr uint8_t RIGHT_IN1_PIN = 9;
constexpr uint8_t RIGHT_IN2_PIN = 10;

// Optional shared enable/standby pin. Set USE_DRIVER_ENABLE_PIN=false for an
// L298N with ENA/ENB driven directly by LEFT_PWM_PIN and RIGHT_PWM_PIN. Set it
// true for a TB6612 and connect DRIVER_ENABLE_PIN to STBY.
constexpr bool USE_DRIVER_ENABLE_PIN = false;
constexpr uint8_t DRIVER_ENABLE_PIN = 4;
constexpr bool DRIVER_ENABLE_ACTIVE_HIGH = true;

// E-stop sense input. This SOFTWARE input is a second safety layer. The
// physical E-stop should also interrupt the motor driver's enable/power path.
constexpr uint8_t ESTOP_SENSE_PIN = 2;
constexpr bool ESTOP_ACTIVE_LOW = true;

// Built-in LED indicates armed state; rapid blinking indicates a fault/E-stop.
constexpr uint8_t STATUS_LED_PIN = LED_BUILTIN;

// Reverse one channel if the two motors are mounted as mirror images. Verify
// with wheels lifted and change only these values if forward directions differ.
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = true;

// Conservative first-test output limit. Increase only after direction, E-stop,
// current draw, and stopping behavior have been measured.
constexpr int16_t MAX_COMMAND = 180;          // Allowed range: 1..255.
constexpr uint8_t MIN_EFFECTIVE_PWM = 55;    // Motor deadband compensation.

// Normal zero command behavior. false = coast, true = H-bridge short brake.
// Short braking can create current spikes; enable only after driver verification.
constexpr bool BRAKE_ON_ZERO_COMMAND = false;

// Output slew limiting for ordinary drive changes. Watchdog, E-stop, DISARM,
// and protocol faults bypass the ramp and stop immediately.
constexpr uint16_t MOTOR_UPDATE_PERIOD_MS = 10;
constexpr int16_t COMMAND_RAMP_STEP = 8;

// The Pi supplies a TTL with each DRIVE frame; it is clamped to this range.
constexpr uint16_t DEFAULT_WATCHDOG_MS = 250;
constexpr uint16_t MIN_WATCHDOG_MS = 100;
constexpr uint16_t MAX_WATCHDOG_MS = 1000;

constexpr uint16_t TELEMETRY_PERIOD_MS = 200;
constexpr uint16_t STATUS_LED_PERIOD_MS = 150;
constexpr size_t RX_BUFFER_SIZE = 96;

// -----------------------------------------------------------------------------
// HC-SR04 ultrasonic obstacle reflex (set USE_ULTRASONIC=false to compile out).
// The sensor lives on the ARDUINO, not the Pi: the Arduino is 5 V logic so ECHO
// needs no voltage divider, and the stop reacts without waiting on the USB link
// or the Pi's control loop.
//
// ECHO must stay OFF pin 2 - that is the E-stop sense input. ECHO is timed with
// a pin-change interrupt so accuracy does not depend on loop() speed; a polled
// read drifts badly while Serial is busy transmitting telemetry.
// ECHO_PIN must be on PORTB (Uno pins 8..13) for the PCINT0 vector used below.
// -----------------------------------------------------------------------------
constexpr bool USE_ULTRASONIC = true;
constexpr uint8_t ULTRASONIC_TRIG_PIN = 11;
constexpr uint8_t ULTRASONIC_ECHO_PIN = 12;   // Uno pin 12 = PB4 = PCINT4

// The HC-SR04 needs >=60 ms between pings or the previous echo bleeds into the next.
constexpr uint16_t ULTRASONIC_PERIOD_MS = 60;
// ~25 ms of flight time is roughly 4.3 m; beyond this we report "no reading".
constexpr unsigned long ULTRASONIC_TIMEOUT_US = 25000UL;

// Obstacle reflex thresholds in centimetres. The gap between them is hysteresis,
// so the car does not chatter in and out of the hold at the boundary.
constexpr uint16_t ULTRASONIC_STOP_CM = 30;
constexpr uint16_t ULTRASONIC_CLEAR_CM = 40;
// Require this many consecutive close readings before holding, so one spurious
// echo cannot stop the car.
constexpr uint8_t ULTRASONIC_CONFIRM_COUNT = 2;

static_assert(MAX_COMMAND > 0 && MAX_COMMAND <= 255,
              "MAX_COMMAND must be between 1 and 255");
static_assert(MIN_EFFECTIVE_PWM <= 255,
              "MIN_EFFECTIVE_PWM must be between 0 and 255");
static_assert(MIN_WATCHDOG_MS <= DEFAULT_WATCHDOG_MS &&
                  DEFAULT_WATCHDOG_MS <= MAX_WATCHDOG_MS,
              "Watchdog limits are inconsistent");
static_assert(!USE_ULTRASONIC || ULTRASONIC_ECHO_PIN != ESTOP_SENSE_PIN,
              "Ultrasonic ECHO must not share the E-stop sense pin");
static_assert(!USE_ULTRASONIC || ULTRASONIC_CLEAR_CM > ULTRASONIC_STOP_CM,
              "Ultrasonic clear distance must exceed the stop distance");
static_assert(!USE_ULTRASONIC ||
                  (ULTRASONIC_ECHO_PIN >= 8 && ULTRASONIC_ECHO_PIN <= 13),
              "Ultrasonic ECHO must be on PORTB (pins 8..13) for PCINT0");

