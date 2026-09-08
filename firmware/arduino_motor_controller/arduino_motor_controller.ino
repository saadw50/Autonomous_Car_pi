#include <Arduino.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "hardware_config.h"

// Frame format:
//   PAYLOAD*CCCC\n
// CCCC is uppercase CRC-16/CCITT-FALSE over the ASCII PAYLOAD.
// Commands:
//   CLEAR,seq
//   ARM,seq
//   DRIVE,seq,left,right,ttl_ms
//   DISARM,seq
//   PING,seq
//   STATUS,seq
// Responses use the same framing:
//   ACK,seq,state,fault_mask
//   NACK,seq,reason
//   TEL,millis,state,fault_mask,current_left,current_right,target_left,
//       target_right,estop,drive_age_ms,last_seq,bad_frames,dist_cm,obstacle
// dist_cm  : HC-SR04 distance in cm, 0 = no reading / out of range
// obstacle : 1 while the ultrasonic reflex is holding the motors at zero

enum class ControllerState : uint8_t {
  DISARMED = 0,
  ARMED = 1,
  ESTOP = 2,
  FAULT = 3,
};

enum FaultFlag : uint16_t {
  FAULT_NONE = 0,
  FAULT_ESTOP = 1U << 0,
  FAULT_WATCHDOG = 1U << 1,
  FAULT_RX_OVERFLOW = 1U << 2,
  FAULT_BAD_CRC = 1U << 3,
  FAULT_BAD_COMMAND = 1U << 4,
  FAULT_BAD_SEQUENCE = 1U << 5,
};

ControllerState controllerState = ControllerState::DISARMED;
uint16_t latchedFaults = FAULT_NONE;

int16_t targetLeft = 0;
int16_t targetRight = 0;
int16_t currentLeft = 0;
int16_t currentRight = 0;

uint16_t watchdogMs = DEFAULT_WATCHDOG_MS;
unsigned long lastDriveMs = 0;
unsigned long lastMotorUpdateMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastLedUpdateMs = 0;

uint16_t lastSequence = 0;
bool hasSequence = false;
uint16_t badFrameCount = 0;

char rxBuffer[RX_BUFFER_SIZE];
size_t rxLength = 0;
bool rxOverflowing = false;
bool statusLedOn = false;

// --- HC-SR04 ultrasonic state ------------------------------------------------
// echoRiseUs/echoWidthUs/echoReady are written by the pin-change ISR, so any
// read from loop() context copies them with interrupts briefly disabled.
volatile unsigned long echoRiseUs = 0;
volatile unsigned long echoWidthUs = 0;
volatile bool echoReady = false;

unsigned long lastPingMs = 0;
unsigned long pingSentUs = 0;
bool pingPending = false;
uint16_t distanceCm = 0;      // 0 means "no valid reading"
bool obstacleHold = false;    // true while the reflex is holding motors at zero
uint8_t closeReadings = 0;

const char *stateName(ControllerState state) {
  switch (state) {
    case ControllerState::DISARMED:
      return "DISARMED";
    case ControllerState::ARMED:
      return "ARMED";
    case ControllerState::ESTOP:
      return "ESTOP";
    case ControllerState::FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

uint16_t crc16CcittFalse(const char *data) {
  uint16_t crc = 0xFFFFU;
  while (*data != '\0') {
    crc ^= static_cast<uint16_t>(static_cast<uint8_t>(*data++)) << 8U;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

void sendPayload(const char *format, ...) {
  char payload[176];
  va_list args;
  va_start(args, format);
  const int written = vsnprintf(payload, sizeof(payload), format, args);
  va_end(args);

  if (written < 0 || static_cast<size_t>(written) >= sizeof(payload)) {
    return;
  }

  const uint16_t crc = crc16CcittFalse(payload);
  Serial.print(payload);
  Serial.print('*');
  if (crc < 0x1000U) Serial.print('0');
  if (crc < 0x0100U) Serial.print('0');
  if (crc < 0x0010U) Serial.print('0');
  Serial.println(crc, HEX);
}

void sendAck(uint16_t sequence) {
  sendPayload("ACK,%u,%s,%u", sequence, stateName(controllerState), latchedFaults);
}

void sendNack(uint16_t sequence, const char *reason) {
  sendPayload("NACK,%u,%s", sequence, reason);
}

bool estopIsActive() {
  const bool pinHigh = digitalRead(ESTOP_SENSE_PIN) == HIGH;
  return ESTOP_ACTIVE_LOW ? !pinHigh : pinHigh;
}

// -----------------------------------------------------------------------------
// HC-SR04 ultrasonic obstacle reflex.
//
// ECHO is timed by a pin-change interrupt rather than pulseIn(): pulseIn blocks
// for up to ~25 ms, which would stall the serial and safety work in loop(), and
// a polled read loses accuracy whenever Serial is busy sending telemetry.
// -----------------------------------------------------------------------------
#if defined(PCINT0_vect) && !defined(ULTRASONIC_NO_ISR)
ISR(PCINT0_vect) {
  if (!USE_ULTRASONIC) return;
  if (digitalRead(ULTRASONIC_ECHO_PIN) == HIGH) {
    echoRiseUs = micros();
  } else if (echoRiseUs != 0UL) {
    echoWidthUs = micros() - echoRiseUs;
    echoRiseUs = 0UL;
    echoReady = true;
  }
}
#endif

void setupUltrasonic() {
  if (!USE_ULTRASONIC) return;
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
#if defined(PCICR) && defined(PCIE0) && defined(PCMSK0)
  // Enable the pin-change interrupt for the ECHO pin (PORTB / PCINT0 group).
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << (ULTRASONIC_ECHO_PIN - 8));
#endif
}

// Fire a ping. The 10 us trigger pulse is short enough to send inline.
void triggerUltrasonic(unsigned long now) {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  lastPingMs = now;
  pingSentUs = micros();
  pingPending = true;
}

void updateUltrasonic(unsigned long now) {
  if (!USE_ULTRASONIC) return;

  // Collect a completed echo, if the ISR finished one.
  bool ready = false;
  unsigned long widthUs = 0;
  noInterrupts();
  if (echoReady) {
    ready = true;
    widthUs = echoWidthUs;
    echoReady = false;
  }
  interrupts();

  if (ready && pingPending) {
    pingPending = false;
    // Sound travels ~29.1 us per cm, and the pulse covers there and back.
    const unsigned long cm = widthUs / 58UL;
    distanceCm = (cm == 0UL || cm > 500UL) ? 0 : static_cast<uint16_t>(cm);
  } else if (pingPending && (micros() - pingSentUs) > ULTRASONIC_TIMEOUT_US) {
    // Nothing came back in time: treat as "nothing within range", not as an
    // obstacle. A missing sensor must never wedge the car in a permanent stop.
    pingPending = false;
    noInterrupts();
    echoRiseUs = 0UL;
    interrupts();
    distanceCm = 0;
  }

  // Hysteresis + confirmation, so one bad echo cannot stop the car and the
  // hold does not chatter at the threshold.
  if (distanceCm != 0 && distanceCm <= ULTRASONIC_STOP_CM) {
    if (closeReadings < 255) ++closeReadings;
    if (closeReadings >= ULTRASONIC_CONFIRM_COUNT) obstacleHold = true;
  } else {
    closeReadings = 0;
    if (distanceCm == 0 || distanceCm >= ULTRASONIC_CLEAR_CM) obstacleHold = false;
  }

  if (!pingPending && (now - lastPingMs) >= ULTRASONIC_PERIOD_MS) {
    triggerUltrasonic(now);
  }
}

void setDriverEnabled(bool enabled) {
  if (!USE_DRIVER_ENABLE_PIN) return;
  const bool pinHigh = DRIVER_ENABLE_ACTIVE_HIGH ? enabled : !enabled;
  digitalWrite(DRIVER_ENABLE_PIN, pinHigh ? HIGH : LOW);
}

void forceMotorPinsSafe() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  digitalWrite(LEFT_IN1_PIN, LOW);
  digitalWrite(LEFT_IN2_PIN, LOW);
  digitalWrite(RIGHT_IN1_PIN, LOW);
  digitalWrite(RIGHT_IN2_PIN, LOW);
  setDriverEnabled(false);
}

void immediateStop(ControllerState state, uint16_t faultToAdd) {
  targetLeft = 0;
  targetRight = 0;
  currentLeft = 0;
  currentRight = 0;
  latchedFaults |= faultToAdd;
  controllerState = state;
  forceMotorPinsSafe();
}

uint8_t commandToPwm(int16_t magnitude) {
  if (magnitude <= 0) return 0;
  if (MAX_COMMAND <= 1 || MIN_EFFECTIVE_PWM >= MAX_COMMAND) {
    return static_cast<uint8_t>(constrain(magnitude, 0, 255));
  }
  const long mapped = map(magnitude, 1, MAX_COMMAND,
                          MIN_EFFECTIVE_PWM, MAX_COMMAND);
  return static_cast<uint8_t>(constrain(mapped, 0, 255));
}

void writeMotor(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin,
                int16_t command, bool inverted) {
  int16_t value = inverted ? -command : command;

  if (value == 0) {
    if (BRAKE_ON_ZERO_COMMAND && controllerState == ControllerState::ARMED) {
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, HIGH);
      analogWrite(pwmPin, 255);
    } else {
      analogWrite(pwmPin, 0);
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
    }
    return;
  }

  const bool forward = value > 0;
  const int16_t magnitude = abs(value);
  digitalWrite(in1Pin, forward ? HIGH : LOW);
  digitalWrite(in2Pin, forward ? LOW : HIGH);
  analogWrite(pwmPin, commandToPwm(magnitude));
}

int16_t moveToward(int16_t current, int16_t target, int16_t step) {
  if (current < target) {
    const int16_t candidate = static_cast<int16_t>(current + step);
    return candidate > target ? target : candidate;
  }
  if (current > target) {
    const int16_t candidate = static_cast<int16_t>(current - step);
    return candidate < target ? target : candidate;
  }
  return current;
}

void updateMotors(unsigned long now) {
  if (now - lastMotorUpdateMs < MOTOR_UPDATE_PERIOD_MS) return;
  lastMotorUpdateMs = now;

  if (controllerState != ControllerState::ARMED || estopIsActive()) {
    forceMotorPinsSafe();
    return;
  }

  // Ultrasonic reflex: hold the output at zero while something is too close.
  // This is a non-latching hold, not a fault - the car resumes by itself once
  // the way is clear, so an obstacle does not need an operator to CLEAR it.
  // Forward motion is blocked; reversing away is still allowed.
  if (obstacleHold && (targetLeft > 0 || targetRight > 0)) {
    currentLeft = moveToward(currentLeft, 0, COMMAND_RAMP_STEP);
    currentRight = moveToward(currentRight, 0, COMMAND_RAMP_STEP);
    setDriverEnabled(true);
    writeMotor(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN,
               currentLeft, LEFT_MOTOR_INVERTED);
    writeMotor(RIGHT_PWM_PIN, RIGHT_IN1_PIN, RIGHT_IN2_PIN,
               currentRight, RIGHT_MOTOR_INVERTED);
    return;
  }

  currentLeft = moveToward(currentLeft, targetLeft, COMMAND_RAMP_STEP);
  currentRight = moveToward(currentRight, targetRight, COMMAND_RAMP_STEP);
  setDriverEnabled(true);
  writeMotor(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN,
             currentLeft, LEFT_MOTOR_INVERTED);
  writeMotor(RIGHT_PWM_PIN, RIGHT_IN1_PIN, RIGHT_IN2_PIN,
             currentRight, RIGHT_MOTOR_INVERTED);
}

bool parseLongStrict(const char *text, long minimum, long maximum, long &value) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  const long parsed = strtol(text, &end, 10);
  if (end == text || *end != '\0' || parsed < minimum || parsed > maximum) {
    return false;
  }
  value = parsed;
  return true;
}

bool isNewSequence(uint16_t sequence) {
  if (!hasSequence) return true;
  const uint16_t delta = static_cast<uint16_t>(sequence - lastSequence);
  return delta != 0U && delta < 0x8000U;
}

bool acceptSequence(uint16_t sequence) {
  if (!isNewSequence(sequence)) return false;
  lastSequence = sequence;
  hasSequence = true;
  return true;
}

void handleProtocolFault(uint16_t fault, const char *reason) {
  ++badFrameCount;
  latchedFaults |= fault;
  if (controllerState == ControllerState::ARMED) {
    immediateStop(ControllerState::FAULT, fault);
  }
  sendNack(0, reason);
}

bool validateAndStripCrc(char *line) {
  char *separator = strrchr(line, '*');
  if (separator == nullptr) return false;
  *separator = '\0';
  const char *crcText = separator + 1;
  if (strlen(crcText) != 4) return false;
  for (uint8_t i = 0; i < 4; ++i) {
    if (!isxdigit(static_cast<unsigned char>(crcText[i]))) return false;
  }
  const uint16_t received = static_cast<uint16_t>(strtoul(crcText, nullptr, 16));
  return received == crc16CcittFalse(line);
}

bool requireNoExtraToken(char *savePointer) {
  return strtok_r(nullptr, ",", &savePointer) == nullptr;
}

void processCommand(char *payload, unsigned long now) {
  char *savePointer = nullptr;
  char *command = strtok_r(payload, ",", &savePointer);
  char *sequenceText = strtok_r(nullptr, ",", &savePointer);
  long sequenceLong = 0;
  if (command == nullptr ||
      !parseLongStrict(sequenceText, 0, 65535, sequenceLong)) {
    handleProtocolFault(FAULT_BAD_COMMAND, "BAD_FORMAT");
    return;
  }
  const uint16_t sequence = static_cast<uint16_t>(sequenceLong);

  if (strcmp(command, "PING") == 0 || strcmp(command, "STATUS") == 0) {
    if (!requireNoExtraToken(savePointer)) {
      handleProtocolFault(FAULT_BAD_COMMAND, "BAD_FORMAT");
      return;
    }
    sendAck(sequence);
    return;
  }

  if (strcmp(command, "CLEAR") == 0) {
    if (!requireNoExtraToken(savePointer)) {
      handleProtocolFault(FAULT_BAD_COMMAND, "BAD_FORMAT");
      return;
    }
    if (estopIsActive()) {
      immediateStop(ControllerState::ESTOP, FAULT_ESTOP);
      sendNack(sequence, "ESTOP_ACTIVE");
      return;
    }
    latchedFaults = FAULT_NONE;
    badFrameCount = 0;
    hasSequence = true;
    lastSequence = sequence;
    immediateStop(ControllerState::DISARMED, FAULT_NONE);
    sendAck(sequence);
    return;
  }

  if (!acceptSequence(sequence)) {
    latchedFaults |= FAULT_BAD_SEQUENCE;
    if (controllerState == ControllerState::ARMED) {
      immediateStop(ControllerState::FAULT, FAULT_BAD_SEQUENCE);
    }
    sendNack(sequence, "BAD_SEQUENCE");
    return;
  }

  if (strcmp(command, "ARM") == 0) {
    if (!requireNoExtraToken(savePointer)) {
      handleProtocolFault(FAULT_BAD_COMMAND, "BAD_FORMAT");
      return;
    }
    if (estopIsActive()) {
      immediateStop(ControllerState::ESTOP, FAULT_ESTOP);
      sendNack(sequence, "ESTOP_ACTIVE");
      return;
    }
    if (latchedFaults != FAULT_NONE) {
      immediateStop(ControllerState::FAULT, FAULT_NONE);
      sendNack(sequence, "FAULT_LATCHED");
      return;
    }
    targetLeft = targetRight = currentLeft = currentRight = 0;
    watchdogMs = DEFAULT_WATCHDOG_MS;
    lastDriveMs = now;
    controllerState = ControllerState::ARMED;
    setDriverEnabled(true);
    sendAck(sequence);
    return;
  }

  if (strcmp(command, "DISARM") == 0) {
    if (!requireNoExtraToken(savePointer)) {
      handleProtocolFault(FAULT_BAD_COMMAND, "BAD_FORMAT");
      return;
    }
    immediateStop(ControllerState::DISARMED, FAULT_NONE);
    sendAck(sequence);
    return;
  }

  if (strcmp(command, "DRIVE") == 0) {
    char *leftText = strtok_r(nullptr, ",", &savePointer);
    char *rightText = strtok_r(nullptr, ",", &savePointer);
    char *ttlText = strtok_r(nullptr, ",", &savePointer);
    long leftLong = 0;
    long rightLong = 0;
    long ttlLong = 0;
    if (!parseLongStrict(leftText, -255, 255, leftLong) ||
        !parseLongStrict(rightText, -255, 255, rightLong) ||
        !parseLongStrict(ttlText, MIN_WATCHDOG_MS, MAX_WATCHDOG_MS, ttlLong) ||
        !requireNoExtraToken(savePointer)) {
      handleProtocolFault(FAULT_BAD_COMMAND, "BAD_DRIVE");
      return;
    }
    if (controllerState != ControllerState::ARMED ||
        latchedFaults != FAULT_NONE || estopIsActive()) {
      immediateStop(estopIsActive() ? ControllerState::ESTOP
                                    : ControllerState::FAULT,
                    estopIsActive() ? FAULT_ESTOP : FAULT_NONE);
      sendNack(sequence, "NOT_ARMED");
      return;
    }
    targetLeft = static_cast<int16_t>(constrain(leftLong, -MAX_COMMAND, MAX_COMMAND));
    targetRight = static_cast<int16_t>(constrain(rightLong, -MAX_COMMAND, MAX_COMMAND));
    watchdogMs = static_cast<uint16_t>(ttlLong);
    lastDriveMs = now;
    sendAck(sequence);
    return;
  }

  latchedFaults |= FAULT_BAD_COMMAND;
  if (controllerState == ControllerState::ARMED) {
    immediateStop(ControllerState::FAULT, FAULT_BAD_COMMAND);
  }
  sendNack(sequence, "UNKNOWN_COMMAND");
}

void processLine(char *line, unsigned long now) {
  if (!validateAndStripCrc(line)) {
    handleProtocolFault(FAULT_BAD_CRC, "BAD_CRC");
    return;
  }
  processCommand(line, now);
}

void readSerial(unsigned long now) {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') continue;

    if (incoming == '\n') {
      if (rxOverflowing) {
        rxOverflowing = false;
        rxLength = 0;
        handleProtocolFault(FAULT_RX_OVERFLOW, "RX_OVERFLOW");
        continue;
      }
      if (rxLength == 0) continue;
      rxBuffer[rxLength] = '\0';
      processLine(rxBuffer, now);
      rxLength = 0;
      continue;
    }

    if (rxOverflowing) continue;
    if (rxLength + 1 >= RX_BUFFER_SIZE) {
      rxOverflowing = true;
      continue;
    }
    rxBuffer[rxLength++] = incoming;
  }
}

void monitorSafety(unsigned long now) {
  if (estopIsActive()) {
    if (controllerState != ControllerState::ESTOP) {
      immediateStop(ControllerState::ESTOP, FAULT_ESTOP);
    }
    return;
  }

  if (controllerState == ControllerState::ARMED &&
      now - lastDriveMs > watchdogMs) {
    immediateStop(ControllerState::FAULT, FAULT_WATCHDOG);
  }
}

void sendTelemetry(unsigned long now) {
  if (now - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;
  const unsigned long driveAge = now - lastDriveMs;
  // Two fields were appended (dist_cm, obstacle). Older parsers that read the
  // first eleven fields keep working; new ones read the distance too.
  sendPayload("TEL,%lu,%s,%u,%d,%d,%d,%d,%u,%lu,%u,%u,%u,%u",
              now, stateName(controllerState), latchedFaults,
              currentLeft, currentRight, targetLeft, targetRight,
              estopIsActive() ? 1U : 0U, driveAge,
              hasSequence ? lastSequence : 0U, badFrameCount,
              distanceCm, obstacleHold ? 1U : 0U);
}

void updateStatusLed(unsigned long now) {
  if (controllerState == ControllerState::ARMED) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    statusLedOn = true;
    return;
  }
  if (controllerState == ControllerState::FAULT ||
      controllerState == ControllerState::ESTOP) {
    if (now - lastLedUpdateMs >= STATUS_LED_PERIOD_MS) {
      lastLedUpdateMs = now;
      statusLedOn = !statusLedOn;
      digitalWrite(STATUS_LED_PIN, statusLedOn ? HIGH : LOW);
    }
    return;
  }
  statusLedOn = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

void setup() {
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_IN1_PIN, OUTPUT);
  pinMode(LEFT_IN2_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_IN1_PIN, OUTPUT);
  pinMode(RIGHT_IN2_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  if (USE_DRIVER_ENABLE_PIN) pinMode(DRIVER_ENABLE_PIN, OUTPUT);
  pinMode(ESTOP_SENSE_PIN, ESTOP_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  setupUltrasonic();

  forceMotorPinsSafe();
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.begin(SERIAL_BAUD);
  lastDriveMs = millis();
  sendPayload("HELLO,1,%s", stateName(controllerState));
}

void loop() {
  const unsigned long now = millis();
  monitorSafety(now);
  readSerial(now);
  monitorSafety(now);
  updateUltrasonic(now);
  updateMotors(now);
  sendTelemetry(now);
  updateStatusLed(now);
}
