#include <Arduino.h>
#include <math.h>
#include <util/atomic.h>

// Arduino Uno R3 / ATmega328P, 5 V logic.
// Receiver CH1 -> D2. DM556 PUL+ -> D8, DIR+ -> D9, ENA+ -> D10.
// DM556 PUL-, DIR-, ENA- -> Uno GND. Use only one controller per driver.

constexpr uint8_t RC_PIN = 2;
constexpr uint8_t STEP_PIN = 8;
constexpr uint8_t DIR_PIN = 9;
constexpr uint8_t ENABLE_PIN = 10;

// For the documented common-cathode DM556 connection.
constexpr uint8_t DRIVER_ENABLE_LEVEL = LOW;
constexpr uint8_t DRIVER_DISABLE_LEVEL = HIGH;

constexpr uint32_t RC_MIN_US = 1000;
constexpr uint32_t RC_REVERSE_US = 1400;
constexpr uint32_t RC_FORWARD_US = 1600;
constexpr uint32_t RC_MAX_US = 2000;
constexpr uint32_t RC_TIMEOUT_US = 200000UL;

constexpr float MAX_STEP_SPEED = 60.0f;  // STEP pulses per second
constexpr float STEP_ACCEL = 30.0f;      // STEP pulses per second squared
constexpr uint32_t ENABLE_DELAY_US = 300000UL;
constexpr uint8_t DIR_SETUP_US = 10;
constexpr uint8_t STEP_PULSE_US = 5;

volatile uint32_t rcRiseUs = 0;
volatile uint32_t rcLastPulseUs = 0;
volatile uint32_t rcWidthUs = 0;
volatile bool rcSawRise = false;
volatile bool rcReceived = false;

struct RcSignal {
    uint32_t widthUs;
    uint32_t ageUs;
    bool received;
    bool valid;
};

bool driverEnabled = false;
int commandDirection = 0;
float stepSpeed = 0;
uint32_t enabledAtUs = 0;
uint32_t lastStepUs = 0;

void captureRc() {
    const uint32_t now = micros();
    if (digitalRead(RC_PIN) == HIGH) {
        rcRiseUs = now;
        rcSawRise = true;
    } else if (rcSawRise) {
        // Keep out-of-range widths so they stop motion instead of being clamped.
        rcWidthUs = now - rcRiseUs;
        rcLastPulseUs = now;
        rcReceived = true;
        rcSawRise = false;
    }
}

RcSignal readRc() {
    uint32_t width;
    uint32_t last;
    bool received;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        width = rcWidthUs;
        last = rcLastPulseUs;
        received = rcReceived;
    }
    // Read time after the ISR snapshot to avoid a timestamp race.
    const uint32_t age = micros() - last;
    const bool valid = received && age <= RC_TIMEOUT_US
        && width >= RC_MIN_US && width <= RC_MAX_US;
    return {width, age, received, valid};
}

int requestedDirection(const RcSignal &rc) {
    if (!rc.valid || rc.widthUs < RC_MIN_US || rc.widthUs > RC_MAX_US) {
        return 0;
    }
    if (rc.widthUs > RC_FORWARD_US) {
        return 1;
    }
    if (rc.widthUs < RC_REVERSE_US) {
        return -1;
    }
    return 0;
}

float rampTo(float current, float target, float maxChange) {
    const float delta = target - current;
    if (fabsf(delta) <= maxChange) {
        return target;
    }
    return current + (delta > 0 ? maxChange : -maxChange);
}

void stopStepper(uint32_t now) {
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(ENABLE_PIN, DRIVER_DISABLE_LEVEL);
    driverEnabled = false;
    stepSpeed = 0;
    lastStepUs = now;
}

void pulseStepper(int direction) {
    digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
}

void updateStepper(const RcSignal &rc, uint32_t now, float dt) {
    commandDirection = requestedDirection(rc);
    if (commandDirection == 0) {
        stopStepper(now);
        return;
    }

    if (!driverEnabled) {
        digitalWrite(ENABLE_PIN, DRIVER_ENABLE_LEVEL);
        driverEnabled = true;
        enabledAtUs = now;
        lastStepUs = now;
    }
    if ((uint32_t)(now - enabledAtUs) < ENABLE_DELAY_US) {
        return;
    }

    float wanted = commandDirection * MAX_STEP_SPEED;
    if (stepSpeed * wanted < 0) {
        wanted = 0;
    }
    stepSpeed = rampTo(stepSpeed, wanted, STEP_ACCEL * dt);
    if (fabsf(stepSpeed) < 0.5f) {
        lastStepUs = now;
        return;
    }

    const uint32_t interval = (uint32_t)(1000000.0f / fabsf(stepSpeed));
    if ((uint32_t)(now - lastStepUs) >= interval) {
        lastStepUs = now;
        pulseStepper(stepSpeed > 0 ? 1 : -1);
    }
}

void printDiagnostics(const RcSignal &rc) {
    Serial.print(F("CH1="));
    Serial.print(rc.widthUs);
    Serial.print(F(" us state="));
    if (!rc.received) {
        Serial.print(F("NEVER"));
    } else if (rc.ageUs > RC_TIMEOUT_US) {
        Serial.print(F("LOST"));
    } else if (!rc.valid) {
        Serial.print(F("OUT_OF_RANGE"));
    } else {
        Serial.print(F("OK"));
    }
    Serial.print(F(" command="));
    Serial.print(commandDirection);
    Serial.print(F(" speed="));
    Serial.print(stepSpeed, 1);
    Serial.print(F(" pulses/s enabled="));
    Serial.println(driverEnabled ? 1 : 0);
}

void setup() {
    // Preload latches before enabling outputs.
    digitalWrite(ENABLE_PIN, DRIVER_DISABLE_LEVEL);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(STEP_PIN, LOW);
    pinMode(STEP_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(RC_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RC_PIN), captureRc, CHANGE);
    Serial.begin(115200);
    Serial.println(F("UNO CH1 DM556 READY"));
}

void loop() {
    static uint32_t lastLoopUs = micros();
    static uint32_t lastPrintMs = 0;
    const RcSignal rc = readRc();
    const uint32_t now = micros();
    float dt = (uint32_t)(now - lastLoopUs) * 0.000001f;
    dt = constrain(dt, 0.0f, 0.02f);
    lastLoopUs = now;
    updateStepper(rc, now, dt);
    if ((uint32_t)(millis() - lastPrintMs) >= 500UL) {
        lastPrintMs = millis();
        printDiagnostics(rc);
    }
}
