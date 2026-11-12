
#include <Arduino.h>
#include <math.h>

// =====================================================
// ESP32-WROOM-32 DEVKIT
//
// CH1: NEMA17 + HW-095 (L298N)
// CH2: 12V ACTUATOR + HW-039 (BTS7960)
// CH3: DAC + RELAY
//
// PlatformIO / Arduino Core 2.x
// =====================================================

// IMPORTANT:
// L298N does not regulate the stepper phase current.
// Keep disabled until current has been checked.
#define ENABLE_STEPPER_POWER 1
#define ENABLE_ACTUATOR_POWER 1

// =====================================================
// GPIO
// =====================================================

// RC receiver
constexpr uint8_t RC1_PIN = 27;
constexpr uint8_t RC2_PIN = 34;
constexpr uint8_t RC3_PIN = 35;

// HW-095
constexpr uint8_t IN1 = 16;
constexpr uint8_t IN2 = 17;
constexpr uint8_t IN3 = 18;
constexpr uint8_t IN4 = 19;
constexpr uint8_t ENA = 21;
constexpr uint8_t ENB = 26;

// HW-039
constexpr uint8_t RPWM = 32;
constexpr uint8_t LPWM = 33;
constexpr uint8_t R_EN = 23;
constexpr uint8_t L_EN = 22;

// DAC / relay
constexpr uint8_t DAC_PIN = 25;
constexpr uint8_t RELAY_PIN = 13;

// Change these if using an active-low relay module
constexpr uint8_t RELAY_ON = HIGH;
constexpr uint8_t RELAY_OFF = LOW;

// =====================================================
// RC SETTINGS
// =====================================================

constexpr uint16_t RC_CAPTURE_MIN = 800;
constexpr uint16_t RC_CAPTURE_MAX = 2500;

constexpr uint16_t RC_MIN = 900;
constexpr uint16_t RC_MAX = 2400;

constexpr uint16_t RC_CENTER = 1500;
constexpr uint16_t RC_DEADBAND = 50;

// Signal timeout: 200 ms
constexpr uint32_t RC_TIMEOUT_US = 200000UL;

// =====================================================
// STEPPER SETTINGS
// =====================================================

constexpr int32_t STEPS_PER_TURN = 200;

constexpr float STEPPER_MAX_SPEED = 60.0f;
constexpr float STEPPER_ACCEL = 30.0f;

constexpr bool STEPPER_RELEASE_IDLE = true;

// =====================================================
// ACTUATOR SETTINGS
// =====================================================

constexpr uint32_t ACTUATOR_MAX_RUN_US =
    10000000UL;

constexpr uint32_t ACTUATOR_REVERSE_PAUSE_US =
    120000UL;

constexpr float ACTUATOR_RAMP = 300.0f;

constexpr uint8_t PWM_R_CH = 0;
constexpr uint8_t PWM_L_CH = 1;

constexpr uint32_t PWM_FREQ = 16000;
constexpr uint8_t PWM_BITS = 8;

// =====================================================
// DAC SETTINGS
// =====================================================

constexpr float DAC_RAMP = 170.0f;

constexpr uint32_t RELAY_PAUSE_US =
    150000UL;

// =====================================================
// RC INPUT STRUCTURE
// =====================================================

struct RcChannel {
    volatile uint32_t rise = 0;
    volatile uint32_t last = 0;

    volatile uint16_t width = RC_CENTER;

    volatile uint32_t accepted = 0;
    volatile uint32_t rejected = 0;
};

RcChannel rc1;
RcChannel rc2;
RcChannel rc3;

portMUX_TYPE rcMux =
    portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// RC INTERRUPTS
// =====================================================

void IRAM_ATTR captureRc(
    RcChannel &ch,
    uint8_t pin
) {
    uint32_t now = micros();

    if (digitalRead(pin) == HIGH) {
        ch.rise = now;
        return;
    }

    uint32_t width = now - ch.rise;

    portENTER_CRITICAL_ISR(&rcMux);

    if (width >= RC_CAPTURE_MIN &&
        width <= RC_CAPTURE_MAX) {

        ch.width = (uint16_t)width;
        ch.last = now;
        ch.accepted++;

    } else {
        ch.rejected++;
    }

    portEXIT_CRITICAL_ISR(&rcMux);
}

void IRAM_ATTR rc1ISR() {
    captureRc(rc1, RC1_PIN);
}

void IRAM_ATTR rc2ISR() {
    captureRc(rc2, RC2_PIN);
}

void IRAM_ATTR rc3ISR() {
    captureRc(rc3, RC3_PIN);
}

// =====================================================
// RC SNAPSHOT
// =====================================================

struct RcSnapshot {
    uint16_t us;
    bool valid;

    uint32_t ageMs;

    uint32_t accepted;
    uint32_t rejected;
};

// FIXED:
// Read timestamp AFTER copying channel information.
// Prevents false LOST when a new interrupt updates
// last between obtaining now and reading last.

RcSnapshot readRc(RcChannel &ch) {
    uint16_t width;
    uint32_t last;
    uint32_t accepted;
    uint32_t rejected;

    portENTER_CRITICAL(&rcMux);

    width = ch.width;
    last = ch.last;

    accepted = ch.accepted;
    rejected = ch.rejected;

    portEXIT_CRITICAL(&rcMux);

    // Critical fix
    const uint32_t sampleNow = micros();

    if (last == 0) {
        return {
            width,
            false,
            UINT32_MAX,
            accepted,
            rejected
        };
    }

    // Unsigned subtraction also handles micros() wrap.
    const uint32_t ageUs = sampleNow - last;

    return {
        width,
        ageUs <= RC_TIMEOUT_US,
        ageUs / 1000UL,
        accepted,
        rejected
    };
}

// =====================================================
// COMMON FUNCTIONS
// =====================================================

bool inNeutral(uint16_t us) {
    return abs((int)us - (int)RC_CENTER)
        <= RC_DEADBAND;
}

float rcCommand(uint16_t us) {
    int delta = (int)us - RC_CENTER;

    if (abs(delta) <= RC_DEADBAND) {
        return 0.0f;
    }

    float range = delta < 0
        ? float(RC_CENTER - RC_MIN - RC_DEADBAND)
        : float(RC_MAX - RC_CENTER - RC_DEADBAND);

    float value =
        float(abs(delta) - RC_DEADBAND) / range;

    value = constrain(value, 0.0f, 1.0f);

    return delta < 0 ? -value : value;
}

float rampTo(
    float current,
    float target,
    float maxChange
) {
    float delta = target - current;

    if (fabsf(delta) <= maxChange) {
        return target;
    }

    return current +
        (delta > 0 ? maxChange : -maxChange);
}

// =====================================================
// CH1 - STEPPER POSITION CONTROL
// =====================================================

bool stepperReference = false;
bool stepperPowered = false;

int32_t stepperPosition = 0;
int32_t stepperTarget = 0;

float stepperSpeed = 0;

uint8_t stepPhase = 0;

uint32_t lastStepUs = 0;

void stepperPower(bool enable) {
    stepperPowered =
        ENABLE_STEPPER_POWER && enable;

    digitalWrite(
        ENA,
        stepperPowered ? HIGH : LOW
    );

    digitalWrite(
        ENB,
        stepperPowered ? HIGH : LOW
    );
}

void setStepPhase(uint8_t phase) {
    static const uint8_t table[4][4] = {
        {1, 0, 1, 0},
        {0, 1, 1, 0},
        {0, 1, 0, 1},
        {1, 0, 0, 1}
    };

    const uint8_t pins[4] = {
        IN1, IN2, IN3, IN4
    };

    for (int i = 0; i < 4; i++) {
        digitalWrite(
            pins[i],
            table[phase & 3][i]
        );
    }
}

int32_t targetSteps(uint16_t us) {
    // 1000us = 0 degrees
    // 1500us = 180 degrees
    // 2000us = 360 degrees

    int bounded = constrain(
        (int)us,
        1000,
        2000
    );

    return (
        (bounded - 1000) *
        STEPS_PER_TURN + 500
    ) / 1000;
}

void updateStepper(
    const RcSnapshot &rc,
    uint32_t now,
    float dt
) {
    if (!rc.valid) {
        stepperReference = false;

        stepperPosition = 0;
        stepperTarget = 0;
        stepperSpeed = 0;

        stepperPower(false);
        lastStepUs = now;
        return;
    }

    if (!stepperReference) {
        stepperReference = true;

        // Relative zero at current shaft position
        stepperPosition = 0;
        stepperTarget = 0;
        stepperSpeed = 0;
        stepPhase = 0;

        setStepPhase(stepPhase);
        lastStepUs = now;

        Serial.println(
            "CH1: RELATIVE ZERO INITIALIZED"
        );
    }

    stepperTarget = targetSteps(rc.us);

    if (!ENABLE_STEPPER_POWER) {
        stepperSpeed = 0;
        stepperPower(false);
        return;
    }

    int32_t remaining =
        stepperTarget - stepperPosition;

    if (remaining == 0 &&
        fabsf(stepperSpeed) < 0.5f) {

        stepperSpeed = 0;

        stepperPower(
            !STEPPER_RELEASE_IDLE
        );

        lastStepUs = now;
        return;
    }

    int direction =
        remaining > 0 ? 1 :
        remaining < 0 ? -1 : 0;

    float brakingSpeed = sqrtf(
        2.0f *
        STEPPER_ACCEL *
        fabsf((float)remaining)
    );

    float wanted =
        direction * fminf(
            STEPPER_MAX_SPEED,
            brakingSpeed
        );

    stepperSpeed = rampTo(
        stepperSpeed,
        wanted,
        STEPPER_ACCEL * dt
    );

    if (remaining == 0 ||
        fabsf(stepperSpeed) < 0.5f) {

        if (remaining == 0) {
            stepperPower(false);
        }

        lastStepUs = now;
        return;
    }

    int actualDirection =
        stepperSpeed > 0 ? 1 : -1;

    if (actualDirection != direction) {
        lastStepUs = now;
        return;
    }

    stepperPower(true);

    uint32_t interval =
        (uint32_t)(
            1000000.0f /
            fabsf(stepperSpeed)
        );

    if ((uint32_t)(now - lastStepUs)
        >= interval) {

        lastStepUs = now;

        stepPhase = (
            stepPhase +
            (actualDirection > 0 ? 1 : 3)
        ) & 3;

        setStepPhase(stepPhase);

        stepperPosition += actualDirection;
    }
}

// =====================================================
// CH2 - ACTUATOR
// =====================================================

enum ActuatorFault {
    ACT_WAIT_NEUTRAL,
    ACT_OK,
    ACT_SIGNAL_LOST,
    ACT_TIMEOUT
};

ActuatorFault actuatorFault =
    ACT_WAIT_NEUTRAL;

bool actuatorArmed = false;
bool actuatorTimedOut = false;
bool actuatorRunStarted = false;

bool actuatorPreviousSignalOK = false;

uint32_t actuatorSignalLossCount = 0;

float actuatorDuty = 0;

int actuatorLastDirection = 0;

uint32_t actuatorRunStartUs = 0;
uint32_t actuatorReverseReadyAt = 0;

// =====================================================
// HW-039 MOTOR OUTPUT
// =====================================================

void actuatorStop() {
    ledcWrite(PWM_R_CH, 0);
    ledcWrite(PWM_L_CH, 0);

    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
}

void actuatorOutput(float duty) {
    if (!ENABLE_ACTUATOR_POWER ||
        fabsf(duty) < 0.5f) {

        actuatorStop();
        return;
    }

    uint8_t pwm = constrain(
        (int)lroundf(fabsf(duty)),
        0,
        255
    );

    // Never drive both directions simultaneously
    ledcWrite(PWM_R_CH, 0);
    ledcWrite(PWM_L_CH, 0);

    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    if (duty > 0) {
        ledcWrite(PWM_R_CH, pwm);
    } else {
        ledcWrite(PWM_L_CH, pwm);
    }
}

// =====================================================
// CH2 CONTROL
// =====================================================

void updateActuator(
    const RcSnapshot &rc,
    uint32_t now,
    float dt
) {
    // -----------------------------------------------
    // SIGNAL LOSS
    // -----------------------------------------------

    if (!rc.valid) {
        if (actuatorPreviousSignalOK) {
            actuatorSignalLossCount++;

            Serial.printf(
                "CH2 SIGNAL LOST count=%lu age=%lu ms\n",
                (unsigned long)actuatorSignalLossCount,
                (unsigned long)rc.ageMs
            );
        }

        actuatorPreviousSignalOK = false;

        actuatorArmed = false;
        actuatorRunStarted = false;

        actuatorDuty = 0;
        actuatorLastDirection = 0;

        actuatorFault = ACT_SIGNAL_LOST;

        actuatorReverseReadyAt =
            now + ACTUATOR_REVERSE_PAUSE_US;

        actuatorStop();
        return;
    }

    // -----------------------------------------------
    // SIGNAL RESTORED
    // -----------------------------------------------

    if (!actuatorPreviousSignalOK) {
        Serial.println(
            "CH2 SIGNAL RESTORED - WAIT NEUTRAL"
        );

        actuatorFault = ACT_WAIT_NEUTRAL;
    }

    actuatorPreviousSignalOK = true;

    const bool neutral = inNeutral(rc.us);

    // -----------------------------------------------
    // ARM AFTER NEUTRAL
    // -----------------------------------------------

    if (neutral) {
        actuatorArmed = true;
        actuatorTimedOut = false;
        actuatorRunStarted = false;

        actuatorFault = ACT_OK;
    }

    // -----------------------------------------------
    // START RUN TIMER
    // -----------------------------------------------

    if (!neutral &&
        actuatorArmed &&
        !actuatorTimedOut &&
        !actuatorRunStarted) {

        actuatorRunStarted = true;
        actuatorRunStartUs = now;
    }

    // -----------------------------------------------
    // 10 SECOND LIMIT
    // -----------------------------------------------

    if (actuatorRunStarted &&
        (uint32_t)(now - actuatorRunStartUs)
            >= ACTUATOR_MAX_RUN_US) {

        actuatorTimedOut = true;
        actuatorArmed = false;
        actuatorRunStarted = false;

        actuatorDuty = 0;
        actuatorLastDirection = 0;

        actuatorFault = ACT_TIMEOUT;

        actuatorStop();

        Serial.println(
            "CH2 TIMEOUT 10 SEC"
        );

        return;
    }

    // -----------------------------------------------
    // BLOCKED
    // -----------------------------------------------

    if (!actuatorArmed ||
        actuatorTimedOut) {

        actuatorDuty = 0;
        actuatorLastDirection = 0;

        actuatorStop();
        return;
    }

    // -----------------------------------------------
    // MOTOR COMMAND
    // -----------------------------------------------

    float desired = neutral
        ? 0.0f
        : rcCommand(rc.us) * 255.0f;

    // Decelerate before reversing
    if (actuatorDuty * desired < 0) {
        desired = 0;
    }

    actuatorDuty = rampTo(
        actuatorDuty,
        desired,
        ACTUATOR_RAMP * dt
    );

    if (fabsf(actuatorDuty) < 0.5f) {
        actuatorDuty = 0;
    }

    int direction =
        actuatorDuty > 0 ? 1 :
        actuatorDuty < 0 ? -1 : 0;

    // -----------------------------------------------
    // STOP
    // -----------------------------------------------

    if (direction == 0) {
        actuatorStop();

        if (actuatorLastDirection != 0) {
            actuatorReverseReadyAt =
                now + ACTUATOR_REVERSE_PAUSE_US;
        }

        actuatorLastDirection = 0;
        return;
    }

    // -----------------------------------------------
    // REVERSAL PAUSE
    // -----------------------------------------------

    if (actuatorLastDirection == 0 &&
        (int32_t)(now - actuatorReverseReadyAt)
            < 0) {

        actuatorStop();
        return;
    }

    actuatorLastDirection = direction;

    actuatorOutput(actuatorDuty);
}

// =====================================================
// CH3 - DAC AND RELAY
// =====================================================

bool analogArmed = false;

float dacLevel = 0;

int dacDirection = 0;

uint32_t relayReadyAt = 0;

int analogDirection(uint16_t us) {
    if (us <= 1440) {
        return -1;
    }

    if (us >= 1550) {
        return 1;
    }

    return 0;
}

float analogMagnitude(uint16_t us) {
    if (us <= 1440) {
        return constrain(
            (1440.0f - us) / 540.0f,
            0.0f,
            1.0f
        );
    }

    if (us >= 1550) {
        return constrain(
            (float(us) - 1550.0f) / 850.0f,
            0.0f,
            1.0f
        );
    }

    return 0.0f;
}

void analogOff() {
    dacWrite(DAC_PIN, 0);

    dacLevel = 0;
    dacDirection = 0;

    digitalWrite(
        RELAY_PIN,
        RELAY_OFF
    );
}

void updateAnalog(
    const RcSnapshot &rc,
    uint32_t now,
    float dt
) {
    if (!rc.valid) {
        analogArmed = false;

        analogOff();

        relayReadyAt =
            now + RELAY_PAUSE_US;

        return;
    }

    int wantedDir =
        analogDirection(rc.us);

    // Arm only after neutral
    if (wantedDir == 0) {
        analogArmed = true;
    }

    if (!analogArmed) {
        analogOff();
        return;
    }

    bool changingDir =
        wantedDir != 0 &&
        wantedDir != dacDirection;

    float target =
        changingDir || wantedDir == 0
        ? 0.0f
        : 255.0f * analogMagnitude(rc.us);

    dacLevel = rampTo(
        dacLevel,
        target,
        DAC_RAMP * dt
    );

    if (dacLevel < 0.5f) {
        dacLevel = 0;
    }

    // Switch relay only when DAC is zero
    if (dacLevel == 0 &&
        wantedDir != dacDirection) {

        dacDirection = wantedDir;

        digitalWrite(
            RELAY_PIN,
            dacDirection < 0
                ? RELAY_ON
                : RELAY_OFF
        );

        relayReadyAt =
            now + RELAY_PAUSE_US;
    }

    // Relay settling delay
    if ((int32_t)(now - relayReadyAt) < 0) {
        dacLevel = 0;
    }

    dacWrite(
        DAC_PIN,
        (uint8_t)constrain(
            (int)lroundf(dacLevel),
            0,
            255
        )
    );
}

// =====================================================
// DIAGNOSTICS
// =====================================================

const char* actuatorFaultText() {
    switch (actuatorFault) {
        case ACT_WAIT_NEUTRAL:
            return "WAIT_NEUTRAL";

        case ACT_OK:
            return "OK";

        case ACT_SIGNAL_LOST:
            return "SIGNAL_LOST";

        case ACT_TIMEOUT:
            return "TIMEOUT_10S";
    }

    return "UNKNOWN";
}

void printRc(
    const char* name,
    const RcSnapshot &rc
) {
    Serial.printf(
        "%s: %u us %s age=",
        name,
        rc.us,
        rc.valid ? "OK" : "LOST"
    );

    if (rc.ageMs == UINT32_MAX) {
        Serial.print("NEVER");
    } else {
        Serial.printf(
            "%lu ms",
            (unsigned long)rc.ageMs
        );
    }

    Serial.printf(
        " accepted=%lu rejected=%lu\n",
        (unsigned long)rc.accepted,
        (unsigned long)rc.rejected
    );
}

void printDiagnostics(
    const RcSnapshot &s,
    const RcSnapshot &a,
    const RcSnapshot &d
) {
    Serial.println();

    Serial.println(
        "========== ESP32 DIAGNOSTICS =========="
    );

    printRc("CH1 GPIO27", s);
    printRc("CH2 GPIO34", a);
    printRc("CH3 GPIO35", d);

    Serial.printf(
        "STEPPER: ref=%d pos=%ld target=%ld "
        "angle=%.1f/%.1f power=%d\n",
        (int)stepperReference,
        (long)stepperPosition,
        (long)stepperTarget,
        stepperPosition * 1.8f,
        stepperTarget * 1.8f,
        (int)stepperPowered
    );

    Serial.printf(
        "ACTUATOR: armed=%d pwm=%.1f "
        "timeout=%d timer=%d fault=%s "
        "signal_losses=%lu\n",
        (int)actuatorArmed,
        actuatorDuty,
        (int)actuatorTimedOut,
        (int)actuatorRunStarted,
        actuatorFaultText(),
        (unsigned long)actuatorSignalLossCount
    );

    Serial.printf(
        "ANALOG: armed=%d DAC=%u/255 "
        "relay=%s\n",
        (int)analogArmed,
        (unsigned)lroundf(dacLevel),
        dacDirection < 0 ? "ON" : "OFF"
    );

    Serial.println(
        "======================================="
    );
}

// =====================================================
// SETUP
// =====================================================

void setup() {
    Serial.begin(115200);

    // DAC and relay
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_OFF);

    dacWrite(DAC_PIN, 0);

    // Stepper
    const uint8_t stepPins[] = {
        IN1, IN2, IN3, IN4, ENA, ENB
    };

    for (uint8_t pin : stepPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    stepperPower(false);
    setStepPhase(0);

    // HW-039
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);

    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);

    ledcSetup(
        PWM_R_CH,
        PWM_FREQ,
        PWM_BITS
    );

    ledcSetup(
        PWM_L_CH,
        PWM_FREQ,
        PWM_BITS
    );

    ledcAttachPin(RPWM, PWM_R_CH);
    ledcAttachPin(LPWM, PWM_L_CH);

    actuatorStop();

    // RC receiver
    pinMode(RC1_PIN, INPUT);
    pinMode(RC2_PIN, INPUT);
    pinMode(RC3_PIN, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(RC1_PIN),
        rc1ISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(RC2_PIN),
        rc2ISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(RC3_PIN),
        rc3ISR,
        CHANGE
    );

    Serial.println(
        "ESP32 3-CHANNEL CONTROLLER READY"
    );

    Serial.println(
        "CH1: STEPPER POSITION 0..360"
    );

    Serial.println(
        "CH2: ACTUATOR 10 SECOND LIMIT"
    );

    Serial.println(
        "CH3: DAC + RELAY"
    );

    Serial.println(
        "RC INPUT RANGE: 800..2500 US"
    );

    Serial.println(
        "RC TIMEOUT: 200 MS"
    );

    Serial.println(
        "RC TIMESTAMP RACE FIX ENABLED"
    );

    Serial.printf(
        "STEPPER POWER: %s\n",
        ENABLE_STEPPER_POWER
            ? "ENABLED"
            : "DISABLED"
    );

    Serial.printf(
        "ACTUATOR POWER: %s\n",
        ENABLE_ACTUATOR_POWER
            ? "ENABLED"
            : "DISABLED"
    );
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
    static uint32_t lastLoopUs = micros();
    static uint32_t lastPrintMs = 0;

    uint32_t now = micros();

    float dt =
        (uint32_t)(now - lastLoopUs)
        * 0.000001f;

    dt = constrain(dt, 0.0f, 0.02f);

    lastLoopUs = now;

    // Fixed: each snapshot reads its own time
    // after copying the ISR data.
    RcSnapshot s = readRc(rc1);
    RcSnapshot a = readRc(rc2);
    RcSnapshot d = readRc(rc3);

    updateStepper(s, now, dt);
    updateActuator(a, now, dt);
    updateAnalog(d, now, dt);

    if (millis() - lastPrintMs >= 500) {
        lastPrintMs = millis();

        printDiagnostics(s, a, d);
    }
}
