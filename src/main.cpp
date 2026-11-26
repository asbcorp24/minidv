
#include <Arduino.h>
#include <math.h>

// =====================================================
// ESP32-WROOM-32 DEVKIT
//
// CH1: NEMA17 + DM556 (continuous rotation)
// CH2: 12V ACTUATOR + HW-039 (BTS7960)
// CH3: DAC + RELAY
//
// PlatformIO / Arduino Core 2.x
// =====================================================

// Set the DM556 phase current and connect a non-inverting 3.3 V to 5 V interface.
#define ENABLE_STEPPER_POWER 1
#define ENABLE_ACTUATOR_POWER 1

// =====================================================
// GPIO
// =====================================================

// RC receiver
constexpr uint8_t RC1_PIN = 27;
constexpr uint8_t RC2_PIN = 34;
constexpr uint8_t RC3_PIN = 35;

// DM556 through a non-inverting 5 V control interface
constexpr uint8_t STEPPER_STEP_PIN = 16;
constexpr uint8_t STEPPER_DIR_PIN = 17;
constexpr uint8_t STEPPER_ENABLE_PIN = 21;

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

constexpr uint16_t ACTUATOR_RC_MIN = 1000;
constexpr uint16_t ACTUATOR_RC_MAX = 2000;

constexpr uint16_t RC_CENTER = 1500;
constexpr uint16_t RC_DEADBAND = 50;

// Signal timeout: 200 ms
constexpr uint32_t RC_TIMEOUT_US = 200000UL;

// =====================================================
// STEPPER SETTINGS
// =====================================================

constexpr uint16_t STEPPER_RC_MIN = 1000;
constexpr uint16_t STEPPER_RC_MAX = 2000;
constexpr uint16_t STEPPER_RC_REVERSE = 1400;
constexpr uint16_t STEPPER_RC_FORWARD = 1600;

constexpr uint32_t STEPPER_PULSE_US = 5;
constexpr uint32_t STEPPER_DIR_SETUP_US = 10;
constexpr uint32_t STEPPER_ENABLE_DELAY_US = 300000UL;

constexpr float STEPPER_MAX_SPEED = 60.0f;
constexpr float STEPPER_ACCEL = 30.0f;


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
    // CH2: saturate accepted RC pulses at the calibrated endpoints.
    const uint16_t bounded = constrain(
        us, ACTUATOR_RC_MIN, ACTUATOR_RC_MAX
    );
    int delta = (int)bounded - RC_CENTER;

    if (abs(delta) <= RC_DEADBAND) {
        return 0.0f;
    }

    float range = delta < 0
        ? float(RC_CENTER - ACTUATOR_RC_MIN - RC_DEADBAND)
        : float(ACTUATOR_RC_MAX - RC_CENTER - RC_DEADBAND);

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
// CH1 - STEPPER CONTINUOUS ROTATION
// =====================================================

bool stepperPowered = false;
int stepperCommandDirection = 0;
float stepperSpeed = 0;

uint32_t lastStepUs = 0;
uint32_t stepperEnabledAtUs = 0;

void stepperPower(bool enable) {
    stepperPowered =
        ENABLE_STEPPER_POWER && enable;

    // Non-inverting 5 V interface: ENA HIGH disables the DM556.
    digitalWrite(STEPPER_ENABLE_PIN, stepperPowered ? LOW : HIGH);
}

void pulseStepper(int direction) {
    digitalWrite(STEPPER_DIR_PIN, direction > 0 ? HIGH : LOW);
    delayMicroseconds(STEPPER_DIR_SETUP_US);
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(STEPPER_PULSE_US);
    digitalWrite(STEPPER_STEP_PIN, LOW);
}

int stepperDirection(uint16_t us) {
    if (us < STEPPER_RC_MIN || us > STEPPER_RC_MAX) {
        return 0;
    }

    if (us > STEPPER_RC_FORWARD) {
        return 1;
    }

    if (us < STEPPER_RC_REVERSE) {
        return -1;
    }

    return 0;
}

void updateStepper(
    const RcSnapshot &rc,
    uint32_t now,
    float dt
) {
    stepperCommandDirection = rc.valid && ENABLE_STEPPER_POWER
        ? stepperDirection(rc.us)
        : 0;

    // Stop pulses immediately in neutral, out of range, or on signal loss.
    if (stepperCommandDirection == 0) {
        stepperSpeed = 0;
        stepperPower(false);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        lastStepUs = now;
        return;
    }

    if (!stepperPowered) {
        stepperPower(true);
        stepperEnabledAtUs = now;
        lastStepUs = now;
    }

    // Wait without blocking CH2/CH3 while the DM556 enables its outputs.
    if ((uint32_t)(now - stepperEnabledAtUs) < STEPPER_ENABLE_DELAY_US) {
        return;
    }

    float wanted = stepperCommandDirection * STEPPER_MAX_SPEED;

    // Decelerate to zero before issuing pulses in the opposite direction.
    if (stepperSpeed * wanted < 0) {
        wanted = 0;
    }

    stepperSpeed = rampTo(
        stepperSpeed,
        wanted,
        STEPPER_ACCEL * dt
    );

    if (fabsf(stepperSpeed) < 0.5f) {
        lastStepUs = now;
        return;
    }

    const uint32_t interval =
        (uint32_t)(1000000.0f / fabsf(stepperSpeed));

    if ((uint32_t)(now - lastStepUs) >= interval) {
        lastStepUs = now;
        pulseStepper(stepperSpeed > 0 ? 1 : -1);
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

    // Preserve fractional duty so fast loop iterations can accumulate.
    // rampTo() reaches zero exactly when stopping or reversing.

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

    // Preserve fractional DAC levels until the hardware output is rounded.
    // rampTo() reaches zero exactly before the relay changes direction.

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
        "STEPPER DM556: command=%d speed=%.1f pulses/s power=%d\n",
        stepperCommandDirection,
        stepperSpeed,
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
    // Set the output latch while the external pull-up holds EN high.
    pinMode(STEPPER_ENABLE_PIN, INPUT);
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);
    pinMode(STEPPER_ENABLE_PIN, OUTPUT);
    stepperPower(false);

    pinMode(STEPPER_STEP_PIN, OUTPUT);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    digitalWrite(STEPPER_DIR_PIN, LOW);

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
        "CH1: DM556 1000..1399 REVERSE, 1400..1600 STOP, 1601..2000 FORWARD"
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
