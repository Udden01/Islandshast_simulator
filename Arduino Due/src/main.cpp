#include <Arduino.h>

// ==================================================
// Motor class
// ==================================================
class motor {
public:
  // Pins
  int pwm;
  int dir;
  int chn1;
  int chn2;

  // Calibration
  float pulserPerMm = 0.0f;

  // Encoder state
  volatile long pulseRaknare = 0;
  volatile uint8_t prevState = 0;

  long skyddadRaknare = 0;
  long tidigareKontrollRaknare = 0;

  // Measured velocity
  float hastighetPulserPerS = 0.0f;
  float filtreradHastighetMmS = 0.0f;

  // Target/debug
  long malPos = 0;

  // Motor command state
  bool iRorelse = false;
  int aktuellPWM = 0;

  // Controller integrators
  float integralFelMm = 0.0f;
  float integralHastighetsFel = 0.0f;

  long tidigareMalPos = -1;
};

// ==================================================
// Motor objects
// ==================================================
motor A;
motor B;
motor C;
motor D;
motor E;
motor F;

static const uint8_t antalMotorer = 6;
motor* allaMotorer[antalMotorer] = {&A, &B, &C, &D, &E, &F};

// ==================================================
// Options
// ==================================================
bool move_utanreglerig = false;

// Fine trim per motor in mm
static const int motorTrimMm[antalMotorer] = {0, 0, 0, 0, 0, 0};

// ==================================================
// Calibration limits
// ==================================================
long maxPulserPerMotor[antalMotorer] = {0};
long minPulserPerMotor[antalMotorer] = {0};

// ==================================================
// Controller constants
// ==================================================
constexpr unsigned long kontrollperiodUs = 10000; // 10 ms = 100 Hz

// Outer position loop:
// position error [mm] -> velocity correction [mm/s]
constexpr float kpPositionTillHastighet = 5.0f;

// Inner velocity loop:
// velocity error [mm/s] -> PWM
constexpr float kpHastighet = 4.0f;
constexpr float kiHastighet = 20.0f;

// Velocity low-pass filter
constexpr float hastighetsFilterAlpha = 0.25f;

// Limits
constexpr float maxHastighetMmS = 80.0f;
constexpr float maxIntegralHastighet = 30.0f;

constexpr int maxPWM = 240;
constexpr int minPWM = 12;

// Static friction compensation
constexpr int friktionsPWM = 18;

// Stop behavior
constexpr float positionToleransMm = 0.5f;
constexpr float hastighetToleransMmS = 2.0f;

// Safety limits in actuator mm
constexpr float maxPositionMm = 100.0f;
constexpr float minPositionMm = 0.0f;

// PWM ramping
constexpr int pwmStegPerCykel = 8;

// Timeout if no new packet arrives
constexpr unsigned long packetTimeoutMs = 500;

// ==================================================
// Packet settings
// ==================================================
static const uint8_t antalPaketAktuatorer = 6;
static const uint8_t sync1 = 0xAA;
static const uint8_t sync2 = 0x55;
static const uint8_t KalibreringKom = 'C';

// 2 sync + 2 seq + 2 dt + 12 pos + 12 vel + 1 checksum = 31 bytes
static const uint8_t paketStorlek = 7 + (4 * antalPaketAktuatorer);

static bool kalibreringKlar = false;

struct AktuatorPaket {
  uint16_t sekvens;
  uint16_t dt_ms;
  int16_t positionerMm[antalPaketAktuatorer];
  int16_t hastigheterMmS[antalPaketAktuatorer];
};

uint8_t rxBuffer[paketStorlek];
uint8_t rxIndex = 0;

uint32_t mottagenPaketAntal = 0;
uint32_t daligPaketAntal = 0;
uint16_t senastMottagnSekvens = 0;

unsigned long senasteDebugPrintUs = 0;
unsigned long senastPaketMs = 0;

// Latest received targets
int malPosisjonerMm[antalMotorer] = {100, 100, 100, 100, 100, 100};
int malHastigheterMmS[antalMotorer] = {0, 0, 0, 0, 0, 0};

// ==================================================
// Encoder interrupts
// ==================================================
void uppdateraEncoderRaknare(motor &Motor) {
  uint8_t state = (digitalRead(Motor.chn1) << 1) | digitalRead(Motor.chn2);
  uint8_t combined = (Motor.prevState << 2) | state;

  switch (combined) {
    // Forward
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      Motor.pulseRaknare--;
      break;

    // Backward
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      Motor.pulseRaknare++;
      break;

    default:
      // Invalid transition / noise
      break;
  }

  Motor.prevState = state;
}

void irqA1() { uppdateraEncoderRaknare(A); }
void irqA2() { uppdateraEncoderRaknare(A); }

void irqB1() { uppdateraEncoderRaknare(B); }
void irqB2() { uppdateraEncoderRaknare(B); }

void irqC1() { uppdateraEncoderRaknare(C); }
void irqC2() { uppdateraEncoderRaknare(C); }

void irqD1() { uppdateraEncoderRaknare(D); }
void irqD2() { uppdateraEncoderRaknare(D); }

void irqE1() { uppdateraEncoderRaknare(E); }
void irqE2() { uppdateraEncoderRaknare(E); }

void irqF1() { uppdateraEncoderRaknare(F); }
void irqF2() { uppdateraEncoderRaknare(F); }

// ==================================================
// Encoder copy and velocity update
// ==================================================
void updateraSkyddadRaknare() {
  noInterrupts();
  for (uint8_t i = 0; i < antalMotorer; i++) {
    allaMotorer[i]->skyddadRaknare = allaMotorer[i]->pulseRaknare;
  }
  interrupts();
}

void updateraMotorHastighet(motor &Motor, float dt) {
  if (dt <= 0.000001f) {
    return;
  }

  long nuvarande = Motor.skyddadRaknare;
  long deltaPulser = nuvarande - Motor.tidigareKontrollRaknare;

  Motor.hastighetPulserPerS = deltaPulser / dt;
  Motor.tidigareKontrollRaknare = nuvarande;

  if (Motor.pulserPerMm > 0.0f) {
    float rawHastighetMmS = Motor.hastighetPulserPerS / Motor.pulserPerMm;

    Motor.filtreradHastighetMmS =
        hastighetsFilterAlpha * rawHastighetMmS +
        (1.0f - hastighetsFilterAlpha) * Motor.filtreradHastighetMmS;
  } else {
    Motor.filtreradHastighetMmS = 0.0f;
  }
}

// ==================================================
// Simple movement mode, kept only for comparison
// ==================================================
void move(motor &Motor, int malMm) {
  if (Motor.pulserPerMm <= 0.0f) {
    analogWrite(Motor.pwm, 0);
    return;
  }

  int begransadMalMm = constrain(malMm, 0, 100);
  long malPulser = (long)(begransadMalMm * Motor.pulserPerMm);

  long nuvarandePulser = Motor.skyddadRaknare;

  long motorIndex = -1;
  for (uint8_t i = 0; i < antalMotorer; i++) {
    if (allaMotorer[i] == &Motor) {
      motorIndex = i;
      break;
    }
  }

  long maxPulserLimit =
      (motorIndex >= 0 && maxPulserPerMotor[motorIndex] > 0)
      ? maxPulserPerMotor[motorIndex]
      : (long)(100.0f * Motor.pulserPerMm);

  if (nuvarandePulser > maxPulserLimit || nuvarandePulser < 0) {
    Motor.iRorelse = false;
    Motor.aktuellPWM = 0;
    analogWrite(Motor.pwm, 0);
    return;
  }

  if (motorIndex >= 0 && maxPulserPerMotor[motorIndex] > 0) {
    malPulser = constrain(malPulser, 0, maxPulserPerMotor[motorIndex]);
  }

  long felPulser = malPulser - nuvarandePulser;

  if (labs(felPulser) <= 5) {
    Motor.iRorelse = false;
    Motor.aktuellPWM = 0;
    analogWrite(Motor.pwm, 0);
    return;
  }

  digitalWrite(Motor.dir, felPulser >= 0 ? HIGH : LOW);
  analogWrite(Motor.pwm, maxPWM);
  Motor.iRorelse = true;
}

// ==================================================
// Cascaded controller
//
// refPosMm  = target actuator position from packet
// refVelMmS = target actuator velocity from packet
//
// Outer loop:
//   position error -> velocity correction
//
// Inner loop:
//   velocity error -> PWM
// ==================================================
void moveCascade(motor &Motor, float refPosMm, float refVelMmS, float dt) {
  if (Motor.pulserPerMm <= 0.0f) {
    analogWrite(Motor.pwm, 0);
    Motor.iRorelse = false;
    return;
  }

  long motorIndex = -1;
  for (uint8_t i = 0; i < antalMotorer; i++) {
    if (allaMotorer[i] == &Motor) {
      motorIndex = i;
      break;
    }
  }

  long nuvarandePulser = Motor.skyddadRaknare;
  float posMm = nuvarandePulser / Motor.pulserPerMm;

  long maxPulserLimit =
      (motorIndex >= 0 && maxPulserPerMotor[motorIndex] > 0)
      ? maxPulserPerMotor[motorIndex]
      : (long)(maxPositionMm * Motor.pulserPerMm);

  // Hard safety stop
  if (nuvarandePulser < 0 || nuvarandePulser > maxPulserLimit) {
    analogWrite(Motor.pwm, 0);
    Motor.integralHastighetsFel = 0.0f;
    Motor.aktuellPWM = 0;
    Motor.iRorelse = false;
    return;
  }

  int motorTrim = (motorIndex >= 0) ? motorTrimMm[motorIndex] : 0;
  refPosMm = constrain(refPosMm + motorTrim, minPositionMm, maxPositionMm);

  float posFelMm = refPosMm - posMm;

  // Velocity feed-forward from Python plus position correction from Arduino
  float velRefMmS = refVelMmS + kpPositionTillHastighet * posFelMm;
  velRefMmS = constrain(velRefMmS, -maxHastighetMmS, maxHastighetMmS);

  // Do not command further into mechanical limits
  if (posMm <= minPositionMm + 1.0f && velRefMmS < 0.0f) {
    velRefMmS = 0.0f;
  }

  if (posMm >= maxPositionMm - 1.0f && velRefMmS > 0.0f) {
    velRefMmS = 0.0f;
  }

  float velMmS = Motor.filtreradHastighetMmS;
  float velFelMmS = velRefMmS - velMmS;

  // Stop only when target is basically stopped
  if (fabs(posFelMm) < positionToleransMm &&
      fabs(refVelMmS) < hastighetToleransMmS &&
      fabs(velMmS) < hastighetToleransMmS) {
    analogWrite(Motor.pwm, 0);
    Motor.integralHastighetsFel = 0.0f;
    Motor.aktuellPWM = 0;
    Motor.iRorelse = false;
    return;
  }

  // Velocity PI loop
  Motor.integralHastighetsFel += velFelMmS * dt;
  Motor.integralHastighetsFel =
      constrain(Motor.integralHastighetsFel,
                -maxIntegralHastighet,
                maxIntegralHastighet);

  float styrsignal =
      kpHastighet * velFelMmS +
      kiHastighet * Motor.integralHastighetsFel;

  // Static friction compensation
  if (fabs(velRefMmS) > 1.0f || fabs(posFelMm) > positionToleransMm) {
    if (styrsignal > 0.0f) {
      styrsignal += friktionsPWM;
    } else if (styrsignal < 0.0f) {
      styrsignal -= friktionsPWM;
    }
  }

  int pwm = abs((int)styrsignal);
  pwm = constrain(pwm, 0, maxPWM);

  if (pwm > 0 && pwm < minPWM) {
    pwm = minPWM;
  }

  // PWM ramp for smoother current changes
  if (pwm > Motor.aktuellPWM) {
    Motor.aktuellPWM = min(Motor.aktuellPWM + pwmStegPerCykel, pwm);
  } else {
    Motor.aktuellPWM = max(Motor.aktuellPWM - pwmStegPerCykel, pwm);
  }

  digitalWrite(Motor.dir, styrsignal >= 0.0f ? HIGH : LOW);
  analogWrite(Motor.pwm, Motor.aktuellPWM);

  Motor.iRorelse = true;
}

// ==================================================
// Calibration
// ==================================================
void kalibrering() {
  delay(200);

  Serial.println("=== CALIBRATION START ===");
  Serial.flush();

  // Move all motors down to bottom
  for (uint8_t i = 0; i < antalMotorer; i++) {
    digitalWrite(allaMotorer[i]->dir, LOW);
    analogWrite(allaMotorer[i]->pwm, 255);
  }

  delay(4000);

  // Stop all motors
  for (uint8_t i = 0; i < antalMotorer; i++) {
    analogWrite(allaMotorer[i]->pwm, 0);
  }

  delay(1000);

  // Zero pulse counters at bottom
  for (uint8_t i = 0; i < antalMotorer; i++) {
    noInterrupts();
    allaMotorer[i]->pulseRaknare = 0;
    allaMotorer[i]->prevState =
        (digitalRead(allaMotorer[i]->chn1) << 1) |
        digitalRead(allaMotorer[i]->chn2);
    interrupts();

    allaMotorer[i]->skyddadRaknare = 0;
    allaMotorer[i]->tidigareKontrollRaknare = 0;
    allaMotorer[i]->filtreradHastighetMmS = 0.0f;
    allaMotorer[i]->hastighetPulserPerS = 0.0f;
  }

  delay(100);

  // Move all motors up to find 100 mm range
  for (uint8_t i = 0; i < antalMotorer; i++) {
    digitalWrite(allaMotorer[i]->dir, HIGH);
    analogWrite(allaMotorer[i]->pwm, 255);
  }

  delay(10000);

  // Stop all motors
  for (uint8_t i = 0; i < antalMotorer; i++) {
    analogWrite(allaMotorer[i]->pwm, 0);
  }

  delay(200);

  updateraSkyddadRaknare();

  for (uint8_t i = 0; i < antalMotorer; i++) {
    maxPulserPerMotor[i] = allaMotorer[i]->skyddadRaknare;
  }

  delay(100);

  for (uint8_t i = 0; i < antalMotorer; i++) {
    if (maxPulserPerMotor[i] == 0) {
      Serial.print("WARNING: Motor ");
      Serial.print((char)('A' + i));
      Serial.println(" has 0 pulse count!");
      allaMotorer[i]->pulserPerMm = 0.0f;
    } else {
      allaMotorer[i]->pulserPerMm = maxPulserPerMotor[i] / 100.0f;
    }

    minPulserPerMotor[i] = 0;

    allaMotorer[i]->malPos = maxPulserPerMotor[i];
    allaMotorer[i]->skyddadRaknare = maxPulserPerMotor[i];
    allaMotorer[i]->tidigareKontrollRaknare = maxPulserPerMotor[i];
    allaMotorer[i]->hastighetPulserPerS = 0.0f;
    allaMotorer[i]->filtreradHastighetMmS = 0.0f;
    allaMotorer[i]->integralFelMm = 0.0f;
    allaMotorer[i]->integralHastighetsFel = 0.0f;
    allaMotorer[i]->tidigareMalPos = -1;
    allaMotorer[i]->iRorelse = false;
    allaMotorer[i]->aktuellPWM = 0;

    // After calibration the actuators are at top, around 100 mm
    malPosisjonerMm[i] = 100;
    malHastigheterMmS[i] = 0;
  }

  senastPaketMs = millis();
  kalibreringKlar = true;

  Serial.println();
  Serial.println("=== CALIBRATION COMPLETE ===");
  Serial.println("Motor pulse resolution:");

  for (uint8_t i = 0; i < antalMotorer; i++) {
    Serial.print("Motor ");
    Serial.print((char)('A' + i));
    Serial.print(": ");
    Serial.print(allaMotorer[i]->pulserPerMm, 2);
    Serial.print(" pulses/mm, max=");
    Serial.print(maxPulserPerMotor[i]);
    Serial.println(" pulses");
  }

  Serial.println("=== CALIBRATION DONE ===");
  Serial.println();

  // Clear serial bytes received during calibration
  while (Serial.available() > 0) {
    Serial.read();
  }

  rxIndex = 0;
}

// ==================================================
// Packet helpers
// ==================================================
uint16_t readU16LE(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

int16_t readI16LE(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint8_t beraknaKontrollSumma(const uint8_t* data, uint8_t langdUtanChecksum) {
  uint16_t sum = 0;

  for (uint8_t i = 0; i < langdUtanChecksum; i++) {
    sum += data[i];
  }

  return (uint8_t)(sum & 0xFF);
}

bool tolkaPaket(const uint8_t* buf, AktuatorPaket& pkt) {
  if (buf[0] != sync1 || buf[1] != sync2) {
    return false;
  }

  uint8_t forvantat = beraknaKontrollSumma(buf, paketStorlek - 1);
  uint8_t mottagit = buf[paketStorlek - 1];

  if (forvantat != mottagit) {
    return false;
  }

  pkt.sekvens = readU16LE(&buf[2]);
  pkt.dt_ms = readU16LE(&buf[4]);

  uint8_t offset = 6;

  for (uint8_t i = 0; i < antalPaketAktuatorer; i++) {
    pkt.positionerMm[i] = readI16LE(&buf[offset]);
    offset += 2;
  }

  for (uint8_t i = 0; i < antalPaketAktuatorer; i++) {
    pkt.hastigheterMmS[i] = readI16LE(&buf[offset]);
    offset += 2;
  }

  return true;
}

// ==================================================
// Process incoming packet
// ==================================================
void processPaket(const AktuatorPaket& pkt) {
  for (uint8_t i = 0; i < antalMotorer; i++) {
    malPosisjonerMm[i] = constrain((int)pkt.positionerMm[i], 0, 100);

    // Negative velocities are allowed
    malHastigheterMmS[i] = pkt.hastigheterMmS[i];
  }

  senastPaketMs = millis();
}

// ==================================================
// Receive packet stream
// ==================================================
void mottagaPaket() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

    if (rxIndex == 0) {
      if (b == KalibreringKom) {
        Serial.println("CALIBRATION COMMAND RECEIVED");
        Serial.flush();

        kalibrering();

        Serial.println("CALIBRATION COMMAND DONE");
        Serial.flush();
        continue;
      }

      if (b == sync1) {
        rxBuffer[rxIndex++] = b;
      }

      continue;
    }

    if (rxIndex == 1) {
      if (b == sync2) {
        rxBuffer[rxIndex++] = b;
      } else {
        rxIndex = 0;
      }

      continue;
    }

    rxBuffer[rxIndex++] = b;

    if (rxIndex >= paketStorlek) {
      AktuatorPaket pkt;

      if (tolkaPaket(rxBuffer, pkt)) {
        mottagenPaketAntal++;
        senastMottagnSekvens = pkt.sekvens;
        processPaket(pkt);
      } else {
        daligPaketAntal++;
        Serial.println("Bad packet");
      }

      rxIndex = 0;
    }
  }
}

// ==================================================
// Optional pin check
// ==================================================
void checkInterruptPin(int pin) {
  int checkPin = digitalPinToInterrupt(pin);

  if (checkPin == -1) {
    Serial.println("Not a valid interrupt pin!");
    Serial.println(pin);
  } else {
    Serial.println("Valid interrupt pin.");
    Serial.println(pin);
  }
}

// ==================================================
// Setup
// ==================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("SETUP STARTED");
  Serial.flush();

  analogWriteResolution(8);

  // Motor A
  A.pwm = 6;
  A.dir = 32;
  A.chn1 = 29; //31
  A.chn2 = 52;

  // Motor B
  B.pwm = 7;
  B.dir = 33;
  B.chn1 = 24;
  B.chn2 = 22;

  // Motor C
  C.pwm = 13;
  C.dir = 40;
  C.chn1 = 18;
  C.chn2 = 19;

  // Motor D
  D.pwm = 12;
  D.dir = 41;
  D.chn1 = 49;
  D.chn2 = 17;

  // Motor E
  E.pwm = 8;
  E.dir = 44;
  E.chn1 = 51;
  E.chn2 = 10;

  // Motor F
  F.pwm = 9;
  F.dir = 45;
  F.chn1 = 60;
  F.chn2 = 53;

  // Initialize pins
  for (uint8_t i = 0; i < antalMotorer; i++) {
    motor* m = allaMotorer[i];

    pinMode(m->pwm, OUTPUT);
    pinMode(m->dir, OUTPUT);
    pinMode(m->chn1, INPUT_PULLUP);
    pinMode(m->chn2, INPUT_PULLUP);

    analogWrite(m->pwm, 0);

    m->prevState = (digitalRead(m->chn1) << 1) | digitalRead(m->chn2);
  }

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(A.chn1), irqA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A.chn2), irqA2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(B.chn1), irqB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(B.chn2), irqB2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(C.chn1), irqC1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C.chn2), irqC2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(D.chn1), irqD1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(D.chn2), irqD2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(E.chn1), irqE1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E.chn2), irqE2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(F.chn1), irqF1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(F.chn2), irqF2, CHANGE);

  Serial.println("DEBUG: All interrupts attached, about to call kalibrering...");
  Serial.flush();

  delay(500);

  kalibrering();

  Serial.println("DEBUG: Kalibrering returned successfully");
  Serial.flush();

  Serial.print("Arduino ready for Stewart packets (");
  Serial.print(antalMotorer);
  Serial.print(" actuators, packet size=");
  Serial.print(paketStorlek);
  Serial.println(" bytes)");
  Serial.flush();
}

// ==================================================
// Main loop
// ==================================================
void loop() {
  updateraSkyddadRaknare();

  if (!kalibreringKlar) {
    return;
  }

  mottagaPaket();

  static unsigned long senastUs = micros();
  unsigned long nuUs = micros();

  if (nuUs - senastUs < kontrollperiodUs) {
    return;
  }

  float dt = (nuUs - senastUs) / 1000000.0f;
  senastUs = nuUs;

  bool packetTimeout = false;

  if (millis() - senastPaketMs > packetTimeoutMs) {
    packetTimeout = true;
  }

  for (uint8_t i = 0; i < antalMotorer; i++) {
    updateraMotorHastighet(*allaMotorer[i], dt);

    if (move_utanreglerig) {
      move(*allaMotorer[i], malPosisjonerMm[i]);
    } else {
      if (packetTimeout) {
        // Hold current position if communication stops
        float currentPosMm = 0.0f;

        if (allaMotorer[i]->pulserPerMm > 0.0f) {
          currentPosMm =
              allaMotorer[i]->skyddadRaknare / allaMotorer[i]->pulserPerMm;
        }

        moveCascade(*allaMotorer[i], currentPosMm, 0.0f, dt);
      } else {
        moveCascade(
            *allaMotorer[i],
            (float)malPosisjonerMm[i],
            (float)malHastigheterMmS[i],
            dt
        );
      }
    }
  }

  // Debug once per 10 ms
  if (nuUs - senasteDebugPrintUs >= 100000UL) {
    Serial.print("Current/target pos:");

    for (uint8_t i = 0; i < antalMotorer; i++) {
      float posMm = 0.0f;

      if (allaMotorer[i]->pulserPerMm > 0.0f) {
        posMm = allaMotorer[i]->skyddadRaknare / allaMotorer[i]->pulserPerMm;
      }

      Serial.print((char)('A' + i));
      Serial.print(": ");
      Serial.print(posMm, 2);
      Serial.print(" -> ");
      Serial.print((float)malPosisjonerMm[i], 2);
    }

    Serial.print(" packets=");
    Serial.print(mottagenPaketAntal);
    Serial.print(" bad=");
    Serial.print(daligPaketAntal);

    if (packetTimeout) {
      Serial.print(" TIMEOUT");
    }

    Serial.println();

    senasteDebugPrintUs = nuUs;
  }
}