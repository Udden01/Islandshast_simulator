#include <Arduino.h>

//motor  klass
class motor {
public:
  // Pinnar
  int pwm;
  int dir;
  int chn1;
  int chn2;

  // pulser per mm
  float pulserPerMm = 0.0f;

  //puls räknare och tidigare tillstånd för encoder
  volatile long pulseRaknare = 0;
  volatile uint8_t prevState = 0;

  //skyddad kopia av räknare för att läsa i kontrollloop utan att oroa sig för avbrott
  long skyddadRaknare = 0;
  long tidigareKontrollRaknare = 0;

  // Hastighet
  float hastighetPulserPerS = 0.0f;
  float filtreradHastighetMmS = 0.0f;

  // mål position i pulser
  long malPos = 0;

  // motor i rörelse flagga och aktuell PWM för rampning
  bool iRorelse = false;
  int aktuellPWM = 0;

  //kontroller integraler
  float integralFelMm = 0.0f;
  float integralHastighetsFel = 0.0f;

  long tidigareMalPos = -1;
};

//motor objekten
motor A;
motor B;
motor C;
motor D;
motor E;
motor F;

static const uint8_t antalMotorer = 6;
//array med pekare till alla motorer för enklare loopar
motor* allaMotorer[antalMotorer] = {&A, &B, &C, &D, &E, &F};


bool move_utanreglerig = false;

// motor trim i mm för att justera mekaniska skillnader mellan aktuatorerna, kan vara positiv eller negativ
static const int motorTrimMm[antalMotorer] = {0, 0, 0, 0, 0, 0};

// Kalibrering resultat: max och min pulser per motor
long maxPulserPerMotor[antalMotorer] = {0};
long minPulserPerMotor[antalMotorer] = {0};


constexpr unsigned long kontrollperiodUs = 10000; // 10 ms = 100 Hz

//yttre position-till-hastighet regulator: position error [mm] -> velocity correction [mm/s]
constexpr float kpPositionTillHastighet = 5.0f;

//Regulator för hastighet: velocity error [mm/s] -> PWM
constexpr float kpHastighet = 4.0f;
constexpr float kiHastighet = 20.0f;

// Hastighet filter
constexpr float hastighetsFilterAlpha = 0.25f;

// Hastighetesgränser
constexpr float maxHastighetMmS = 80.0f;
constexpr float maxIntegralHastighet = 30.0f;

//PWM gränser
constexpr int maxPWM = 240;
constexpr int minPWM = 12;

// statisk friktionskompensation i PWM steg, justera efter behov för att övervinna startfriktion
constexpr int friktionsPWM = 18;

// Toleranser för att avgöra när vi kan stoppa motorn, både i position och hastighet
constexpr float positionToleransMm = 0.5f;
constexpr float hastighetToleransMmS = 2.0f;

// Mekaniska gränser i mm, används för säkerhetsstopp och för att begränsa målpositioner. Justera efter din aktuators faktiska rörelseomfång.
constexpr float maxPositionMm = 100.0f;
constexpr float minPositionMm = 0.0f;

// PWM ramp steg per kontrollcykel
constexpr int pwmStegPerCykel = 8;

// Timeout om inga nya paket kommer in
constexpr unsigned long packetTimeoutMs = 500;

//paket information
static const uint8_t antalPaketAktuatorer = 6;
static const uint8_t sync1 = 0xAA;
static const uint8_t sync2 = 0x55;
static const uint8_t KalibreringKom = 'C';

// 2 sync + 2 seq + 2 dt + 12 pos + 12 vel + 1 checksum = 31 bytes
static const uint8_t paketStorlek = 7 + (4 * antalPaketAktuatorer);

static bool kalibreringKlar = false;

// Struktur för att hålla data från mottagna paket
struct AktuatorPaket {
  uint16_t sekvens;
  uint16_t dt_ms;
  int16_t positionerMm[antalPaketAktuatorer];
  int16_t hastigheterMmS[antalPaketAktuatorer];
};

// Buffer för att ta emot paket och index för att spåra mottagning
uint8_t rxBuffer[paketStorlek];
uint8_t rxIndex = 0;

//mottagna paket
uint32_t mottagenPaketAntal = 0;

//dåliga paket
uint32_t daligPaketAntal = 0;
//sekvensnummer för senaste mottagna paket
uint16_t senastMottagnSekvens = 0;

// Tid för senaste debugutskrift och senaste mottagna paket
unsigned long senasteDebugPrintUs = 0;
unsigned long senastPaketMs = 0;

//mål positioner och hastigheter i mm och mm/s, uppdateras av mottagna paket och används i kontrollloop
int malPosisjonerMm[antalMotorer] = {100, 100, 100, 100, 100, 100};
int malHastigheterMmS[antalMotorer] = {0, 0, 0, 0, 0, 0};

//uppdaterar räknare baserat på encoderövergångar, kallas från alla encoder interrupt handlers
void uppdateraEncoderRaknare(motor &Motor) {
  uint8_t state = (digitalRead(Motor.chn1) << 1) | digitalRead(Motor.chn2);
  uint8_t combined = (Motor.prevState << 2) | state;

  switch (combined) {
    // Frammåt
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      Motor.pulseRaknare--;
      break;

    // Bakåt
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      Motor.pulseRaknare++;
      break;

    default:
      // ogiltig övergång, kan bero på brus eller snabb rörelse, ignorera
      break;
  }

  Motor.prevState = state;
}

// Avbrottshanterare för encoder, två per motor för att fånga alla övergångar
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

//updaterar skyddad räknare
void updateraSkyddadRaknare() {
  noInterrupts();
  for (uint8_t i = 0; i < antalMotorer; i++) {
    allaMotorer[i]->skyddadRaknare = allaMotorer[i]->pulseRaknare;
  }
  interrupts();
}

//uppdaterar motorhastighet 
void updateraMotorHastighet(motor &Motor, float dt) {
  if (dt <= 0.000001f) {
    return;
  }
  //tar fram nuvarande skyddad räknare
  long nuvarande = Motor.skyddadRaknare;
  //beräknar skillnaden i pulser sedan senaste kontrollräknaren
  long deltaPulser = nuvarande - Motor.tidigareKontrollRaknare;

  Motor.hastighetPulserPerS = deltaPulser / dt;
  Motor.tidigareKontrollRaknare = nuvarande;
  //beräknar hastighet i mm/s och filtrerar den för att få en stabilare hastighetsmätning
  if (Motor.pulserPerMm > 0.0f) {
    float rawHastighetMmS = Motor.hastighetPulserPerS / Motor.pulserPerMm;

    Motor.filtreradHastighetMmS =
        hastighetsFilterAlpha * rawHastighetMmS +
        (1.0f - hastighetsFilterAlpha) * Motor.filtreradHastighetMmS;
  } else {
    Motor.filtreradHastighetMmS = 0.0f;
  }
}

// enkel rörelsefunktion utan reglering, används för att snabbt testa motorer och mekanik
void move(motor &Motor, int malMm) {
  if (Motor.pulserPerMm <= 0.0f) {
    analogWrite(Motor.pwm, 0);
    return;
  }
  //beräknar mål i pulser baserat på mål i mm och pulser per mm
  int begransadMalMm = constrain(malMm, 0, 100);
  long malPulser = (long)(begransadMalMm * Motor.pulserPerMm);

  long nuvarandePulser = Motor.skyddadRaknare;

  // Hitta motorindex för att kolla max pulser från kalibrering
  long motorIndex = -1;
  for (uint8_t i = 0; i < antalMotorer; i++) {
    if (allaMotorer[i] == &Motor) {
      motorIndex = i;
      break;
    }
  }

    // Om max pulser per motor är definierat från kalibrering, använd det som gräns, annars använd en generell gräns baserat på mekanisk maxposition
  long maxPulserLimit =
      (motorIndex >= 0 && maxPulserPerMotor[motorIndex] > 0)
      ? maxPulserPerMotor[motorIndex]
      : (long)(100.0f * Motor.pulserPerMm);

  // Säkerhetsstopp: om nuvarande pulser är utanför det mekaniska området, stoppa motorn och sätt iRorelse till false
  if (nuvarandePulser > maxPulserLimit || nuvarandePulser < 0) {
    Motor.iRorelse = false;
    Motor.aktuellPWM = 0;
    analogWrite(Motor.pwm, 0);
    return;
  }

  // Begränsa mål i pulser baserat på kalibrering eller mekaniska gränser
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


// Denna funktion implementerar en kaskadregulator där den yttre loopen korrigerar positionen 
//och den inre loopen korrigerar hastigheten. Den tar hänsyn till mekaniska gränser, 
//statisk friktion och har en rampning av PWM för att få en mjukare rörelse.
void moveCascade(motor &Motor, float refPosMm, float refVelMmS, float dt) {
  if (Motor.pulserPerMm <= 0.0f) {
    analogWrite(Motor.pwm, 0);
    Motor.iRorelse = false;
    return;
  }

  // Hitta motorindex för att kolla max pulser från kalibrering
  long motorIndex = -1;
  for (uint8_t i = 0; i < antalMotorer; i++) {
    if (allaMotorer[i] == &Motor) {
      motorIndex = i;
      break;
    }
  }

  long nuvarandePulser = Motor.skyddadRaknare;
  float posMm = nuvarandePulser / Motor.pulserPerMm;

  //max pulser gräns baserat på kalibrering eller mekaniska gränser
  long maxPulserLimit =
      (motorIndex >= 0 && maxPulserPerMotor[motorIndex] > 0)
      ? maxPulserPerMotor[motorIndex]
      : (long)(maxPositionMm * Motor.pulserPerMm);

  //säkerhetsstopp: om nuvarande pulser är utanför det mekaniska området, stoppa motorn och sätt iRorelse till false
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

  // Yttre position-till-hastighet regulator
  float velRefMmS = refVelMmS + kpPositionTillHastighet * posFelMm;
  velRefMmS = constrain(velRefMmS, -maxHastighetMmS, maxHastighetMmS);

  //säkerhetsstopp för att inte köra utanför mekaniska gränser, tillåter rörelse tillbaka in i området
  if (posMm <= minPositionMm + 1.0f && velRefMmS < 0.0f) {
    velRefMmS = 0.0f;
  }

  if (posMm >= maxPositionMm - 1.0f && velRefMmS > 0.0f) {
    velRefMmS = 0.0f;
  }

  // Uppdatera motorhastighet baserat på senaste räknare
  float velMmS = Motor.filtreradHastighetMmS;
  float velFelMmS = velRefMmS - velMmS;

  // Om både position och hastighet är inom toleranser, stoppa motorn och nollställ integralen
  if (fabs(posFelMm) < positionToleransMm &&
      fabs(refVelMmS) < hastighetToleransMmS &&
      fabs(velMmS) < hastighetToleransMmS) {
    analogWrite(Motor.pwm, 0);
    Motor.integralHastighetsFel = 0.0f;
    Motor.aktuellPWM = 0;
    Motor.iRorelse = false;
    return;
  }

  // hastighetsregulator med integralkomponent, begränsad för att undvika vindup
  Motor.integralHastighetsFel += velFelMmS * dt;
  Motor.integralHastighetsFel =
      constrain(Motor.integralHastighetsFel,
                -maxIntegralHastighet,
                maxIntegralHastighet);

  //styrsignalen
  float styrsignal =
      kpHastighet * velFelMmS +
      kiHastighet * Motor.integralHastighetsFel;

  
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

  // PWM ramp för smidigare motorsteg
  if (pwm > Motor.aktuellPWM) {
    Motor.aktuellPWM = min(Motor.aktuellPWM + pwmStegPerCykel, pwm);
  } else {
    Motor.aktuellPWM = max(Motor.aktuellPWM - pwmStegPerCykel, pwm);
  }

  digitalWrite(Motor.dir, styrsignal >= 0.0f ? HIGH : LOW);
  analogWrite(Motor.pwm, Motor.aktuellPWM);

  Motor.iRorelse = true;
}

//kalibrering av alla motorer
void kalibrering() {
  delay(200);

  Serial.println("=== CALIBRATION START ===");
  Serial.flush();

  // Alla motorer ner
  for (uint8_t i = 0; i < antalMotorer; i++) {
    digitalWrite(allaMotorer[i]->dir, LOW);
    analogWrite(allaMotorer[i]->pwm, 255);
  }

  delay(4000);

  // Stanna alla motorer
  for (uint8_t i = 0; i < antalMotorer; i++) {
    analogWrite(allaMotorer[i]->pwm, 0);
  }

  delay(1000);

  //sätt nya värden på alla motorer
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

  // Kör alla motorer uppåt för att hitta 100 mm-området
  for (uint8_t i = 0; i < antalMotorer; i++) {
    digitalWrite(allaMotorer[i]->dir, HIGH);
    analogWrite(allaMotorer[i]->pwm, 255);
  }

  delay(10000);

  // Stoppa alla motorer
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

    // Efter kalibrering är motorerna i toppen, ungefär 100 mm
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

  // Töm seriell buffert mottagen under kalibrering
  while (Serial.available() > 0) {
    Serial.read();
  }

  rxIndex = 0;
}


// Pakethjälpfunktioner
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

//verifiera paket
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


// Bearbeta mottaget paket
void processPaket(const AktuatorPaket& pkt) {
  for (uint8_t i = 0; i < antalMotorer; i++) {
    malPosisjonerMm[i] = constrain((int)pkt.positionerMm[i], 0, 100);

    // Negativa hastigheter är tillåtna
    malHastigheterMmS[i] = pkt.hastigheterMmS[i];
  }

  senastPaketMs = millis();
}

// Ta emot paket
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


// Valfri pin-kontroll
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


// Initiering
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("SETUP STARTED");
  Serial.flush();

  analogWriteResolution(8);

  // Motor A
  A.pwm = 6;
  A.dir = 32;
  A.chn1 = 29; 
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

  // Initiera pinnar
  for (uint8_t i = 0; i < antalMotorer; i++) {
    motor* m = allaMotorer[i];

    pinMode(m->pwm, OUTPUT);
    pinMode(m->dir, OUTPUT);
    pinMode(m->chn1, INPUT_PULLUP);
    pinMode(m->chn2, INPUT_PULLUP);

    analogWrite(m->pwm, 0);

    m->prevState = (digitalRead(m->chn1) << 1) | digitalRead(m->chn2);
  }

  // Koppla avbrott
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

// Huvudloop
void loop() {
  updateraSkyddadRaknare();

  //kollar om kalibreringen klar
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
        // Behåll aktuell position om kommunikationen avbryts
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

  // Debugga varje 10 ms
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