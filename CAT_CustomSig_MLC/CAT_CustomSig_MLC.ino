// Mega 2560 7-channel PWM generator @500Hz from external analog 0-5V control inputs
// Inputs:  LHJSX A0, LHJSY A1, RHJSX A2, RHJSY A3, TRAVLH A4, TRAVRH A5, BLDY A6
// Outputs: LHJSX D11(OC1A), LHJSY D12(OC1B), RHJSX D5(OC3A), RHJSY D2(OC3B),
//          TRAVLH D3(OC3C), TRAVRH D6(OC4A), BLDY D7(OC4B)
// Control input: neutral 2.5V, range 0.5V..4.5V => maps to 0..100% duty
// Power: feed regulated 5V into 5V pin + GND (NOT VIN)

#include <Arduino.h>
#include <math.h>

// ---------------- CONFIG ----------------
#define DEBUG_ENABLE 1   // set to 0 when scoping for cleanest edges

// Analog inputs
constexpr uint8_t VIN_LHJSX  = A0;
constexpr uint8_t VIN_LHJSY  = A1;
constexpr uint8_t VIN_RHJSX  = A2;
constexpr uint8_t VIN_RHJSY  = A3;
constexpr uint8_t VIN_TRAVLH = A4;
constexpr uint8_t VIN_TRAVRH = A5;
constexpr uint8_t VIN_BLDY   = A6;

// PWM outputs (hardware timer pins on Mega2560)
constexpr uint8_t PWM_LHJSX  = 11; // OC1A
constexpr uint8_t PWM_LHJSY  = 12; // OC1B
constexpr uint8_t PWM_RHJSX  = 5;  // OC3A
constexpr uint8_t PWM_RHJSY  = 2;  // OC3B
constexpr uint8_t PWM_TRAVLH = 3;  // OC3C
constexpr uint8_t PWM_TRAVRH = 6;  // OC4A
constexpr uint8_t PWM_BLDY   = 7;  // OC4B

// 500 Hz => 2000 us period. Timer prescaler 8 => 0.5 us tick => TOP = 2000/0.5 - 1 = 3999
constexpr uint16_t TIMER_TOP = 3999;

// External control voltage spec (using AVcc as ADC reference)
constexpr float VIN_MIN_V = 0.5f;
constexpr float VIN_MAX_V = 4.5f;
constexpr float VIN_DEADBAND_FRAC = 0.01f; // deadband as fraction of full span (1% of 0..1 duty)

// Input cleanup
constexpr float   VIN_ALPHA = 0.05f; // IIR smoothing
constexpr uint8_t VIN_AVG_N = 4;     // simple averaging

// OCR update deadband (timer counts). 1 count = 0.5us / 2000us = 0.025% duty.
constexpr uint16_t OCR_DEADBAND_COUNTS = 1;

// Debug
constexpr uint32_t BAUD            = 115200;
constexpr uint32_t DEBUG_PERIOD_MS = 100;

// ---------------- HELPERS ----------------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline uint16_t dutyToOcr(float duty) {
  duty = clampf(duty, 0.0f, 1.0f);
  uint16_t ocr = (uint16_t)lroundf(duty * (float)TIMER_TOP);
  if (ocr > TIMER_TOP) ocr = TIMER_TOP;
  return ocr;
}

static inline void writeOcrWithDeadband(volatile uint16_t *ocrReg, uint16_t newVal, uint16_t &lastVal) {
  uint16_t diff = (newVal > lastVal) ? (newVal - lastVal) : (lastVal - newVal);
  if (diff >= OCR_DEADBAND_COUNTS) {
    *ocrReg = newVal;
    lastVal = newVal;
  }
}

// Simple averaging to reduce ADC noise
static int analogReadAvg(uint8_t pin, uint8_t n) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < n; i++) sum += (uint16_t)analogRead(pin);
  return (int)(sum / n);
}

// Vcc sense via 1.1V bandgap vs AVCC (approx; bandgap tolerance applies)
static uint16_t readVcc_mV() {
  const uint8_t oldADMUX  = ADMUX;
  const uint8_t oldADCSRB = ADCSRB;
  const uint8_t oldADCSRA = ADCSRA;

  ADCSRA |= _BV(ADEN);
  ADCSRA = (ADCSRA & ~(_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0))) | (_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));

  ADMUX = _BV(REFS0) | 0x1E;     // AVCC ref, MUX=1.1V bandgap
  ADCSRB &= ~_BV(MUX5);

  delayMicroseconds(200);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)) {}
  const uint16_t adc = ADC;

  ADMUX  = oldADMUX;
  ADCSRB = oldADCSRB;
  ADCSRA = oldADCSRA;

  if (adc == 0) return 0;
  return (uint16_t)((uint32_t)1100UL * 1024UL / (uint32_t)adc);
}

// ---------------- TIMER SETUP (500 Hz) ----------------
// Fast PWM, TOP=ICRn (mode 14), non-inverting outputs, prescaler=8

static void setupTimer1_500Hz() {
  pinMode(PWM_LHJSX, OUTPUT);
  pinMode(PWM_LHJSY, OUTPUT);

  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS11);
  ICR1   = TIMER_TOP;
  OCR1A  = 0;
  OCR1B  = 0;
}

static void setupTimer3_500Hz() {
  pinMode(PWM_RHJSX, OUTPUT);
  pinMode(PWM_RHJSY, OUTPUT);
  pinMode(PWM_TRAVLH, OUTPUT);

  TCCR3A = 0; TCCR3B = 0; TCNT3 = 0;
  TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM31);
  TCCR3B = (1 << WGM33)  | (1 << WGM32)  | (1 << CS31);
  ICR3   = TIMER_TOP;
  OCR3A  = 0;
  OCR3B  = 0;
  OCR3C  = 0;
}

static void setupTimer4_500Hz() {
  pinMode(PWM_TRAVRH, OUTPUT);
  pinMode(PWM_BLDY, OUTPUT);

  TCCR4A = 0; TCCR4B = 0; TCNT4 = 0;
  TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << WGM41);
  TCCR4B = (1 << WGM43)  | (1 << WGM42)  | (1 << CS41);
  ICR4   = TIMER_TOP;
  OCR4A  = 0;
  OCR4B  = 0;
}

// ---------------- CHANNEL TABLE ----------------
struct Chan {
  uint8_t  ain;
  volatile uint16_t *ocr;
  float    filt;     // IIR state (ADC counts)
  uint16_t lastOcr;  // last written OCR for deadband
};

static Chan ch[7] = {
  { VIN_LHJSX,  &OCR1A, 512.0f, 0 }, // LHJSX
  { VIN_LHJSY,  &OCR1B, 512.0f, 0 }, // LHJSY
  { VIN_RHJSX,  &OCR3A, 512.0f, 0 }, // RHJSX
  { VIN_RHJSY,  &OCR3B, 512.0f, 0 }, // RHJSY
  { VIN_TRAVLH, &OCR3C, 512.0f, 0 }, // TRAVLH
  { VIN_TRAVRH, &OCR4A, 512.0f, 0 }, // TRAVRH
  { VIN_BLDY,   &OCR4B, 512.0f, 0 }, // BLDY
};

void setup() {
#if DEBUG_ENABLE
  Serial.begin(BAUD);
  Serial.println(F("DBG: vOk vccmV "
                   "LHJSX_raw LHJSY_raw RHJSX_raw RHJSY_raw TRAVLH_raw TRAVRH_raw BLDY_raw "
                   "LHJSX_duty LHJSY_duty RHJSX_duty RHJSY_duty TRAVLH_duty TRAVRH_duty BLDY_duty"));
#endif
  setupTimer1_500Hz();
  setupTimer3_500Hz();
  setupTimer4_500Hz();
}

void loop() {
  static uint32_t lastDbgMs = 0;

  // Read Vcc once per loop (used for all channel voltage conversions)
  const float vcc = (float)readVcc_mV() / 1000.0f;

  int   raw[7];
  float vinV[7];     // channel voltage used for mapping (V)
  float dutyPct[7];  // channel duty (0..100%)
  // Read + average + filter each analog input
  for (uint8_t i = 0; i < 7; i++) {
    raw[i] = analogReadAvg(ch[i].ain, VIN_AVG_N);
    ch[i].filt += VIN_ALPHA * ((float)raw[i] - ch[i].filt);
  }

  noInterrupts();
  for (uint8_t i = 0; i < 7; i++) {
    const float vin = (ch[i].filt * vcc) / 1023.0f;  // volts at ADC pin (using measured Vcc)
    vinV[i] = vin;

    float duty = (vin - VIN_MIN_V) / (VIN_MAX_V - VIN_MIN_V);
    duty = clampf(duty, 0.0f, 1.0f);

    // Small deadband around mid-scale (reduces tiny output twitch near 2.5V)
    const float db = VIN_DEADBAND_FRAC;
    if (fabsf(duty - 0.5f) < db) duty = 0.5f;

    dutyPct[i] = duty * 100.0f;

    const uint16_t ocr = dutyToOcr(duty);
    writeOcrWithDeadband(ch[i].ocr, ocr, ch[i].lastOcr);
  }
  interrupts();

#if DEBUG_ENABLE
const uint32_t nowMs = millis();
if (nowMs - lastDbgMs >= DEBUG_PERIOD_MS) {
  lastDbgMs = nowMs;

  Serial.print(F("LHJSX_Vin="));  Serial.print(vinV[0], 2);  Serial.print(F("V "));
  Serial.print(F("LHJSY_Vin="));  Serial.print(vinV[1], 2);  Serial.print(F("V "));
  Serial.print(F("RHJSX_Vin="));  Serial.print(vinV[2], 2);  Serial.print(F("V "));
  Serial.print(F("RHJSY_Vin="));  Serial.print(vinV[3], 2);  Serial.print(F("V "));
  Serial.print(F("TRAVLH_Vin=")); Serial.print(vinV[4], 2);  Serial.print(F("V "));
  Serial.print(F("TRAVRH_Vin=")); Serial.print(vinV[5], 2);  Serial.print(F("V "));
  Serial.print(F("BLDY_Vin="));   Serial.print(vinV[6], 2);  Serial.print(F("V "));

  Serial.print(F("LHJSX_Duty="));  Serial.print(dutyPct[0], 1);  Serial.print(F("% "));
  Serial.print(F("LHJSY_Duty="));  Serial.print(dutyPct[1], 1);  Serial.print(F("% "));
  Serial.print(F("RHJSX_Duty="));  Serial.print(dutyPct[2], 1);  Serial.print(F("% "));
  Serial.print(F("RHJSY_Duty="));  Serial.print(dutyPct[3], 1);  Serial.print(F("% "));
  Serial.print(F("TRAVLH_Duty=")); Serial.print(dutyPct[4], 1);  Serial.print(F("% "));
  Serial.print(F("TRAVRH_Duty=")); Serial.print(dutyPct[5], 1);  Serial.print(F("% "));
  Serial.print(F("BLDY_Duty="));   Serial.print(dutyPct[6], 1);  Serial.println(F("%"));
}
#endif
}