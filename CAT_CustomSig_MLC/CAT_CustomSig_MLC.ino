// Mega 2560 dual-channel PWM generator @500Hz from external analog 0-5V control inputs
// CH1: VIN_CTRL A0 -> PWM_OUT D11 (OC1A)
// CH2: VIN_CTRL A1 -> PWM_OUT D12 (OC1B)
// Control input: neutral 2.5V, range 0.5V..4.5V => maps to 0..100% duty
// Power: feed regulated 5V into 5V pin + GND (NOT VIN)

#include <Arduino.h>
#include <math.h>

// ---------------- CONFIG ----------------
#define DEBUG_ENABLE 1   // turns on/off serial output. set to 0 when scoping for cleanest edges

constexpr uint8_t VIN1     = A0;  // 0-5V input 1 from MLC
constexpr uint8_t VIN2     = A1;  // 0-5V input 2 from MLC 

constexpr uint8_t PWM_OUT1 = 11; // output 1 PWM signal
constexpr uint8_t PWM_OUT2 = 12; // output 2 PWM signal

// 500 Hz => 2000 us period. Timer1 prescaler 8 => 0.5 us tick => TOP = 2000/0.5 - 1 = 3999
constexpr uint16_t TIMER1_TOP = 3999;

// External control voltage spec (using AVcc as ADC reference)
constexpr float VIN_MIN_V = 0.5f;
constexpr float VIN_MAX_V = 4.5f;
constexpr float VIN_DEADBAND_FRAC = 0.01f; // deadband as fraction of full span (1% of 0..1 duty)

// Input cleanup
constexpr float VIN_ALPHA   = 0.05f; // IIR smoothing
constexpr uint8_t VIN_AVG_N = 4;     // simple averaging

// OCR update deadband (timer counts). 1 count = 0.5us / 2000us = 0.025% duty.
constexpr uint16_t OCR_DEADBAND_COUNTS = 3;

// Debug
constexpr uint32_t BAUD            = 115200;
constexpr uint32_t DEBUG_PERIOD_MS = 100;

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline uint16_t dutyToOcr(float duty) {
  duty = clampf(duty, 0.0f, 1.0f);
  uint16_t ocr = (uint16_t)lroundf(duty * (float)TIMER1_TOP);
  if (ocr > TIMER1_TOP) ocr = TIMER1_TOP;
  return ocr;
}

static inline void writeOcrWithDeadband(volatile uint16_t &ocrReg, uint16_t newVal, uint16_t &lastVal) {
  uint16_t diff = (newVal > lastVal) ? (newVal - lastVal) : (lastVal - newVal);
  if (diff >= OCR_DEADBAND_COUNTS) {
    ocrReg = newVal;
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

// setup timer to create the PWM signal
void setupTimer1_500Hz_OC1A_OC1B() {
  pinMode(PWM_OUT1, OUTPUT);
  pinMode(PWM_OUT2, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Fast PWM, TOP=ICR1 (mode 14), non-inverting OC1A/OC1B, prescaler=8
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS11);

  ICR1  = TIMER1_TOP;
  OCR1A = 0;
  OCR1B = 0;
}

void setup() {
#if DEBUG_ENABLE
  Serial.begin(BAUD);
  Serial.println(F("DBG: vOk vccmV vin1_raw vin2_raw"));
#endif
  setupTimer1_500Hz_OC1A_OC1B();
}

void loop() {
  static uint32_t lastDbgMs = 0;

  // IIR filter states in ADC counts
  static float vinFilt1 = 512.0f;
  static float vinFilt2 = 512.0f;

  // Last OCR values for deadbanded updates
  static uint16_t lastOcrA = 0;
  static uint16_t lastOcrB = 0;

  // Read + average + filter analog control inputs (ADC counts 0..1023)
  const int vinRaw1 = analogReadAvg(VIN1, VIN_AVG_N);
  const int vinRaw2 = analogReadAvg(VIN2, VIN_AVG_N);

  vinFilt1 += VIN_ALPHA * ((float)vinRaw1 - vinFilt1);
  vinFilt2 += VIN_ALPHA * ((float)vinRaw2 - vinFilt2);

  // Convert ADC counts -> volts using measured Vcc (keeps mapping stable if 5V rail isn't exactly 5.000V)
  const float vcc = (float)readVcc_mV() / 1000.0f;
  const float vin1 = (vinFilt1 * vcc) / 1023.0f;
  const float vin2 = (vinFilt2 * vcc) / 1023.0f;

  // Map 0.5..4.5V to 0..1 duty
  float duty1 = (vin1 - VIN_MIN_V) / (VIN_MAX_V - VIN_MIN_V);
  float duty2 = (vin2 - VIN_MIN_V) / (VIN_MAX_V - VIN_MIN_V);

  duty1 = clampf(duty1, 0.0f, 1.0f);
  duty2 = clampf(duty2, 0.0f, 1.0f);

  // Small deadband around mid-scale (reduces tiny output twitch near 2.5V)
  const float db = VIN_DEADBAND_FRAC; // in duty units
  if (fabsf(duty1 - 0.5f) < db) duty1 = 0.5f;
  if (fabsf(duty2 - 0.5f) < db) duty2 = 0.5f;

  const uint16_t ocrA = dutyToOcr(duty1);
  const uint16_t ocrB = dutyToOcr(duty2);

  noInterrupts();
  writeOcrWithDeadband(OCR1A, ocrA, lastOcrA);
  writeOcrWithDeadband(OCR1B, ocrB, lastOcrB);
  interrupts();

// debug lines to output to serial monitor 
#if DEBUG_ENABLE
  const uint32_t nowMs = millis();
  if (nowMs - lastDbgMs >= DEBUG_PERIOD_MS) {
    lastDbgMs = nowMs;

    const uint16_t vccmV = readVcc_mV();
    const bool vOk = (vccmV >= 4700) && (vccmV <= 5300);

    Serial.print(vOk ? 1 : 0); Serial.print(' ');
    Serial.print(vccmV);       Serial.print(' ');
    Serial.print(vinRaw1);     Serial.print(' ');
    Serial.println(vinRaw2);
  }
#endif
}
