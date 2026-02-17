// Portenta H7 dual-channel PWM @500Hz from external analog inputs
// CH1: A0 -> PWM2 (label on breakout; maps to D4 on Portenta H7 pinout)
// CH2: A1 -> PWM3 (label on breakout; maps to D3 on Portenta H7 pinout)
//
// Expanded: 7-channel version.
//
// CURRENT HARDWARE ASSUMPTIONS (THIS MATTERS):
// - Portenta H7 is running the Arduino Mbed OS core (mbed::PwmOut is used for PWM generation).
// - REFP/AREF is externally driven with a stable 3.0V reference (must be <= 3.1V on Portenta H7).
// - REFN is tied to GND (same ground reference as the external analog sources).
// - All VIN_* signals are constrained to 0.0V .. 3.0V (relative to REFN/GND).
//   Example: joystick powered from 3.0V so neutral is ~1.5V.
//
// WHAT THE CODE DOES:
// - Reads 7 analog channels using 12-bit ADC (0..4095 counts).
// - Converts each ADC reading linearly to PWM duty cycle:
//     0 counts   -> 0% duty
//     ~2048      -> ~50% duty (for ~1.5V input with 3.0V reference)
//     4095 counts-> 100% duty
// - Outputs 7 PWM channels at 500 Hz on PWM2..PWM8 (Portenta Breakout labels).
// - Optional serial debug prints raw ADC counts and computed duty cycle.

#include <Arduino.h>
#include <math.h>
#include "mbed.h"
#include <Arduino_PortentaBreakout.h>

// ---------------- CONFIG ----------------
#define DEBUG_ENABLE 0
constexpr uint32_t BAUD = 115200;
constexpr uint32_t DEBUG_PERIOD_MS = 100;

// Inputs
constexpr uint8_t VIN_LHJSX = A0;   // LH JS X
constexpr uint8_t VIN_LHJSY = A1;   // LH JS y
constexpr uint8_t VIN_RHJSX = A2;   // RH JS X
constexpr uint8_t VIN_RHJSY = A3;   // RH JS y
constexpr uint8_t VIN_LHTRAV = A4;  // LH Travel
constexpr uint8_t VIN_RHTRAV = A5;  // RH Travel
constexpr uint8_t VIN_BLADEY = A6;  // Blade y

constexpr uint8_t LHJSX_PWM = PWM2;   // PWM2 (breakout label)  -> CH1 output PWM signal
constexpr uint8_t LHJSY_PWM = PWM3;   // PWM3 (breakout label)  -> CH2 output PWM signal
constexpr uint8_t RHJSX_PWM = PWM4;   // PWM4 (breakout label)  -> CH3 output PWM signal
constexpr uint8_t RHJSY_PWM = PWM5;   // PWM5 (breakout label)  -> CH4 output PWM signal
constexpr uint8_t LHTRAV_PWM = PWM6;  // PWM6 (breakout label)  -> CH5 output PWM signal
constexpr uint8_t RHTRAV_PWM = PWM7;  // PWM7 (breakout label)  -> CH6 output PWM signal
constexpr uint8_t BLADEY_PWM = PWM8;  // PWM8 (breakout label)  -> CH7 output PWM signal

// PWM
constexpr uint32_t PWM_FREQ_HZ = 500;
constexpr uint32_t PWM_PERIOD_US = 1000000UL / PWM_FREQ_HZ;  // 2000us

// ADC
// 12-bit reads => range 0..4095. With REFP=3.0V, 1 count ~ 3.0/4095 V.
constexpr int ADC_BITS = 12;
constexpr int ADC_MAX = (1 << ADC_BITS) - 1;

// Read a "settled" ADC value on a given channel.
// Purpose: reduce channel-to-channel carryover when switching ADC mux inputs.
// Method: discard the first conversion after switching channel, then use the second conversion.
static inline int analogReadSettled(uint8_t pin) {
  (void)analogRead(pin);   // discard first conversion after mux switch
  delayMicroseconds(20);   // allow sample/hold to settle; adjust if needed (5..50us typical)
  return analogRead(pin);  // conversion used by the control loop
}

// Linear mapping: ADC counts -> duty cycle (0.0 .. 1.0).
// With REFP=3.0V and VIN constrained to 0..3.0V, this directly yields duty ~= Vin/3.0.
static inline float adcToDutyLinear(int adc) {
  if (adc <= 0)      return 0.0f;
  if (adc >= ADC_MAX) return 1.0f;
  return (float)adc / (float)ADC_MAX;
}

// mbed PWM objects (frequency set via period_us)
static mbed::PwmOut pwm_lhjsx(digitalPinToPinName(LHJSX_PWM));
static mbed::PwmOut pwm_lhjsy(digitalPinToPinName(LHJSY_PWM));
static mbed::PwmOut pwm_rhjsx(digitalPinToPinName(RHJSX_PWM));
static mbed::PwmOut pwm_rhjsy(digitalPinToPinName(RHJSY_PWM));
static mbed::PwmOut pwm_lhtrav(digitalPinToPinName(LHTRAV_PWM));
static mbed::PwmOut pwm_rhtrav(digitalPinToPinName(RHTRAV_PWM));
static mbed::PwmOut pwm_bladey(digitalPinToPinName(BLADEY_PWM));

void setup() {
#if DEBUG_ENABLE
  Serial.begin(BAUD);
  // Debug header: raw ADC counts followed by duty cycles (same channel order).
  Serial.println(F("DBG: LHJSX_raw LHJSY_raw RHJSX_raw RHJSY_raw LHTRAV_raw RHTRAV_raw BLADEY_raw "
                   "LHJSX_duty LHJSY_duty RHJSX_duty RHJSY_duty LHTRAV_duty RHTRAV_duty BLADEY_duty"));
#endif

  // Configure ADC resolution (12-bit: 0..4095).
  analogReadResolution(ADC_BITS);

  // Allow the external reference and analog sources to stabilise before starting control.
  delay(250);

  // Configure PWM frequency (500Hz) on each channel and initialise outputs to 0% duty.
  pwm_lhjsx.period_us(PWM_PERIOD_US);  pwm_lhjsx.write(0.0f);
  pwm_lhjsy.period_us(PWM_PERIOD_US);  pwm_lhjsy.write(0.0f);
  pwm_rhjsx.period_us(PWM_PERIOD_US);  pwm_rhjsx.write(0.0f);
  pwm_rhjsy.period_us(PWM_PERIOD_US);  pwm_rhjsy.write(0.0f);
  pwm_lhtrav.period_us(PWM_PERIOD_US); pwm_lhtrav.write(0.0f);
  pwm_rhtrav.period_us(PWM_PERIOD_US); pwm_rhtrav.write(0.0f);
  pwm_bladey.period_us(PWM_PERIOD_US); pwm_bladey.write(0.0f);
}

void loop() {
  static uint32_t lastDbgMs = 0;

  // Read all analog inputs (settled reads to minimise mux/s&h carryover).
  const int lhjsx_raw  = analogReadSettled(VIN_LHJSX);
  const int lhjsy_raw  = analogReadSettled(VIN_LHJSY);
  const int rhjsx_raw  = analogReadSettled(VIN_RHJSX);
  const int rhjsy_raw  = analogReadSettled(VIN_RHJSY);
  const int lhtrav_raw = analogReadSettled(VIN_LHTRAV);
  const int rhtrav_raw = analogReadSettled(VIN_RHTRAV);
  const int bladey_raw = analogReadSettled(VIN_BLADEY);

  // Convert each channel to a PWM duty cycle (0.0 .. 1.0).
  const float lhjsx_duty  = adcToDutyLinear(lhjsx_raw);
  const float lhjsy_duty  = adcToDutyLinear(lhjsy_raw);
  const float rhjsx_duty  = adcToDutyLinear(rhjsx_raw);
  const float rhjsy_duty  = adcToDutyLinear(rhjsy_raw);
  const float lhtrav_duty = adcToDutyLinear(lhtrav_raw);
  const float rhtrav_duty = adcToDutyLinear(rhtrav_raw);
  const float bladey_duty = adcToDutyLinear(bladey_raw);

  // Update PWM outputs.
  pwm_lhjsx.write(lhjsx_duty);
  pwm_lhjsy.write(lhjsy_duty);
  pwm_rhjsx.write(rhjsx_duty);
  pwm_rhjsy.write(rhjsy_duty);
  pwm_lhtrav.write(lhtrav_duty);
  pwm_rhtrav.write(rhtrav_duty);
  pwm_bladey.write(bladey_duty);

#if DEBUG_ENABLE
  // Periodic debug output (rate-limited).
  const uint32_t nowMs = millis();
  if (nowMs - lastDbgMs >= DEBUG_PERIOD_MS) {
    lastDbgMs = nowMs;

    Serial.print(F("LHJSX_raw="));   Serial.print(lhjsx_raw);   Serial.print(F(" "));
    Serial.print(F("LHJSY_raw="));   Serial.print(lhjsy_raw);   Serial.print(F(" "));
    Serial.print(F("RHJSX_raw="));   Serial.print(rhjsx_raw);   Serial.print(F(" "));
    Serial.print(F("RHJSY_raw="));   Serial.print(rhjsy_raw);   Serial.print(F(" "));
    Serial.print(F("LHTRAV_raw="));  Serial.print(lhtrav_raw);  Serial.print(F(" "));
    Serial.print(F("RHTRAV_raw="));  Serial.print(rhtrav_raw);  Serial.print(F(" "));
    Serial.print(F("BLADEY_raw="));  Serial.print(bladey_raw);  Serial.print(F(" "));

    Serial.print(F("LHJSX_duty="));  Serial.print(lhjsx_duty, 4);  Serial.print(F(" "));
    Serial.print(F("LHJSY_duty="));  Serial.print(lhjsy_duty, 4);  Serial.print(F(" "));
    Serial.print(F("RHJSX_duty="));  Serial.print(rhjsx_duty, 4);  Serial.print(F(" "));
    Serial.print(F("RHJSY_duty="));  Serial.print(rhjsy_duty, 4);  Serial.print(F(" "));
    Serial.print(F("LHTRAV_duty=")); Serial.print(lhtrav_duty, 4); Serial.print(F(" "));
    Serial.print(F("RHTRAV_duty=")); Serial.print(rhtrav_duty, 4); Serial.print(F(" "));
    Serial.print(F("BLADEY_duty=")); Serial.println(bladey_duty, 4);
  }
#endif
}
