#include <Arduino.h>

// ======================================================
// ESP32 - Lecture 4 encodeurs quadrature + USB série
// Robot Mecanum - GA25-370 DC 6V 280 RPM
// Format envoyé :
// rpm_M1,rpm_M2,rpm_M3,rpm_M4,ticks_M1,ticks_M2,ticks_M3,ticks_M4
// ======================================================

#define SERIAL_BAUD 115200
#define SAMPLE_PERIOD_MS 50

// Calibration validée après correction des RPM
#define TICKS_PER_REV 468.0f

#define RPM_ALPHA 0.30f
#define LED_PIN 2

// -------------------- GPIO ENCODEURS ------------------
// M1
#define M1_A 32
#define M1_B 33

// M2
#define M2_A 25
#define M2_B 26

// M3
#define M3_A 27
#define M3_B 14

// M4
#define M4_A 12
#define M4_B 13

// -------------------- SIGNES ENCODEURS ----------------
// Correction pour que rpm_meas ait le même signe que rpm_ref
#define SIGN_M1 -1.0f
#define SIGN_M2  1.0f
#define SIGN_M3 -1.0f
#define SIGN_M4  1.0f

volatile long ticks_m1 = 0;
volatile long ticks_m2 = 0;
volatile long ticks_m3 = 0;
volatile long ticks_m4 = 0;

long last_ticks_m1 = 0;
long last_ticks_m2 = 0;
long last_ticks_m3 = 0;
long last_ticks_m4 = 0;

float rpm_m1 = 0.0f;
float rpm_m2 = 0.0f;
float rpm_m3 = 0.0f;
float rpm_m4 = 0.0f;

unsigned long last_sample_ms = 0;

// -------------------- ISR ENCODEURS -------------------

void IRAM_ATTR isr_m1() {
  bool a = digitalRead(M1_A);
  bool b = digitalRead(M1_B);
  if (a == b) ticks_m1++;
  else        ticks_m1--;
}

void IRAM_ATTR isr_m2() {
  bool a = digitalRead(M2_A);
  bool b = digitalRead(M2_B);
  if (a == b) ticks_m2++;
  else        ticks_m2--;
}

void IRAM_ATTR isr_m3() {
  bool a = digitalRead(M3_A);
  bool b = digitalRead(M3_B);
  if (a == b) ticks_m3++;
  else        ticks_m3--;
}

void IRAM_ATTR isr_m4() {
  bool a = digitalRead(M4_A);
  bool b = digitalRead(M4_B);
  if (a == b) ticks_m4++;
  else        ticks_m4--;
}

// -------------------- OUTILS --------------------------

float computeRPM(long delta_ticks, float dt_s) {
  if (dt_s <= 0.0f) return 0.0f;
  return ((float)delta_ticks / TICKS_PER_REV) * (60.0f / dt_s);
}

void setupEncoderPins() {
  pinMode(M1_A, INPUT_PULLUP);
  pinMode(M1_B, INPUT_PULLUP);

  pinMode(M2_A, INPUT_PULLUP);
  pinMode(M2_B, INPUT_PULLUP);

  pinMode(M3_A, INPUT_PULLUP);
  pinMode(M3_B, INPUT_PULLUP);

  pinMode(M4_A, INPUT_PULLUP);
  pinMode(M4_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_A), isr_m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_A), isr_m2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M3_A), isr_m3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M4_A), isr_m4, CHANGE);
}

// -------------------- SETUP ---------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setupEncoderPins();

  last_sample_ms = millis();

  Serial.println("ESP32_ENCODER_READER_READY");
  Serial.println("Motor: GA25-370 DC 6V 280RPM");
  Serial.println("TICKS_PER_REV=468.0");
  Serial.println("format:rpm_M1,rpm_M2,rpm_M3,rpm_M4,ticks_M1,ticks_M2,ticks_M3,ticks_M4");
}

// -------------------- LOOP ----------------------------

void loop() {
  unsigned long now_ms = millis();

  if (now_ms - last_sample_ms >= SAMPLE_PERIOD_MS) {
    float dt_s = (now_ms - last_sample_ms) / 1000.0f;
    last_sample_ms = now_ms;

    noInterrupts();
    long current_m1 = ticks_m1;
    long current_m2 = ticks_m2;
    long current_m3 = ticks_m3;
    long current_m4 = ticks_m4;
    interrupts();

    long delta_m1 = current_m1 - last_ticks_m1;
    long delta_m2 = current_m2 - last_ticks_m2;
    long delta_m3 = current_m3 - last_ticks_m3;
    long delta_m4 = current_m4 - last_ticks_m4;

    last_ticks_m1 = current_m1;
    last_ticks_m2 = current_m2;
    last_ticks_m3 = current_m3;
    last_ticks_m4 = current_m4;

    float raw_rpm_m1 = SIGN_M1 * computeRPM(delta_m1, dt_s);
    float raw_rpm_m2 = SIGN_M2 * computeRPM(delta_m2, dt_s);
    float raw_rpm_m3 = SIGN_M3 * computeRPM(delta_m3, dt_s);
    float raw_rpm_m4 = SIGN_M4 * computeRPM(delta_m4, dt_s);

    rpm_m1 = (1.0f - RPM_ALPHA) * rpm_m1 + RPM_ALPHA * raw_rpm_m1;
    rpm_m2 = (1.0f - RPM_ALPHA) * rpm_m2 + RPM_ALPHA * raw_rpm_m2;
    rpm_m3 = (1.0f - RPM_ALPHA) * rpm_m3 + RPM_ALPHA * raw_rpm_m3;
    rpm_m4 = (1.0f - RPM_ALPHA) * rpm_m4 + RPM_ALPHA * raw_rpm_m4;

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    Serial.print(rpm_m1, 2); Serial.print(",");
    Serial.print(rpm_m2, 2); Serial.print(",");
    Serial.print(rpm_m3, 2); Serial.print(",");
    Serial.print(rpm_m4, 2); Serial.print(",");

    Serial.print(current_m1); Serial.print(",");
    Serial.print(current_m2); Serial.print(",");
    Serial.print(current_m3); Serial.print(",");
    Serial.println(current_m4);
  }
}
