#include <Arduino.h>
#include <Wire.h>

/* ================== BOS1921 addresses ================== */
static const uint8_t BOS_ADDR      = 0x44;
static const uint8_t REG_REFERENCE = 0x00;  // WFS commands / RAM ACCESS / FIFO
static const uint8_t REG_CONFIG    = 0x05;  // GAIND, PLAY_MODE, OE, RST
static const uint8_t REG_COMM      = 0x0B;  // RDADDR (indirect read target)
static const uint8_t REG_IC_STATUS = 0x10;  // status: STATE, faults, PLAYST
static const uint8_t REG_RAM_DATA  = 0x1B;  // RAM read window (indirect)
static const uint8_t REG_CHIP_ID   = 0x1E;  // expect 0x3781

/* ================== User I/O ================== */
#define AMP_POT_PIN   A0   // amplitude (0..120 Vpk UI, device clamps to ~95 Vpk @ GAIND=0)
#define FREQ_POT_PIN  A1   // frequency (10..500 Hz)
#define PERIOD_POT_PIN A2  // new: controls tap period

/* ===== Tap behavior (tweak ranges here) ===== */
static const float  TAP_MS          = 20.0f;     // duration of each tap in milliseconds
static const bool   HALF_CYCLE_TICK = false;     // true → single half-cycle "tick"
static const float  PERIOD_MIN_MS   = 30.0f;     // pot min = 50 ms
static const float  PERIOD_MAX_MS   = 1000.0f;   // pot max = 2000 ms

// Desired UI ranges
static const float AMP_MIN_V = 0.0f;
static const float AMP_MAX_V = 120.0f;
static const float FREQ_MIN  = 10.0f;   // Hz
static const float FREQ_MAX  = 500.0f;  // Hz

// Update policy (when to accept new pot values)
static const float AMP_STEP_V   = 5.0f;     // reprogram if amplitude changes > 5 V
static const float FREQ_STEP_HZ = 10.0f;    // reprogram if frequency changes > 10 Hz
static const uint32_t UPDATE_MIN_MS = 50;   // don’t re-read too often

// Smoothing (EMA) for pots
static const float EMA_ALPHA = 0.25f;     // 0..1 (higher = snappier)

/* ================== I2C helpers ================== */
void writeReg16(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(BOS_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)((val >> 8) & 0xFF)); // MSB first
  Wire.write((uint8_t)(val & 0xFF));
  Wire.endTransmission();
}

void setReadTarget(uint8_t reg) {
  writeReg16(REG_COMM, (uint16_t)(reg & 0x1F));
}

uint16_t readReg16(uint8_t reg) {
  setReadTarget(reg);
  Wire.requestFrom((int)BOS_ADDR, 2);
  uint8_t msb = Wire.read(), lsb = Wire.read();
  return (uint16_t)((msb << 8) | lsb);
}

/* ================== CONFIG (WFS) ================== */
uint16_t makeCONFIG_WFS(bool oe_on) {
  const uint16_t GAIND     = (0u << 11);
  const uint16_t PLAY_MODE = (0b11u << 9);
  const uint16_t OE        = oe_on ? (1u << 4) : 0u;
  return (GAIND | PLAY_MODE | OE);
}

/* ================== WFS helpers ================== */
inline void wfsWord(uint16_t w) { writeReg16(REG_REFERENCE, w); }

void wfsWrite3(uint16_t addr, uint16_t w1, uint16_t w2, uint16_t w3) {
  wfsWord(0x0001);                                // RAM ACCESS command
  uint16_t rw_addr = (0u << 10) | (addr & 0x03FF);// R/W=0 (write)
  wfsWord(rw_addr);
  wfsWord(w1); wfsWord(w2); wfsWord(w3);
}

/* ================== Field encoders (WFS) ================== */
uint16_t ampCodeFromVpk(float vpk) {
  if (vpk < 0) vpk = 0;
  if (vpk > 95.0f) vpk = 95.0f;
  uint32_t code = (uint32_t)(0.5f + 4095.0f * (vpk / 95.0f));
  if (code > 4095) code = 4095;
  return (uint16_t)code;
}

uint8_t freqCodeFromHz(float hz) {
  if (hz < 3.9f) hz = 3.9f;
  float maxHz = 3.9f * 255.0f;
  if (hz > maxHz) hz = maxHz;
  int c = (int)(0.5f + hz / 3.9f);
  if (c < 1) c = 1; if (c > 255) c = 255;
  return (uint8_t)c;
}

/* ================== Status helpers ================== */
uint16_t readStatus() { return readReg16(REG_IC_STATUS); }

uint8_t state2b() {
  uint16_t s = readStatus();
  uint8_t lo = (uint8_t)(s & 0x3);
  uint8_t hi = (uint8_t)((s >> 8) & 0x3);
  if (hi == 0 || hi == 2 || hi == 3) return hi;
  return lo;
}

bool playDone() { return (readStatus() & 0x0001) != 0; }

bool stopAndWait(uint32_t timeout_ms = 500) {
  wfsWord(0x0012);
  uint16_t word = (0u << 12) | (0u << 8) | (1u << 3) | 0u;
  wfsWord(word);

  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (playDone()) break;
    if (state2b() == 0x0) break;
    delay(1);
  }
  uint32_t t1 = millis();
  while (millis() - t1 < 30) {
    if (state2b() == 0x0) return true;
    delay(1);
  }
  return (state2b() == 0x0);
}

void softResetIfError() {
  if (state2b() == 0x3) {
    writeReg16(REG_CONFIG, (1u << 15));
    delay(1);
    writeReg16(REG_CONFIG, makeCONFIG_WFS(false));
  }
}

/* ================== WFS addresses ================== */
static const uint16_t SLICE_ADDR = 0x0200;
static const uint16_t WAVE0_ADDR = 0x0000;

/* ================== Tap synthesis ================== */
uint8_t cyclesFromDuration(float hz, float ms) {
  float cycles = hz * (ms / 1000.0f);
  if (cycles < 0.0f) cycles = 0.0f;
  if (cycles > 255.0f) cycles = 255.0f;
  return (uint8_t)(cycles + 0.5f);
}

void programAndStart_TAP(float vpk, float hz, float tap_ms = 20.0f, bool halfCycle = false) {
  if (!stopAndWait(500)) {
    softResetIfError();
  }

  writeReg16(REG_CONFIG, makeCONFIG_WFS(false));

  const uint16_t ampl = ampCodeFromVpk(vpk);
  const uint8_t  fq   = freqCodeFromHz(hz);
  uint8_t cycles = halfCycle ? 0 : cyclesFromDuration(hz, tap_ms);

  uint16_t w2 = ((uint16_t)cycles << 8) | fq;
  uint16_t w3 = (0u << 12) | (0x1u << 10) | (halfCycle ? (1u << 9) : 0u);

  wfsWrite3(SLICE_ADDR, ampl, w2, w3);
  wfsWrite3(WAVE0_ADDR, SLICE_ADDR, SLICE_ADDR + 2, 0x0001);

  writeReg16(REG_CONFIG, makeCONFIG_WFS(true));

  wfsWord(0x0012);
  uint16_t payload = (0u << 12) | (0u << 8) | 0x0000;
  wfsWord(payload);
}

/* ================== Pots → params ================== */
float emaAmp = 30.0f;
float emaFreq = 150.0f;
float emaPeriod = 500.0f;
float lastAmp = -1.0f, lastFreq = -1.0f, lastPeriod = -1.0f;
uint32_t lastUpdateMs = 0;

float clampf(float x, float lo, float hi) { if (x < lo) return lo; if (x > hi) return hi; return x; }

void readPotsAndUpdateEMA() {
#if defined(ARDUINO_ARCH_ESP32)
  const int ADC_MAX = 4095;
#else
  const int ADC_MAX = 1023;
#endif
  int rawA = analogRead(AMP_POT_PIN);
  int rawF = analogRead(FREQ_POT_PIN);
  int rawP = analogRead(PERIOD_POT_PIN);

  float ampV  = AMP_MIN_V + (AMP_MAX_V - AMP_MIN_V) * ((float)rawA / (float)ADC_MAX);
  float freqH = FREQ_MIN  + (FREQ_MAX  - FREQ_MIN)  * ((float)rawF / (float)ADC_MAX);
  float perMs = PERIOD_MIN_MS + (PERIOD_MAX_MS - PERIOD_MIN_MS) * ((float)rawP / (float)ADC_MAX);

  ampV  = clampf(ampV,  AMP_MIN_V, AMP_MAX_V);
  freqH = clampf(freqH, FREQ_MIN,  FREQ_MAX);
  perMs = clampf(perMs, PERIOD_MIN_MS, PERIOD_MAX_MS);

  emaAmp    = EMA_ALPHA * ampV  + (1.0f - EMA_ALPHA) * emaAmp;
  emaFreq   = EMA_ALPHA * freqH + (1.0f - EMA_ALPHA) * emaFreq;
  emaPeriod = EMA_ALPHA * perMs + (1.0f - EMA_ALPHA) * emaPeriod;
}

/* ================== Debug helpers ================== */
void printHex16(const char *label, uint16_t v) {
  Serial.print(label); Serial.print("0x");
  if (v < 0x1000) Serial.print("0");
  if (v < 0x0100) Serial.print("0");
  if (v < 0x0010) Serial.print("0");
  Serial.println(v, HEX);
}

/* ================== Setup & Loop ================== */
uint32_t nextTapAt = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  delay(5);

  writeReg16(REG_CONFIG, 0x0000);
  delayMicroseconds(50);

  uint16_t chip = readReg16(REG_CHIP_ID);
  printHex16("CHIP_ID=", chip);

#if defined(analogReadResolution)
  analogReadResolution(12);
#endif

  nextTapAt = millis();
  Serial.println("Tap mode ready: A0=Amplitude, A1=Frequency, A2=Period.");
}

void loop() {
  uint32_t now = millis();
  if (now - lastUpdateMs >= UPDATE_MIN_MS) {
    readPotsAndUpdateEMA();
    lastUpdateMs = now;

    bool ampChanged    = (lastAmp  < 0) || (fabs(emaAmp    - lastAmp)    > AMP_STEP_V);
    bool freqChanged   = (lastFreq < 0) || (fabs(emaFreq   - lastFreq)   > FREQ_STEP_HZ);
    bool periodChanged = (lastPeriod < 0) || (fabs(emaPeriod - lastPeriod) > 20.0f);

    if (ampChanged || freqChanged || periodChanged) {
      Serial.print("→ Amp "); Serial.print(emaAmp, 1);
      Serial.print(" Vpk, Freq "); Serial.print(emaFreq, 1);
      Serial.print(" Hz, Period "); Serial.print(emaPeriod, 0);
      Serial.println(" ms");
      lastAmp = emaAmp;
      lastFreq = emaFreq;
      lastPeriod = emaPeriod;
    }
  }

  if ((int32_t)(now - nextTapAt) >= 0) {
    //programAndStart_TAP(emaAmp, emaFreq, TAP_MS, HALF_CYCLE_TICK);
    //nextTapAt = now + (uint32_t)emaPeriod;
    programAndStart_TAP(70.0f, 250.0f, TAP_MS, HALF_CYCLE_TICK); //hardcoded amplitude for testing
    nextTapAt = now + (uint32_t)100.0f; //hardcoded period for testing
  }

  delay(2);
}
