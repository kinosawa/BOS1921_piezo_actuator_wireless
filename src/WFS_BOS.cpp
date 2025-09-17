#include <Arduino.h>
#include <Wire.h>

// ============ USER PINS (CHANGE TO YOUR ADC PINS) ============
#define AMP_POT_PIN   A0
#define FREQ_POT_PIN  A1

// ============ BOS1921 I2C ============
static const uint8_t BOS_ADDR      = 0x44;
static const uint8_t REG_REFERENCE = 0x00;  // command/data ingress (RAM ACCESS & RAM SYNTH)
static const uint8_t REG_CONFIG    = 0x05;  // GAIND, PLAY_MODE, OE

// ============ RANGES & UPDATE POLICY ============
static const float AMP_MIN_V = 0.0f;
static const float AMP_MAX_V = 60.0f;   // unipolar Vpk
static const float FREQ_MIN  = 10.0f;   // Hz
static const float FREQ_MAX  = 500.0f;  // Hz

static const float AMP_STEP_V   = 0.5f;   // print + reprogram if amplitude changes > 0.5 V
static const float FREQ_STEP_HZ = 2.0f;   // print + reprogram if frequency changes > 2 Hz
static const uint32_t UPDATE_MIN_MS = 40; // don’t reprogram more often than this

// ============ LOW-LEVEL I2C (16-bit, MSB-first on writes) ============
void writeReg16(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(BOS_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)((val >> 8) & 0xFF)); // MSB
  Wire.write((uint8_t)(val & 0xFF));        // LSB
  Wire.endTransmission();
}
inline void wfsWord(uint16_t w) { writeReg16(REG_REFERENCE, w); }

// ============ CONFIG (PLAY_MODE = WFS, GAIND = ±95 V, OE on/off) ============
uint16_t makeCONFIG_WFS(bool oe_on) {
  const uint16_t GAIND     = (0u << 11);     // ±95 V range
  const uint16_t PLAY_MODE = (0b11u << 9);   // 0b11 = RAM Synthesis (WFS)
  const uint16_t OE        = oe_on ? (1u << 4) : 0u;
  return (GAIND | PLAY_MODE | OE);
}

// ============ WFS RAM HELPERS (*** R/W is BIT 10 ***) ============
void wfsWrite3(uint16_t addr, uint16_t w1, uint16_t w2, uint16_t w3) {
  wfsWord(0x0001);                                   // RAM ACCESS command
  uint16_t rw_addr = (0u << 10) | (addr & 0x03FF);   // R/W=0 (write) at bit 10, address in [9:0]
  wfsWord(rw_addr);
  wfsWord(w1); wfsWord(w2); wfsWord(w3);
}

// ============ FIELD ENCODERS (WFS RAM Synthesis) ============
// Unipolar amplitude code (GAIND=0): AMPLITUDE = round(4095 * Vpk / 95)
uint16_t ampCodeFromVpk(float vpk) {
  if (vpk < 0) vpk = 0;
  if (vpk > 95.0f) vpk = 95.0f;
  uint32_t code = (uint32_t)(0.5f + 4095.0f * (vpk / 95.0f));
  if (code > 4095) code = 4095;
  return (uint16_t)code;
}
// Frequency code: f_out ≈ 3.9 * code  (code 1..255)
uint8_t freqCodeFromHz(float hz) {
  if (hz < 3.9f) hz = 3.9f;
  float maxHz = 3.9f * 255.0f;
  if (hz > maxHz) hz = maxHz;
  int c = (int)(0.5f + hz / 3.9f);
  if (c < 1) c = 1; if (c > 255) c = 255;
  return (uint8_t)c;
}

// ============ PROGRAM & START (one unipolar continuous sine) ============
static const uint16_t SLICE_ADDR = 0x0200;  // SLICE at 0x0200..0x0202
static const uint16_t WAVE0_ADDR = 0x0000;  // WAVE #0 at  0x0000..0x0002

void startSineUnipolar_WFS(float vpk, float hz) {
  // 1) Enter WFS with OE=0 (don’t write RAM while playing)
  writeReg16(REG_CONFIG, makeCONFIG_WFS(false));

  // 2) SLICE: AMPL, CYCLES/FREQ, FLAGS (CONT=1, MODE=01 unipolar+)
  uint16_t w1 = ampCodeFromVpk(vpk);
  uint16_t w2 = ((uint16_t)0x00 << 8) | freqCodeFromHz(hz); // CYCLES ignored (CONT=1)
  uint16_t w3 = (1u << 12) | (0x1u << 10);                  // CONT=1, MODE=01, no ramps
  wfsWrite3(SLICE_ADDR, w1, w2, w3);

  // 3) WAVE #0 → SLICE span, COUNT=0 (repeat forever)
  wfsWrite3(WAVE0_ADDR, SLICE_ADDR, SLICE_ADDR + 2, 0x0000);

  // 4) Enable output and start synthesis (START=END=WAVE#0, RPT=1)
  writeReg16(REG_CONFIG, makeCONFIG_WFS(true));   // OE=1
  wfsWord(0x0012);                                // RAM SYNTHESIS
  uint16_t payload = (0u << 12) | (0u << 8) | 0x0001; // END=0, START=0, RPT=1
  wfsWord(payload);
}

// ============ UTILITIES ============
float clampf(float x, float lo, float hi) { if (x < lo) return lo; if (x > hi) return hi; return x; }

// Simple smoothing (EMA)
static const float EMA_ALPHA = 0.25f; // 0..1 (higher = snappier)
float emaAmp = 0, emaFreq = 0;
float lastAmp = -1.0f, lastFreq = -1.0f;
uint32_t lastUpdateMs = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);  // keep I2C conservative & stable

#if defined(analogReadResolution)
  analogReadResolution(12); // ESP32-S3: 0..4095; on AVR it’ll be 0..1023
#endif

  // Start with an audible baseline and print it
  startSineUnipolar_WFS(30.0f, 150.0f);
  emaAmp = lastAmp = 30.0f;
  emaFreq = lastFreq = 150.0f;
  Serial.println("BOS1921 WFS pot-test ready.");
  Serial.println("Turn the knobs; I will print and apply changes.");
}

void loop() {
#if defined(ARDUINO_ARCH_ESP32)
  const int ADC_MAX = 4095;
#else
  const int ADC_MAX = 1023;
#endif

  // Read pots
  int rawA = analogRead(AMP_POT_PIN);
  int rawF = analogRead(FREQ_POT_PIN);

  // Map to engineering units
  float ampV  = AMP_MIN_V + (AMP_MAX_V - AMP_MIN_V) * ((float)rawA / (float)ADC_MAX);
  float freqH = FREQ_MIN  + (FREQ_MAX  - FREQ_MIN)  * ((float)rawF / (float)ADC_MAX);
  ampV  = clampf(ampV,  AMP_MIN_V, AMP_MAX_V);
  freqH = clampf(freqH, FREQ_MIN,  FREQ_MAX);

  // Smooth
  emaAmp  = EMA_ALPHA * ampV  + (1.0f - EMA_ALPHA) * emaAmp;
  emaFreq = EMA_ALPHA * freqH + (1.0f - EMA_ALPHA) * emaFreq;

  // Update policy
  uint32_t now = millis();
  bool timeOK = (now - lastUpdateMs) >= UPDATE_MIN_MS;
  bool ampChanged  = (lastAmp  < 0) || (fabs(emaAmp  - lastAmp)  > AMP_STEP_V);
  bool freqChanged = (lastFreq < 0) || (fabs(emaFreq - lastFreq) > FREQ_STEP_HZ);

  if (timeOK && (ampChanged || freqChanged)) {
    // Printouts for changing inputs (as requested)
    Serial.print("→ Amp ");
    Serial.print(emaAmp, 1);
    Serial.print(" Vpk, Freq ");
    Serial.print(emaFreq, 1);
    Serial.println(" Hz");

    // Apply to WFS
    startSineUnipolar_WFS(emaAmp, emaFreq);

    // Latch state
    lastUpdateMs = now;
    lastAmp = emaAmp;
    lastFreq = emaFreq;
  }

  delay(5);
}
