/*
  Low-Voltage AC Signal Reader & Grapher for ESP32-D
  ----------------------------------------------------
  Signal spec: ~1.5 Vpp, 1.65 V DC offset, 5 Hz

  Since this signal is already within the ESP32's native 0-3.3V ADC
  range and centered near mid-scale, it can be wired DIRECTLY to an
  ADC pin -- no external sensor module or isolation needed (that's
  only required for actual mains voltage).

  Wiring:
    Signal source (+) -> ESP32 GPIO34 (ADC1_CH6)
    Signal source GND -> ESP32 GND
    Optional: 100nF ceramic cap from GPIO34 to GND, right at the pin,
    to filter high-frequency noise (recommended for small signals).

  Graphing:
    Arduino IDE -> Tools -> Serial Plotter, 115200 baud.
    Two lines are plotted: the raw instantaneous waveform (fast) and
    the smoothed RMS value (slow), so you can see both waveform shape
    and a stable amplitude reading.
*/

#include <Arduino.h>

// ---------- SIGNAL PARAMETERS ----------
const int   ADC_PIN            = 35;     // ADC1_CH7
const float ADC_MAX_VALUE      = 4095.0; // 12-bit ADC
const float ADC_VREF           = 3.3;    // ESP32 ADC reference voltage
const float SIGNAL_FREQ_HZ     = 5.0;    // your signal's frequency
const int   SAMPLES_PER_CYCLE  = 200;    // resolution per cycle
const int   CYCLES_TO_AVERAGE  = 2;      // cycles per RMS calculation
const int   OVERSAMPLE_COUNT   = 8;      // ADC reads averaged per sample point

// No external sensor scaling needed for a direct-wired low-voltage signal.
// Set to something other than 1.0 only if you add your own divider/gain stage.
const float CAL_FACTOR = 1.0;

// Smoothing for the displayed RMS value (0.0-1.0, higher = smoother/slower)
const float RMS_SMOOTHING = 0.85;

// ---------- INTERNAL ----------
const int SAMPLE_COUNT = SAMPLES_PER_CYCLE * CYCLES_TO_AVERAGE;
unsigned long sampleIntervalMicros;
float smoothedRms = 0.0;
bool firstRun = true;

void setup() {
  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // full 0-3.3V range

  sampleIntervalMicros = (unsigned long)(1000000.0 / (SIGNAL_FREQ_HZ * SAMPLES_PER_CYCLE));

  Serial.println("Raw_Voltage,RMS_Voltage"); // Serial Plotter header (2 traces)
}

// Oversampled single reading: averages several ADC reads to cut noise
int readOversampled() {
  long sum = 0;
  for (int i = 0; i < OVERSAMPLE_COUNT; i++) {
    sum += analogRead(ADC_PIN);
  }
  return sum / OVERSAMPLE_COUNT;
}

// ---------- REPLACES readRmsVoltage() AND loop() ----------

int sampleBuffer[SAMPLE_COUNT];
int bufferIndex = 0;
unsigned long nextSampleTime = 0;

void loop() {
  // Wait until it's time for the next sample (paces us to SIGNAL_FREQ_HZ * SAMPLES_PER_CYCLE)
  if (nextSampleTime == 0) nextSampleTime = micros();
  while (micros() < nextSampleTime) {
    // precise timing wait
  }
  nextSampleTime += sampleIntervalMicros;

  // Take one real-time sample and print it immediately -- this is the actual waveform
  int rawCounts = readOversampled();
  float rawVolts = (rawCounts / ADC_MAX_VALUE) * ADC_VREF;

  sampleBuffer[bufferIndex] = rawCounts;
  bufferIndex++;

   static int printCounter = 0;
  const int PRINT_EVERY_N = 10; // raise this number to slow the plot further

  printCounter++;
  if (printCounter >= PRINT_EVERY_N) {
    printCounter = 0;
    Serial.print(rawVolts, 3);
    Serial.print(",");
    Serial.println(smoothedRms, 3);
  }
  // Once we've collected a full window, compute RMS over it
  if (bufferIndex >= SAMPLE_COUNT) {
    double sumRaw = 0.0;
    for (int i = 0; i < SAMPLE_COUNT; i++) sumRaw += sampleBuffer[i];
    double dcOffset = sumRaw / SAMPLE_COUNT;

    double sumSquares = 0.0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
      double acValue = sampleBuffer[i] - dcOffset;
      sumSquares += acValue * acValue;
    }

    double rmsCounts = sqrt(sumSquares / SAMPLE_COUNT);
    double rms = (rmsCounts / ADC_MAX_VALUE) * ADC_VREF * CAL_FACTOR;

    if (firstRun) {
      smoothedRms = rms;
      firstRun = false;
    } else {
      smoothedRms = (RMS_SMOOTHING * smoothedRms) + ((1.0 - RMS_SMOOTHING) * rms);
    }

    bufferIndex = 0; // start collecting the next window
  }
}