#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <random>
#include <secrets.h>

// 1 = run on fake data from playback_data.h instead of the geophone (see playback.h)
// 0 = normal, read the real ADC pin
#define PLAYBACK 0
#include "playback.h"

/*Select ESP32 Dev Module for the board and COM11 for the port*/

//NTP
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET = -25200; // Arizona UTC-7
//FOR KENYA
//const long  GMT_OFFSET = 10800; // Kenya UTC+3

#define ADC_PIN     34
#define SAMPLES     256
#define SAMPLE_DELAY_US 1000  // microseconds between samples → ~1000 Hz

// ─── Converting FFT magnitude to millivolts ────────────
// Samples are now in millivolts (readADC() returns calibrated mV), so the FFT
// works on mV. It isn't normalized, so its magnitude grows with SAMPLES.
// For a steady sine wave of amplitude A mV:
//     magnitude = A * (SAMPLES / 2) * windowGain
// windowGain = average Hamming window weight (~0.54), computed in setup().
// So  A = magnitude / (SAMPLES / 2 * windowGain).
// Exact for steady tones (hum, engine). For short thumps like footsteps it
// UNDERESTIMATES the real peak, because the thump only fills part of the window.
double windowGain = 0.54;

// ─── Clipping detection ────────────────────────────────
// ESP32 ADC at 11 dB is only accurate between ~150 and ~2450 mV (Espressif spec).
// Outside that it compresses and then sticks at the rail. Samples out there are
// counted as "out_of_range". Best DC bias for this range is ~1300 mV, not 1650.
#define ADC_LINEAR_LOW_MV   0
#define ADC_LINEAR_HIGH_MV  2450
// If the amplifier saturates BEFORE the ADC does, the wave still gets flat tops,
// just not at the ADC rails. So we also count samples sitting at the window's
// own min/max. A clean sine has ~13% of samples within 2% of its peaks;
// a clipped one has far more.
#define FLAT_TOL_FRACTION   0.02
#define FLAT_CLIP_PCT       25.0
#define MIN_SWING_FOR_FLAT_MV 100   // ignore flat-top check on tiny/quiet signals

/*
  Sampling never stops. The work is split three ways so the slow WiFi upload
  (~1 s per HTTP request) can't make us miss footsteps:

    loop()      core 1  reads the ADC every 1 ms into one of two buffers.
                        When a buffer fills, it hands it to fftTask and
                        keeps sampling into the other one.
    fftTask     core 0  FFT + peak search on the full buffer (takes a few ms)
    uploadTask  core 0  sends results to InfluxDB. Results that pile up while
                        an upload is in progress go out together in the next one.
*/

// ─── Sample Buffers (double buffered) ──────────────────
uint16_t sampleBuf[2][SAMPLES];   // loop() fills one while fftTask reads the other
int      fillBuf   = 0;           // which buffer loop() is filling
int      fillIdx   = 0;           // next sample position in that buffer
uint32_t nextSampleUs;
uint32_t windowStartUs;

struct Window {                   // loop() -> fftTask
  int      buf;
  uint32_t elapsedUs;             // how long the 256 samples took
  uint64_t endEpochMs;            // when the window finished (for InfluxDB)
  float    playbackT;             // position in the fake recording (playback only)
};

struct Result {                   // fftTask -> uploadTask
  float    hz;
  float    magnitude;
  float    amplitudeMv;           // magnitude converted to millivolts at the ADC pin
  uint64_t epochMs;
  uint16_t vminMv, vmaxMv;        // raw min/max voltage in the window
  uint16_t dcMv;                  // average voltage = DC bias of the circuit
  float    outOfRangePct;         // % samples outside the ADC's accurate range
  float    flatPct;               // % samples stuck at the window's min/max
  bool     clipped;
};

QueueHandle_t windowQueue;
QueueHandle_t resultQueue;
volatile uint32_t windowsDropped = 0;   // should stay 0: fftTask fell behind
volatile uint32_t resultsDropped = 0;   // should stay 0: upload queue overflowed
volatile uint32_t lateSamples    = 0;   // samples taken >1 ms late

// ─── FFT, written by hand (no library) ─────────────────
/*
  The FFT turns 256 voltage samples (time) into 256 frequency "bins".
  Bin k holds how much of the frequency  k * sampleRate / SAMPLES  is in the
  signal (~3.9 Hz per bin at 1000 Hz). Only bins 0..SAMPLES/2 are meaningful;
  the top half is a mirror image.

  This is the classic radix-2 Cooley-Tukey FFT. It needs SAMPLES to be a
  power of 2 (64, 128, 256, 512, ...). It does the same math as a plain DFT
      X[k] = sum over n of  x[n] * e^(-i * 2*pi * k * n / N)
  but in N*log2(N) steps instead of N*N (2,048 vs 65,536 for N = 256).
*/
static_assert((SAMPLES & (SAMPLES - 1)) == 0, "SAMPLES must be a power of 2");

double vReal[SAMPLES];             // real part (starts as the samples)
double vImag[SAMPLES];             // imaginary part (starts as 0)
double hammingW[SAMPLES];          // Hamming window weights, filled in fftSetup()
double twCos[SAMPLES / 2];         // "twiddle factors": cos and -sin of 2*pi*k/N,
double twSin[SAMPLES / 2];         // precomputed so the FFT never calls cos/sin

// Run once in setup(): precompute the window and the twiddle factors.
void fftSetup() {
  double sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    // Hamming window: tapers the ends of the window to ~0.08 so the sudden
    // start/stop of the 256-sample chunk doesn't smear energy into other bins
    hammingW[i] = 0.54 - 0.46 * cos(TWO_PI * i / (SAMPLES - 1));
    sum += hammingW[i];
  }
  windowGain = sum / SAMPLES;      // average window weight (~0.54), used for mV

  for (int k = 0; k < SAMPLES / 2; k++) {
    twCos[k] =  cos(TWO_PI * k / SAMPLES);
    twSin[k] = -sin(TWO_PI * k / SAMPLES);   // minus = forward transform
  }
}

// In-place FFT of re[] / im[] (each SAMPLES long).
void fft(double* re, double* im) {
  // Step 1: reorder the samples by "bit-reversed" index (e.g. for N = 8,
  // index 1 = 001 goes to 100 = 4). This lines them up so step 2 can
  // combine neighbours in place.
  for (int i = 1, j = 0; i < SAMPLES; i++) {
    int bit = SAMPLES >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      double t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }

  // Step 2: "butterflies". Combine pairs of 1-point FFTs into 2-point FFTs,
  // pairs of those into 4-point FFTs, ... until one 256-point FFT is left.
  for (int len = 2; len <= SAMPLES; len <<= 1) {
    int half = len >> 1;
    int step = SAMPLES / len;                  // which twiddle factors this stage uses
    for (int start = 0; start < SAMPLES; start += len) {
      for (int k = 0; k < half; k++) {
        double wr = twCos[k * step];
        double wi = twSin[k * step];
        int a = start + k;
        int b = a + half;
        // t = w * x[b]   (complex multiply)
        double tr = re[b] * wr - im[b] * wi;
        double ti = re[b] * wi + im[b] * wr;
        // x[a], x[b] = x[a] + t, x[a] - t
        re[b] = re[a] - tr;  im[b] = im[a] - ti;
        re[a] += tr;         im[a] += ti;
      }
    }
  }
}

// ───────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    analogSetPinAttenuation(ADC_PIN, ADC_11db);

  // Give WiFi 15 s, then start detecting anyway. The ESP32 keeps retrying in
  // the background, and uploads start working once it connects.
  WiFi.setAutoReconnect(true);
  WiFi.begin(SSID, PASSWORD);
  Serial.println("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nNo WiFi yet, running offline (results only on serial)");
  }

  configTime(GMT_OFFSET, 0, NTP_SERVER);
  struct tm timeinfo;

  //should connect after ~5 seconds
  int attempts = WiFi.status() == WL_CONNECTED ? 0 : 10;   // no WiFi = no clock sync, skip it
  while (attempts < 10 && !getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(500);
    attempts++;
  }
  if (attempts >= 10) {
    Serial.println("\nNo NTP");
  } else {
    Serial.println("\nTime synced: " + getTimestamp());
  }

  fftSetup();   // Hamming window + twiddle factors for the hand-written FFT

  windowQueue = xQueueCreate(1, sizeof(Window));    // 1 = only the other buffer can be waiting
  resultQueue = xQueueCreate(64, sizeof(Result));   // ~16 s of results if WiFi stalls
  xTaskCreatePinnedToCore(fftTask,    "fft",    8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(uploadTask, "upload", 8192, NULL, 1, NULL, 0);

  nextSampleUs  = micros();
  windowStartUs = nextSampleUs;
}

// Only job: take one sample every SAMPLE_DELAY_US, on schedule, forever.
void loop() {
    while ((int32_t)(micros() - nextSampleUs) < 0) { }   // wait for this sample's time slot
    if ((int32_t)(micros() - nextSampleUs) > SAMPLE_DELAY_US) lateSamples++;
    nextSampleUs += SAMPLE_DELAY_US;

    sampleBuf[fillBuf][fillIdx++] = readADC(ADC_PIN);

    if (fillIdx == SAMPLES) {
        uint32_t now = micros();
        Window w = { fillBuf, now - windowStartUs, getEpochMs(), 0 };
#if PLAYBACK
        w.playbackT = playbackTimeS();
#endif
        // Never block here, sampling has to keep going. If fftTask is somehow
        // still busy with the other buffer, this window is lost (and counted).
        if (xQueueSend(windowQueue, &w, 0) == pdTRUE) {
            fillBuf = 1 - fillBuf;
        } else {
            windowsDropped++;
        }
        fillIdx = 0;
        windowStartUs = now;
    }
}

// Same math as before, just running on its own task.
void fftTask(void*) {
    Window w;
    while (true) {
        xQueueReceive(windowQueue, &w, portMAX_DELAY);

        // Raw-signal stats + clipping check, before the FFT touches the data
        const uint16_t* s = sampleBuf[w.buf];
        uint16_t vmin = 65535, vmax = 0;
        uint32_t sum = 0, outOfRange = 0;
        for (int i = 0; i < SAMPLES; i++) {
            uint16_t v = s[i];
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
            sum += v;
            if (v < ADC_LINEAR_LOW_MV || v > ADC_LINEAR_HIGH_MV) outOfRange++;
        }
        uint32_t flat = 0;
        uint16_t swing = vmax - vmin;
        if (swing >= MIN_SWING_FOR_FLAT_MV) {
            uint16_t tol = (uint16_t)(FLAT_TOL_FRACTION * swing / 2.0) + 3;
            for (int i = 0; i < SAMPLES; i++)
                if (s[i] >= vmax - tol || s[i] <= vmin + tol) flat++;
        }
        float outOfRangePct = 100.0f * outOfRange / SAMPLES;
        float flatPct       = 100.0f * flat / SAMPLES;
        bool  clipped       = outOfRange > 0 || flatPct > FLAT_CLIP_PCT;

        double sampleRate = (SAMPLES / (double)w.elapsedUs) * 1e6;

        // Remove DC (subtract the average voltage, so the 1.2 V bias doesn't
        // swamp bin 0) and apply the Hamming window, in one pass
        double mean = (double)sum / SAMPLES;
        for (int i = 0; i < SAMPLES; i++) {
            vReal[i] = ((double)s[i] - mean) * hammingW[i];
            vImag[i] = 0.0;
        }

        fft(vReal, vImag);

        // Magnitude of each bin = length of the complex number (only the
        // first half is needed, the rest is a mirror image)
        for (int i = 0; i <= SAMPLES / 2; i++)
            vReal[i] = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);

    int    peakBin       = 1;
    double peakMagnitude = 0.0;

    for (int i = 1; i < SAMPLES / 2; i++) {
        if (vReal[i] > peakMagnitude) {
            peakMagnitude = vReal[i];
            peakBin       = i;
        }
    }

    // The FFT can only answer in steps of sampleRate/SAMPLES (~3.9 Hz):
    // 3.91, 7.81, 11.72, 15.62, 19.53 ... A real 14 Hz or 17 Hz signal both
    // land in the 15.62 bin. Fit a parabola through the peak bin and its two
    // neighbours (on log magnitude, which is near exact for a Hamming window)
    // to estimate where the true peak sits between bins.
    double binOffset = 0.0;
    if (peakBin > 1 && peakBin < SAMPLES / 2 - 1) {
        double a = log(vReal[peakBin - 1] + 1e-9);
        double b = log(vReal[peakBin]     + 1e-9);
        double c = log(vReal[peakBin + 1] + 1e-9);
        double denom = a - 2.0 * b + c;
        if (denom < 0) {
            binOffset = 0.5 * (a - c) / denom;                     // between -0.5 and +0.5
            peakMagnitude = exp(b - 0.25 * (a - c) * binOffset);   // undo the in-between-bins loss
        }
    }
    double peakHz = (peakBin + binOffset) * sampleRate / SAMPLES;

#if PLAYBACK
    float testT;                                   // where we are in the recording
    const char* test = playbackTest(w.playbackT, &testT);
    Serial.printf("[t=%6.2f s | %s t=%5.2f s] ", w.playbackT, test, testT);
#endif
    double amplitudeMv = peakMagnitude / (SAMPLES / 2.0 * windowGain);
    Serial.printf("%.2f Hz at %.2f magnitude (%.1f mV) | raw %u-%u mV, DC %u mV%s\n",
                  peakHz, peakMagnitude, amplitudeMv, vmin, vmax, (uint16_t)(sum / SAMPLES),
                  clipped ? "  ** CLIPPED **" : "");

    Result r = { (float)peakHz, (float)peakMagnitude, (float)amplitudeMv, w.endEpochMs,
                 vmin, vmax, (uint16_t)(sum / SAMPLES), outOfRangePct, flatPct, clipped };
    if (xQueueSend(resultQueue, &r, 0) != pdTRUE) resultsDropped++;
    }
}

// Waits for results, then sends everything that has piled up in ONE request.
void uploadTask(void*) {
    Result r;
    while (true) {
        xQueueReceive(resultQueue, &r, portMAX_DELAY);
        String body;
        int count = 0;
        do {
            body += influxLine(r);
            count++;
        } while (xQueueReceive(resultQueue, &r, 0) == pdTRUE);
        sendToInflux(body, count);
    }
}

// One InfluxDB line-protocol line, with the time the window was actually recorded
String influxLine(const Result& r) {
  // Format 64-bit timestamp manually (String() can't handle uint64_t)
  char tsStr[25];
  snprintf(tsStr, sizeof(tsStr), "%llu", r.epochMs);

  // fake data gets its own tag so it never mixes with real geophone readings
  return String(PLAYBACK ? "seismic,sensor=playback" : "seismic,sensor=geophone1") +
         " hz=" + String(r.hz, 4) +
         ",magnitude=" + String(r.magnitude, 4) +
         ",amplitude_mv=" + String(r.amplitudeMv, 3) +
         ",vmin_mv=" + String(r.vminMv) + "i" +
         ",vmax_mv=" + String(r.vmaxMv) + "i" +
         ",dc_mv=" + String(r.dcMv) + "i" +
         ",out_of_range_pct=" + String(r.outOfRangePct, 1) +
         ",flat_pct=" + String(r.flatPct, 1) +
         ",clipped=" + String(r.clipped ? 1 : 0) + "i" +
         " " + String(tsStr) + "\n";
}

void sendToInflux(const String& body, int count) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("WiFi disconnected, %d windows not sent\n", count);
    return;
  }

  HTTPClient http;
  http.begin(INFLUX_URL);
  http.addHeader("Authorization", String("Token ") + INFLUX_TOKEN);
  http.addHeader("Content-Type", "text/plain; charset=utf-8");

  int code = http.POST(body);

  if (code == 204) {
    Serial.printf("Sent %d windows to InfluxDB (dropped so far: %u windows, %u results, "
                  "%u late samples)\n", count, windowsDropped, resultsDropped, lateSamples);
  }
  else {
    Serial.println("InfluxDB error: " + String(code));
    Serial.println(http.getString());
  }

  http.end();
}

uint64_t getEpochMs() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return ((uint64_t)tv.tv_sec * 1000ULL) + (tv.tv_usec / 1000);
}

String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "T+" + String(millis()) + "ms";
  }
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}
