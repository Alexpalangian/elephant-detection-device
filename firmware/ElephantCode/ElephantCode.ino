#include <WiFi.h>
#include <HTTPClient.h>
#include <arduinoFFT.h>
#include <time.h>
#include <random>
#include <secrets.h>

/*Select ESP32 Dev Module for the board and COM11 for the port*/

//NTP
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET = -25200; // Arizona UTC-7
//FOR KENYA
//const long  GMT_OFFSET = 10800; // Kenya UTC+3

#define ADC_PIN     34
#define SAMPLES     256
#define SAMPLE_DELAY_US 1000  // microseconds between samples → ~1000 Hz

// ─── FFT Buffers ───────────────────────────────────────
double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, 1.0);

// ─── Results ───────────────────────────────────────────
double peakHz        = 0.0;
double peakMagnitude = 0.0;
double sampleRate    = 0.0;

// ───────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    analogSetPinAttenuation(ADC_PIN, ADC_11db);

  WiFi.begin(SSID, PASSWORD);
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

  configTime(GMT_OFFSET, 0, NTP_SERVER);
  struct tm timeinfo;

  //should connect after ~5 seconds
  int attempts = 0;
  while (!getLocalTime(&timeinfo) && attempts < 10) {
    Serial.print(".");
    delay(500);
    attempts++;
  }
  if (attempts >= 10) {
    Serial.println("\nNo NTP");
  } else {
    Serial.println("\nTime synced: " + getTimestamp());
  }
}

void loop() {
    unsigned long startMicros = micros();

    Serial.println(digitalRead(ADC_PIN));

    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (double)analogRead(ADC_PIN);
        vImag[i] = 0.0;

          //Serial.println(vReal[i]); //debug

        delayMicroseconds(SAMPLE_DELAY_US);
    }

    unsigned long elapsedMicros = micros() - startMicros;
    sampleRate = (SAMPLES / (double)elapsedMicros) * 1e6;
    double mean = 0;
    // There is a constant value that is being added to all the values, and then we find the mean, and since the readings are +ve and -ve
    // they cancel each other, and all we have is the constant value multiplied by the number of values. 
    for (int i = 0; i < SAMPLES; i++) {
        mean += vReal[i];
    }
      mean /= SAMPLES; // we divide the total mean by the number of samples, so we get just the constant value.
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] -= mean; // then we subtract the mean from each value to get the raw values.
    }
    FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, sampleRate);

    FFT.dcRemoval();
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    peakHz        = 0.0;
    peakMagnitude = 0.0;

    for (int i = 1; i < SAMPLES / 2; i++) {
        if (vReal[i] > peakMagnitude) {
            peakMagnitude = vReal[i];
        //    Serial.println(i);   //checkng i value
            peakHz        = (i * sampleRate) / SAMPLES;
        }
    }

    sendToInflux(peakHz, peakMagnitude);
}


void sendToInflux(float hz, float magnitude) {
  Serial.printf("Sending %.2f Hz at %.2f magnitude\n", hz, magnitude);
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected");
    return;
  }

  HTTPClient http;
  http.begin(INFLUX_URL);
  http.addHeader("Authorization", String("Token ") + INFLUX_TOKEN);
  http.addHeader("Content-Type", "text/plain; charset=utf-8");

  uint64_t ts = getEpochMs();

  // Format 64-bit timestamp manually (String() can't handle uint64_t)
  char tsStr[25];
  snprintf(tsStr, sizeof(tsStr), "%llu", ts);

  String body = "seismic,sensor=geophone1 hz=" + String(hz, 4) +
                ",magnitude=" + String(magnitude, 4) +
                " " + String(tsStr);

  int code = http.POST(body);

  if (code == 204) {
    Serial.println("Sent to InfluxDB");
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
