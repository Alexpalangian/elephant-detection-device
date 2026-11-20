/*
  Playback mode: lets the real firmware run on fake geophone data.
  -----------------------------------------------------------------
  With PLAYBACK 1, readADC() returns samples from playback_data.h instead of
  the ADC pin. Everything else in ElephantCode.ino runs unchanged.

  The recording plays in REAL TIME and loops: readADC() returns whatever
  sample the recording is at "right now". So when the firmware is busy (e.g.
  sending to InfluxDB) it misses samples, exactly like it would with the real
  geophone.

  playback_data.h can hold one test or a playlist of several played back to back.

  Make the data:  python artificial_tests/make_playback_header.py mixed
  With PLAYBACK 0, readADC() is just analogRead() and playback_data.h isn't needed.
*/
#pragma once

#if PLAYBACK
#include "playback_data.h"
#include <esp_timer.h>

int64_t playbackStartUs = -1;

// Seconds into the whole playlist right now
float playbackTimeS() {
  if (playbackStartUs < 0) return 0;
  int64_t elapsed = esp_timer_get_time() - playbackStartUs;
  return (float)((elapsed * PLAYBACK_FS / 1000000) % PLAYBACK_N) / PLAYBACK_FS;
}

// Which test a playlist time falls in, and how far into that test it is.
// Match these against that test's _truth.json
const char* playbackTest(float playlistT, float* testT) {
  uint32_t i = (uint32_t)(playlistT * PLAYBACK_FS);
  int k = 0;
  while (k + 1 < PLAYBACK_TESTS && i >= PLAYBACK_TEST_START[k + 1]) k++;
  *testT = (float)(i - PLAYBACK_TEST_START[k]) / PLAYBACK_FS;
  return PLAYBACK_TEST_NAME[k];
}

int readADC(int pin) {
  (void)pin;
  if (playbackStartUs < 0) {   // first read starts the recording at t = 0
    playbackStartUs = esp_timer_get_time();
    Serial.printf("PLAYBACK: \"%s\", %.1f s at %u Hz, looping\n", PLAYBACK_NAME,
                  (float)PLAYBACK_N / PLAYBACK_FS, PLAYBACK_FS);
  }
  int64_t elapsed = esp_timer_get_time() - playbackStartUs;
  // returns millivolts, same unit as the real readADC() below
  return (int)(PLAYBACK_DATA[(elapsed * PLAYBACK_FS / 1000000) % PLAYBACK_N] * PLAYBACK_MV_PER_COUNT + 0.5);
}

#else
// Calibrated millivolts at the pin. The ESP32 ADC is not linear, so
// analogRead() * 3300/4095 is off by 100+ mV near the top of the range.
// analogReadMilliVolts() uses the chip's factory calibration instead.
int readADC(int pin) { return analogReadMilliVolts(pin); }
#endif
