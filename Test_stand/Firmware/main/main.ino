#include "HX711.h"

// =========================
// User settings
// =========================
#define USE_SECOND_LOADCELL 0   // 1 = use both, 0 = use only first
#define STREAM_HZ 100           // output frequency to PC
#define AVG_COUNT 4             // number of HX711 reads to average

// HX711 #1 pins
const int HX1_DOUT = 2;
const int HX1_SCK  = 3;

// HX711 #2 pins
const int HX2_DOUT = 4;
const int HX2_SCK  = 5;

// Load cells offset (measure only EDF trust)
long offset1 = 0;
long offset2 = 0;

// =========================
// HX711 objects
// =========================
HX711 scale1;
HX711 scale2;

// Timing
unsigned long samplePeriodUs = 1000000UL / STREAM_HZ;
unsigned long lastSampleUs = 0;


////////////////////////////////////////////////////////
/* Functions */
long readAverage(HX711 &scale, int avgCount) 
{
  long sum = 0;
  int validReads = 0;

  for (int i = 0; i < avgCount; i++) {
    unsigned long start = micros();
    while (!scale.is_ready()) {
      if ((micros() - start) > 5000) { // 5 ms timeout per try
        break;
      }
    }

    if (scale.is_ready()) {
      sum += scale.read();
      validReads++;
    }
  }

  if (validReads == 0) {
    return 0;
  }

  return sum / validReads;
}



void performTare() 
{
  const int N = 50;

  long sum1 = 0;
  long sum2 = 0;

  for (int i = 0; i < N; i++) {

    while (!scale1.is_ready()) {}
    sum1 += scale1.read();

#if USE_SECOND_LOADCELL
    while (!scale2.is_ready()) {}
    sum2 += scale2.read();
#endif

    delay(5);
  }

  offset1 = sum1 / N;

#if USE_SECOND_LOADCELL
  offset2 = sum2 / N;
#endif

  Serial.println("# TARE_DONE");
}


///////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  scale1.begin(HX1_DOUT, HX1_SCK);

#if USE_SECOND_LOADCELL
  scale2.begin(HX2_DOUT, HX2_SCK);
#endif

  delay(1000);

  Serial.println("# Arduino HX711 streamer started");
  Serial.println("# Format: arduino_us,load1,load2");
}


///////////////////////////////////////////////////////////////
void loop() {
  unsigned long nowUs = micros();

  if (Serial.available()) {

    char c = Serial.read();

    if (c == 'T') {
      performTare();
    }
  }

  if ((nowUs - lastSampleUs) >= samplePeriodUs) {
    lastSampleUs = nowUs;

    long load1 = 0;
    long load2 = 0;

    if (scale1.is_ready()) {
      load1 = readAverage(scale1, AVG_COUNT) - offset1;
    }

#if USE_SECOND_LOADCELL
    if (scale2.is_ready()) {
      load2 = readAverage(scale2, AVG_COUNT) - offset2;
    }
#else
    load2 = 0;
#endif

    // CSV line: timestamp_us,load1,load2
    Serial.print(nowUs);
    Serial.print(",");
    Serial.print(load1);
    Serial.print(",");
    Serial.println(load2);
  }
}