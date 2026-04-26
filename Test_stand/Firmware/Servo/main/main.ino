// ======================================================
// Encoder streamer for Servo test stand
// (structure matched to Thrust/main/main.ino)
//
// Serial protocol:
//  - Streams CSV lines: arduino_us,enc_count
//  - Send 'T' over serial to zero the encoder (prints "# TARE_DONE")
// ======================================================

#define Serial SerialUSB

// =========================
// User settings
// =========================
#define STREAM_HZ 50 // output frequency to PC

// Encoder pins - D5 and D6 confirmed interrupt-capable on Arduino M0
const int ENC_A = 5;
const int ENC_B = 6;

volatile long encoderCount = 0;
volatile long encoderOffset = 0;

enum OutputMode : uint8_t {
  OUTPUT_CSV = 0,
  OUTPUT_RAW = 1,
};
volatile OutputMode outputMode = OUTPUT_CSV;

// Timing
unsigned long samplePeriodUs = 1000000UL / STREAM_HZ;
unsigned long lastSampleUs = 0;


void isrEncoderA() {
  // Quadrature decode on CHANGE: compare A and B for direction
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);
  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}


void performTare() {
  noInterrupts();
  encoderOffset = encoderCount;
  interrupts();
  Serial.println("# TARE_DONE");
}

void setOutputMode(OutputMode mode) {
  noInterrupts();
  outputMode = mode;
  interrupts();
  if (mode == OUTPUT_RAW) {
    Serial.println("# MODE_RAW");
  } else {
    Serial.println("# MODE_CSV");
  }
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(5, isrEncoderA, CHANGE);  // D5 confirmed working on this M0

  delay(200);
  Serial.println("# Arduino encoder streamer started");
  Serial.println("# Format (CSV): arduino_us,enc_count");
  Serial.println("# Commands: 'T' zero | 'R' raw mode | 'C' csv mode");
}


void loop() {
  unsigned long nowUs = micros();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'T') {
      performTare();
    } else if (c == 'R') {
      setOutputMode(OUTPUT_RAW);
    } else if (c == 'C') {
      setOutputMode(OUTPUT_CSV);
    }
  }

  if ((nowUs - lastSampleUs) >= samplePeriodUs) {
    lastSampleUs = nowUs;

    long count;
    long offset;
    noInterrupts();
    count = encoderCount;
    offset = encoderOffset;
    interrupts();

    long tared = count - offset;

    OutputMode mode;
    noInterrupts();
    mode = outputMode;
    interrupts();

    if (mode == OUTPUT_RAW) {
      Serial.println(tared);
    } else {
      Serial.print(nowUs);
      Serial.print(",");
      Serial.println(tared);
    }
  }
}
