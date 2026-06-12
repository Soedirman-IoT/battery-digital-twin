#include <ESP32Encoder.h>

#define ENCODER_A 34
#define ENCODER_B 35

ESP32Encoder encoder;

long last_count = 0;
unsigned long last_time = 0;

float rpm = 0;

int PPR = 1000; // sesuai datasheet

void setup() {
  Serial.begin(115200);

  // Attach encoder (quadrature full)
  encoder.attachFullQuad(ENCODER_A, ENCODER_B);

  encoder.clearCount();

  Serial.println("Encoder PCNT ready");
}

void loop() {
  unsigned long now = millis();

  if (now - last_time >= 100) { // update tiap 100 ms
    long count = encoder.getCount();
    long delta = count - last_count;

    // RPM calculation
    rpm = (delta / (float)(PPR * 4)) * (600.0); 
    // 600 karena 100 ms → 60 / 0.1

    // posisi (derajat)
    float position = (count % (PPR * 4)) * (360.0 / (PPR * 4));

    Serial.print("RPM: ");
    Serial.print(rpm);
    Serial.print(" | Posisi: ");
    Serial.println(position);

    last_count = count;
    last_time = now;
  }
}